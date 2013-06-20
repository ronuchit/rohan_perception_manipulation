import roslib
roslib.load_manifest("openrave_bridge")
import rospy
import openravepy
from openravepy.databases.inversekinematics import InverseKinematicsModel
import tf
from tf import transformations

import pr2_control_utilities
from geometry_msgs.msg import PoseStamped
import numpy as np

from openrave_bridge import utils


pr2_l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, 
                    -0.0962141, -0.0864407]

pr2_r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, 
                    -1.4175, -1.8417, 0.21436]


class PR2Robot(object):
    def __init__(self, env = None):
        if env == None:
            self.env = openravepy.Environment()
            self.robot = self.env.ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml")
            self.env.Add(self.robot)
        
        elif len(env.GetRobots())>0:
                self.env = env
                self.robot = env.GetRobots()[0]
        else:
            rospy.loginfo("Valid env not found")
            sys.exit()
             
        
        rospy.loginfo("Loading IK for rightarm")
        self.robot.SetActiveManipulator("rightarm")
        self.rightarm_ik = InverseKinematicsModel(self.robot, 
                                                  iktype=openravepy.IkParameterization.Type.Transform6D)
        if not self.rightarm_ik.load():
            self.rightarm_ik.autogenerate()
            
        rospy.loginfo("Loading IK for leftarm")
        self.robot.SetActiveManipulator("leftarm")
        self.leftarm_ik = InverseKinematicsModel(self.robot, 
                                                 iktype=openravepy.IkParameterization.Type.Transform6D)
        if not self.leftarm_ik.load():
            self.leftarm_ik.autogenerate()
            
        self.robot_state = pr2_control_utilities.RobotState()
        self.controller = pr2_control_utilities.PR2JointMover(robot_state = self.robot_state,
                                                              name = "PR2 Controller",
                                                              time_to_reach=5.0
                                                              )
        self.listener = tf.TransformListener()
        
        #fixing the joints
        joint_msg = self.robot_state.last_joint_msg
        ros_names = joint_msg.name
        inds_ros2rave = np.array([self.robot.GetJointIndex(name) for name in ros_names])
        self.good_ros_inds = np.flatnonzero(inds_ros2rave != -1) # ros joints inds with matching rave joint
        self.rave_inds = inds_ros2rave[self.good_ros_inds] # openrave indices corresponding to those joints
        
        # make the joint limits match the PR2 soft limits
        low_limits, high_limits = self.robot.GetDOFLimits()
        rarm_low_limits = [-2.1353981634, -0.3536, -3.75, -2.1213, None, -2.0, None]
        rarm_high_limits = [0.564601836603, 1.2963, 0.65, -0.15, None, -0.1, None]
        for rarm_index, low, high in zip(self.robot.GetManipulator("rightarm").GetArmIndices(),
                                         rarm_low_limits, rarm_high_limits):
            if low is not None and high is not None:
                low_limits[rarm_index] = low
                high_limits[rarm_index] = high
        larm_low_limits = [-0.564601836603, -0.3536, -0.65, -2.1213, None, -2.0, None]
        larm_high_limits = [2.1353981634, 1.2963, 3.75, -0.15, None, -0.1, None]
        for larm_index, low, high in zip(self.robot.GetManipulator("leftarm").GetArmIndices(),
                                         larm_low_limits, larm_high_limits):
            if low is not None and high is not None:
                low_limits[larm_index] = low
                high_limits[larm_index] = high
        self.robot.SetDOFLimits(low_limits, high_limits)
        
    
    def convertPose(self, pose):
        assert isinstance(pose, PoseStamped)
        self.listener.waitForTransform("/base_footprint", pose.header.frame_id,
                                       rospy.Time.now(), rospy.Duration(1))
        newpose = self.listener.transformPose("/base_footprint", pose)        
        translation = tf.listener.xyz_to_mat44(newpose.pose.position), 
        orientation = tf.listener.xyzw_to_mat44(newpose.pose.orientation)
        
        matrix4 = np.dot(translation, orientation).squeeze()
        return matrix4
    
    def __ik_solution(self, pose, manip, end_effector_link, ignore_end_effector=True,
                      multiple_soluitions = False):
        T = self.convertPose(pose)
        self.update_rave()
        worldFromEE = utils.tf_for_link(T, manip, end_effector_link)
        filter_options = openravepy.IkFilterOptions.CheckEnvCollisions
        if ignore_end_effector:
            filter_options = filter_options | openravepy.IkFilterOptions.IgnoreEndEffectorCollisions
        
        if multiple_soluitions:
            sol = manip.FindIKSolutions(worldFromEE, filter_options)
        else:
            sol = manip.FindIKSolution(worldFromEE, filter_options)
        return sol
    
    def find_rightarm_ik(self, pose, ignore_end_effector=True,
                      multiple_soluitions = False):
        manip = self.robot.SetActiveManipulator("rightarm")
        return self.__ik_solution(pose, manip, "r_wrist_roll_link",
                                  ignore_end_effector, 
                                  multiple_soluitions)
    
    def find_leftarm_ik(self, pose, ignore_end_effector=True,
                      multiple_soluitions = False):
        manip = self.robot.SetActiveManipulator("leftarm")
        return self.__ik_solution(pose, manip, "l_wrist_roll_link",
                                  ignore_end_effector,
                                  multiple_soluitions)    
    
    def move_right_arm(self, pose, ignore_end_effector = True):
        sol = self.find_rightarm_ik(pose, ignore_end_effector)
        if sol is None:
            rospy.logerr("Could not find an IK solution!")
            return False
        self.controller.set_arm_state(sol.tolist(), "right", True)
        self.update_rave()
        return True        
    
    def move_left_arm(self, pose, ignore_end_effector = True):
        sol = self.find_leftarm_ik(pose, ignore_end_effector)
        if sol is None:
            rospy.logerr("Could not find an IK solution!")
            return False
        self.controller.set_arm_state(sol.tolist(), "left", True)
        self.update_rave()
        return True
        
    def update_rave(self):
        ros_values = self.robot_state.last_joint_msg.position
        rave_values = [ros_values[i_ros] for i_ros in self.good_ros_inds]
        self.robot.SetJointValues(rave_values[:20],self.rave_inds[:20])
        self.robot.SetJointValues(rave_values[20:],self.rave_inds[20:])
        
        
    def tuck_left_arm(self):
        self.controller.set_arm_state(pr2_l_arm_tucked, "left", True)
        self.update_rave()
        
    def tuck_right_arm(self):
        self.controller.set_arm_state(pr2_r_arm_tucked, "right", True)
        self.update_rave()
        
    def tuck_arms(self):
        self.tuck_left_arm()
        self.tuck_right_arm()
