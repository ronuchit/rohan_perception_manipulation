#! /usr/bin/python
import roslib
roslib.load_manifest("workspace")

import rospy
import actionlib
import tabletop_actions.object_detector as object_detector
import pr2_control_utilities
import tf 
from tf import transformations
import numpy as np
import openravepy
import pdb
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from arm_navigation_msgs.msg import *
from control_msgs.msg import FollowJointTrajectoryAction
import sys

def create_openrave_bodies(env):
	""" Detects an object and creates corresponding OpenRave body in environment env with specified name.
			Returns the created OpenRave body. """	

	robot_state = pr2_control_utilities.RobotState()
	joint_mover = pr2_control_utilities.PR2JointMover(robot_state)
	detector = object_detector.GenericDetector()		
	
	rospy.loginfo("Attempting to detect an object.")
	"""detector.search_for_object(joint_mover, trials=15,	cluster_choser=detector.find_biggest_cluster,	max_pan=0.5, min_pan=-0.5, max_tilt = 1.1, min_tilt = 0.6)			
				
	res = detector.last_detection_msg
	for i in range(0, 10):
		rospy.loginfo("Detection trial #" + str(i))
		res = detector.detect()
		if res is not None:
			break;
		
	pdb.set_trace()
	print("Clusters: " + len(res.detection.cluster) + "\n" + repr(res.detection.clusters))
	
	if res is None:
		rospy.logerr("No object detected.")
		return None
	
	rospy.loginfo("Object detected.")
	cluster = detector.find_biggest_cluster(res.detection.clusters)"""
	
	clusters, table = detector.find_all_clusters(trials=2)

	for i in range(len(clusters)):
		cluster = clusters[i]
		
		box_msg = detector.detect_bounding_box(cluster) # a FindClusterBoundingBoxResponse message
		
		if box_msg.error_code != box_msg.SUCCESS:
			rospy.logerr("No bounding box detected for object " + repr(i))
			return None
		
		rospy.loginfo("Bounding box detected for object " + repr(i))
		values = [0,0,0, box_msg.box_dims.x, box_msg.box_dims.y, box_msg.box_dims.z]
		body = openravepy.RaveCreateKinBody(env,'')
		body.SetName("object" + repr(i))
				
		body.InitFromBoxes(np.array([values]), True)
		T = body.GetTransform()
		T[:,-1] = [box_msg.pose.pose.position.x, box_msg.pose.pose.position.y, box_msg.pose.pose.position.z, 1]
		body.SetTransform(T)
				
		env.Add(body, True)
		rospy.loginfo("Body " + repr(i) + " added to env.")
		
	add_table(table, env)
	rospy.loginfo("Table added to env.")
	
	return body


def add_table(table, env):    
    body = openravepy.RaveCreateKinBody(env,'')
    body.SetName('table')
    z = table.pose.pose.position.z
    x_min = table.x_min
    x_max = table.x_max
    y_min = table.y_min
    y_max = table.y_max
    
    x = (x_max-x_min)/2 + x_min
    y = (y_max-y_min)/2 + y_min
    dim_x = (x_max - x_min)/2 + 0.1
    dim_y = (y_max - y_min)/2 + 0.1
    body.InitFromBoxes(np.array([[x, y, z, dim_x, dim_y, 0.06]]), True)
    env.Add(body, True)
    
    return body
    
def move_link(client_name, group_name, link_name, end_pose):
  '''action_name = 'tuck_arms'
  #rospy.init_node(action_name)
  rospy.sleep(0.001)  # wait for time
  #tuck_arms_action_server = tuck_arms_main.TuckArmsActionServer(action_name)

  goal = tuck_arms_main.TuckArmsGoal()
  goal.tuck_left = True
  goal.tuck_right = True

  tuck_arm_client = actionlib.SimpleActionClient(action_name, tuck_arms_main.TuckArmsAction)
  rospy.loginfo('Waiting for action server to start')
  tuck_arm_client.wait_for_server(rospy.Duration(10.0))
  rospy.loginfo('Sending goal to action server')
  tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

  #rospy.spin()'''
    
  move_arm = actionlib.SimpleActionClient(client_name, MoveArmAction)
  rospy.loginfo("Waiting for server...")
  move_arm.wait_for_server()
  rospy.loginfo("Found server.")
  goal = MoveArmGoal()

  goal.motion_plan_request.group_name = group_name
  goal.motion_plan_request.num_planning_attempts = 1
  goal.motion_plan_request.planner_id = ""
  goal.planner_service_name = "ompl_planning/plan_kinematic_path"
  goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
  
  desired_pose = SimplePoseConstraint()
  desired_pose.header.frame_id = "torso_lift_link"
  desired_pose.link_name = link_name
  desired_pose.pose.position.x = end_pose[0]
  desired_pose.pose.position.y = end_pose[1]
  desired_pose.pose.position.z = end_pose[2]

  desired_pose.pose.orientation.x = end_pose[3]
  desired_pose.pose.orientation.y = end_pose[4]
  desired_pose.pose.orientation.z = end_pose[5]
  desired_pose.pose.orientation.w = end_pose[6]

  desired_pose.absolute_position_tolerance.x = 0.02
  desired_pose.absolute_position_tolerance.y = 0.02
  desired_pose.absolute_position_tolerance.z = 0.02

  desired_pose.absolute_roll_tolerance = 0.04
  desired_pose.absolute_pitch_tolerance = 0.04
  desired_pose.absolute_yaw_tolerance = 0.04
  
  addGoalConstraintToMoveArmGoal(desired_pose, goal)

  rospy.loginfo("Sending goal.")
  move_arm.send_goal(goal)
  move_arm.wait_for_result()



def poseConstraintToPositionOrientationConstraints(pose_constraint):
	position_constraint = PositionConstraint()
	orientation_constraint = OrientationConstraint()
	position_constraint.header = pose_constraint.header
	position_constraint.link_name = pose_constraint.link_name
	position_constraint.position = pose_constraint.pose.position
	position_constraint.constraint_region_shape.type = 0
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

	position_constraint.constraint_region_orientation.x = 0.0
	position_constraint.constraint_region_orientation.y = 0.0
	position_constraint.constraint_region_orientation.z = 0.0
	position_constraint.constraint_region_orientation.w = 1.0

	position_constraint.weight = 1.0

	orientation_constraint.header = pose_constraint.header
	orientation_constraint.link_name = pose_constraint.link_name
	orientation_constraint.orientation = pose_constraint.pose.orientation
	orientation_constraint.type = pose_constraint.orientation_constraint_type

	orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
	orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
	orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
	orientation_constraint.weight = 1.0
	 
	return (position_constraint, orientation_constraint)
 
 
def addGoalConstraintToMoveArmGoal(pose_constraint, move_arm_goal):
	position_constraint, orientation_constraint = poseConstraintToPositionOrientationConstraints(pose_constraint)
	move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
	move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)	
  
if __name__ == "__main__":
	rospy.init_node("workspace", anonymous=True)

	l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407]
	r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436]
	#move_link("move_right_arm", "right_arm", "r_wrist_roll_link", r_arm_tucked) # tuck right arm
	#move_link("move_left_arm", "left_arm", "l_wrist_roll_link", l_arm_tucked) # tuck left arm
	rospy.loginfo("Arms tucked.")
	
	env = openravepy.Environment() # load new OpenRave environment
	#env.Load("bare_bone.dae")
	
	create_openrave_bodies(env)
	
	env.Save("created_info.dae")
