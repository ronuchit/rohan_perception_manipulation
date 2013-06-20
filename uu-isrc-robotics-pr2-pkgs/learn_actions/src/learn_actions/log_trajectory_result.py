import roslib
roslib.load_manifest('learn_actions')
import rospy
import tf
from learn_actions.msg import ObjectTrajectoryResults
from tabletop_object_detector.srv import TabletopDetectionResponse
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessingResponse
from geometry_msgs.msg import Point
from pr2_control_utilities import utils
from pr2_control_utilities import RobotState
from std_srvs.srv import Empty
import threading

class LogTrajectoryResult(object):
    """
    First observes the scene, memorizing all the objects there. It then executes
    a trajectory (using the trajectory service provided in the package
    pr2_trajectory_markers), then it observes again the scene, re-memorizing the
    objects in it.
    The data collected is used to publish a ObjectTrajectoryResult msg.

    It works with the PR2TrajectoryMarkers class in the pr2_trajectory_markers
    package by invoking it execute_trajectory service.

    Constructor paramters:
    detector: an ObjectDetector instance
    joint_mover: a Pr2JointMover instance
    trajectory_service: used to execute a trajectory (see PR2TrajectoryMarkers)
    whicharm: "left" | "right" which arm to use.
    frame_to_use: a frame relative to the robot (not external)
    tf_listner: a TransformListener instance
    """

    def __init__(self, detector, joint_mover,
            trajectory_service,
            whicharm,
            planner,
            frame_to_use = "base_link",
            tf_listener = None):

        if whicharm not in ("right", "left"):
            raise ValueError("Wrong arm name: %s", whicharm)

        self.detector = detector
        self.joint_mover = joint_mover
        self.frame = frame_to_use
        self.planner = planner
        self.whicharm = whicharm
        self.robot_state = RobotState()

        self.object_pose = None
        if tf_listener is None:
            self.tf_listner = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.obj_changes_pub = rospy.Publisher("trajectory_effect",
                ObjectTrajectoryResults)

        rospy.loginfo("Waiting for %s service", trajectory_service)
        rospy.wait_for_service(trajectory_service)
        self.execute_trajectory = rospy.ServiceProxy(trajectory_service, Empty)

        #initializing the fillin strucure
        self.pub_msg= ObjectTrajectoryResults()
        if whicharm == "left":
            self.pub_msg.whicharm = ObjectTrajectoryResults.LEFT
            self.gripper_tracking_func = self.planner.get_left_gripper_pose
        else:
            self.pub_msg.whicharm = ObjectTrajectoryResults.RIGHT
            self.gripper_tracking_func = self.planner.get_right_gripper_pose

        self.__keep_tracking = False
        rospy.loginfo("LogPushingResult is ready")

    def search_object(self):
        """
        Searches for objects using the detector.

        Returns:
        a list of (pos, dims, names) where pos is a PoseStamped(converted in the self.frame fame with
        the object pose if the object is found), dims is a Vector3 specifing the x,y,z dimensions of
        the box and names are the names of the objects.

        None if no object is found
        """
        res = self.detector.detect()
        if not res:
            return None

        coll_res = self.detector.call_collision_map_processing(res)

        isinstance(coll_res, TabletopCollisionMapProcessingResponse)

        rospy.loginfo("%d objects found, changing their poses into frame %s",
                      len(coll_res.graspable_objects),self.frame)

        poses = []
        dims = []

        for graspable in coll_res.graspable_objects:
            cluster = graspable.cluster
            box_pose = self.detector.detect_bounding_box(cluster).pose
            self.tf_listener.waitForTransform(self.frame,
                                              box_pose.header.frame_id,
                                              rospy.Time(0),
                                              rospy.Duration(1)
                                              )
            object_pose = self.tf_listener.transformPose(self.frame, box_pose)
            poses.append(object_pose)
            dims.append(self.detector.last_box_msg.box_dims)

        return poses, dims, coll_res.collision_object_names


    def __track_gripper(self, poses, grippers):
        sleeper = rospy.Rate(100)
        if self.whicharm == "left":
            gripper_open = lambda: self.robot_state.l_gripper_positions[0]
        else:
            gripper_open = lambda: self.robot_state.r_gripper_positions[0]

        while self.__keep_tracking:
            pos = self.gripper_tracking_func(frame=self.frame)
            pos.header.stamp = rospy.Time.now()
            poses.append(pos)
            grippers.append(gripper_open())
            sleeper.sleep()


    def publish_object_changes(self):
        """
        Searches for an object using search_object and, if it is found, executes
        a trajectory (via an external node) monitoring the gripper pose. Finally
        it published a ObjectTrajectoryResult msg with the pre position of the
        object, the post position and the trajectory itself.
        """
        res = self.search_object()
        if res is None:
            rospy.logerr("No object found!")
            return

        poses, dims, names = res
        self.pub_msg.pre_movement_object_poses =[p for p in poses]
        self.pub_msg.pre_movement_box_dims = [d for d in dims]
        self.pub_msg.pre_object_names = names

        if self.whicharm == "right":
            pre_arm_pose = self.joint_mover.robot_state.right_arm_positions
        else:
            pre_arm_pose = self.joint_mover.robot_state.left_arm_positions


        #preparing to start the tracking thread
        poses = []
        grippers = []
        t = threading.Thread(target = self.__track_gripper, args = (poses,grippers))
        self.__keep_tracking = True
        error_occurred = False
        t.start()
        #this is a blocking call, that's why we need a thread
        try:
            self.execute_trajectory()
        except:
            rospy.logerr("Error while communicating with the trajectory service")
            error_occurred = True
        finally:
            self.__keep_tracking = False
            t.join()
            rospy.loginfo("Thread has finished")

        if error_occurred:
            error_occurred = False
            return

        self.pub_msg.trajectory = poses
        self.pub_msg.gripper_open = grippers
        #moving the arm away from the view
        #if self.whicharm == "right":
        #    gripper_position = self.planner.get_right_gripper_pose()
        #else:
        #    gripper_position = self.planner.get_left_gripper_pose()

        self.joint_mover.set_arm_state(pre_arm_pose, self.whicharm, wait=True)
        #turning the head towards the gripper final position
        #pos = (gripper_position.pose.position.x,
        #       gripper_position.pose.position.y,
        #       gripper_position.pose.position.z,
        #      )
        #self.joint_mover.point_head_to(pos, gripper_position.header.frame_id)

        res = self.search_object()
        if res is None:
            rospy.logerr("No object found!")
            return

        poses, dims, names = res
        self.pub_msg.post_movement_object_poses =[p for p in poses]
        self.pub_msg.post_movement_box_dims = [d for d in dims]
        self.pub_msg.post_object_names = names

        self.obj_changes_pub.publish(self.pub_msg)
        #rospy.loginfo("Message is: %s", self.pub_msg)
        rospy.loginfo("Number of elements in the trajectory: %d",
                len(self.pub_msg.trajectory))

