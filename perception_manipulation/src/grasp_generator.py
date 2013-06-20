import rospy
import math
import numpy as np

from object_manipulation_msgs.msg import Grasp
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from tf import transformations


def generate_grasps(box,
                    num_grasps=4,
                    desired_approach_distance = 0.10,
                    min_approach_distance = 0.05,
                    effort = 50):
    """
    Generates grasps 

    Parameters:
    box: bounding box to genereate grasps for
    num_grasps: number of grasps to generate, spaced equally around the box
    desired_approach_distance: how far the pre-grasp should ideally be away from the grasp
    min_approach_distance: how much distance between pre-grasp and grasp must actually be feasible for the grasp not to be rejected

    Return: a list of grasps
    """

    rospy.loginfo("Generating grasps")
    grasps = []
    for i in range(num_grasps):
        g = Grasp()

        # align z-axis rotation to box
        euler = transformations.euler_from_quaternion([
            box.pose.pose.orientation.x,
            box.pose.pose.orientation.y,
            box.pose.pose.orientation.z,
            box.pose.pose.orientation.w])
        rot_ang = euler[2]

        # apply transformations based on rotations
        rot_ang += i * (2 * math.pi) / num_grasps
        t1 = transformations.translation_matrix([
            box.pose.pose.position.x,
            box.pose.pose.position.y,
            box.pose.pose.position.z + \
                box.box_dims.z * 0.05])
        r1 = transformations.rotation_matrix(rot_ang, [0, 0, 1])
        box_width = max(box.box_dims.x, box.box_dims.y)
        t2 = transformations.translation_matrix([-(box_width/2 + 0.12), 0, 0])
        mat = transformations.concatenate_matrices(t1, r1, t2)

        translation = transformations.translation_from_matrix(mat)
        g.grasp_pose.position.x = translation[0]
        g.grasp_pose.position.y = translation[1]
        g.grasp_pose.position.z = translation[2]

        q = transformations.quaternion_from_matrix(mat)
        g.grasp_pose.orientation.x = q[0]
        g.grasp_pose.orientation.y = q[1]
        g.grasp_pose.orientation.z = q[2]
        g.grasp_pose.orientation.w = q[3]

        g.desired_approach_distance = desired_approach_distance
        g.min_approach_distance = min_approach_distance

        pre_grasp_posture = JointState()
        pre_grasp_posture.header.stamp = rospy.Time.now()
        pre_grasp_posture.header.frame_id = 'base_link'
        pre_grasp_posture.name = ['r_wrist_flex_link']
        pre_grasp_posture.position = [0.8]
        pre_grasp_posture.effort = [effort]
        g.pre_grasp_posture = pre_grasp_posture

        grasp_posture = JointState()
        grasp_posture.header.stamp = rospy.Time.now()
        grasp_posture.header.frame_id = 'base_link'
        grasp_posture.name = ['r_wrist_flex_link']
        grasp_posture.position = [0.45]
        grasp_posture.effort = [effort]
        g.grasp_posture = grasp_posture

        grasps.append(g)
    np.random.shuffle(grasps)
    return grasps


def draw_grasps(grasps):
    """
    Draws grasps in RVIZ

    Parameters:
    grasps: a list of grasps
    """
    # TODO: add fingers to gripper
    publisher = rospy.Publisher("visualization_marker", Marker)
    for i, g in enumerate(grasps):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "grasps"
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae"
        marker.action = marker.ADD
        marker.id = i
        marker.pose.position = g.grasp_pose.position
        marker.pose.orientation = g.grasp_pose.orientation
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.mesh_use_embedded_materials = True
        publisher.publish(marker)
