#! /usr/bin/python
import roslib
roslib.load_manifest("perception_manipulation")

import rospy
import object_detector
import pr2_control_utilities
import tf 
from tf import transformations
import numpy as np
import openravepy
import pdb
from decimal import Decimal
import sys
import pr2model
import random
from sensor_msgs.msg import *
from visualization_msgs.msg import Marker
from ar_pose.msg import *

TABLE_TOP_PADDING = 0.06

def add_openrave_bodies():
  fname1 = "env_temp1.dae"
  fname2 = "env_temp2.xml"

  robot_state = pr2_control_utilities.RobotState()
  joint_mover = pr2_control_utilities.PR2JointMover(robot_state)
  detector = object_detector.GenericDetector()
  
  rospy.loginfo("Attempting to detect an object.")

  joint_mover.point_head_to((0.7, -0.15, 0), '/base_link')
  clusters, table = detector.find_all_clusters(trials=1, draw_markers=False)

  env1 = openravepy.Environment()

  add_pr2(env1)
  rospy.loginfo("PR2 added to env.")

  add_detected_table(table, env1)
  rospy.loginfo("Table added to env.")

  add_block(env1, 'table6', 0, -0.8, 0.5, 0.5, 0.25, 0.005)
  rospy.loginfo("Drop table added to env.")

  env1.Save(fname1)

  cluster_map = {}
  env_xml = '<Environment>'
  for i, cluster in enumerate(clusters):
      box_msg = detector.detect_bounding_box(cluster) 
      # a FindClusterBoundingBoxResponse message
      
      if box_msg.error_code != box_msg.SUCCESS:
        rospy.logerr("No bounding box detected for object " \
                 + repr(i))
        return None
      
      rospy.loginfo("Bounding box detected for object " + repr(i))

      body_name = "object" + repr(i)
      cylinder_xml = generate_cylinder_xml(body_name, box_msg, padding=0.005)
      env_xml += cylinder_xml

      rospy.loginfo("Body " + repr(i) + " added to env.")
      cluster_map[body_name] = i
  env_xml += '</Environment>'

  with open(fname2, 'w') as env_file:
    env_file.write(env_xml)

  env = openravepy.Environment()
  env.Load(fname1)
  env.Load(fname2)

  return env, (detector, cluster_map), clusters
"""

def create_openrave_bodies(env, viewer=True):
  ""
  Detects an object and creates corresponding OpenRave body
  in environment env with specified name.

  Mutates the provided environment input
  Returns a tuple of (detector, cluster_map), where cluster_map is a dict of
    key: OpenRave object name
    value: cluster index
  ""

  add_pr2(env)

  #ORPR2 = pr2model.PR2Robot(env)
  #ORPR2.tuck_arms()
  if viewer:
    env.SetViewer('qtcoin')
        
  robot_state = pr2_control_utilities.RobotState()
  joint_mover = pr2_control_utilities.PR2JointMover(robot_state)
  detector = object_detector.GenericDetector()
  
  rospy.loginfo("Attempting to detect an object.")
  
  #detector.search_for_object(joint_mover, trials=15,
  # cluster_choser=detector.find_biggest_cluster, max_pan=0.5,
  # min_pan=-0.5, max_tilt = 1.1, min_tilt = 0.6)
        
  # res = detector.last_detection_msg
  # for i in range(0, 10):
  #   rospy.loginfo("Detection trial #" + str(i))
  #   res = detector.detect()
  #   if res is not None:
  #     break;
    

  # if res is None:
  #   rospy.logerr("No object detected.")
  #   return None
  
  # rospy.loginfo("Object detected.")
  # cluster = detector.find_biggest_cluster(res.detection.clusters)
  joint_mover.point_head_to((0.9, -0.15, 0), '/base_link')
  clusters, table = detector.find_all_clusters(trials=1, draw_markers=False)

  cluster_map = {}
  for i, cluster in enumerate(clusters): # each cluster is an array of PointCloud msgs
    box_msg = detector.detect_bounding_box(cluster)
    # a FindClusterBoundingBoxResponse message
    
    if box_msg.error_code != box_msg.SUCCESS:
      rospy.logerr("No bounding box detected for object " \
               + repr(i))
      return None
    
    rospy.loginfo("Bounding box detected for object " + repr(i))

    body_name = "object" + repr(i)
    add_body(env, body_name, box_msg, 1.05, 0.01)
    rospy.loginfo("Body " + repr(i) + " added to env.")
    cluster_map[body_name] = i
    
    
  add_detected_table(table, env)
  rospy.loginfo("Table added to env.")
  # add_block(env,"x",  1,1,1, 0.3,0.01,0.01)
  # add_block(env,"z",  1,1,1, 0.01,0.01,0.3)
  return detector, cluster_map, clusters
"""

def generate_cylinder_xml(body_name, box_msg, scale=1.0, padding=0.00):
    x = box_msg.pose.pose.position.x
    y = box_msg.pose.pose.position.y
    z = box_msg.pose.pose.position.z + TABLE_TOP_PADDING
    radius = max(box_msg.box_dims.x/2, box_msg.box_dims.y/2) * scale + padding
    height = box_msg.box_dims.z# * scale + padding
    xml_str = """
      <KinBody name="%s">
        <Translation> %f %f %f </Translation>
        <RotationAxis>1 0 0 90</RotationAxis>        
        <Body type="dynamic">
          <Geom type="cylinder">
            <radius> %f </radius>
            <height> %f </height>
          </Geom>
        </Body>
      </KinBody>
    """%(body_name, x, y, z, radius, height)
      
    return xml_str

def add_body(env, body_name, box_msg, scale=1.0, padding=0):
    ###@$!@#$!@#!!! openrave takes half extents
    values = [0,0,0,\
              box_msg.box_dims.x*scale/2 + padding,\
              box_msg.box_dims.y*scale/2 + padding,\
              box_msg.box_dims.z*scale/2 + padding]
    # values = [box_msg.pose.pose.position.x, \
    #     box_msg.pose.pose.position.y,\
    #     box_msg.pose.pose.position.z,\
    #     box_msg.box_dims.x, \
    #     box_msg.box_dims.y,\
    #       box_msg.box_dims.z]

    print values
    print "box_msg: "+ repr(box_msg)

    body = openravepy.RaveCreateKinBody(env,'')
    body.SetName(body_name)
        
    body.InitFromBoxes(np.array([values]), True)
    T = body.GetTransform()
    # T[:,-1] = [box_msg.pose.pose.position.x, \
    #          box_msg.pose.pose.position.y, \
    #          box_msg.pose.pose.position.z, 1]
    obj_position = [box_msg.pose.pose.position.x,\
        box_msg.pose.pose.position.y,\
        box_msg.pose.pose.position.z + \
        (box_msg.box_dims.z*scale/2 + padding - box_msg.box_dims.z/2)]

    ###@$!@#$!@#!!! Openrave expresses quaternions as wxyz!!!
    obj_rot =  [box_msg.pose.pose.orientation.w,\
            box_msg.pose.pose.orientation.x,\
            box_msg.pose.pose.orientation.y,\
            box_msg.pose.pose.orientation.z]

    poseForOpenrave = []
    poseForOpenrave[0:4] = obj_rot
    poseForOpenrave[4:6] = obj_position
    # T = openravepy.matrixFromQuat(obj_rot)
    # body.SetTransform(T)
    T = openravepy.matrixFromPose(poseForOpenrave)
    body.SetTransform(T)
           
    env.Add(body, True)

def add_block(env, name, x, y, z, dimx, dimy, dimz):
    values = [x,y,z,dimx,dimy,dimz]
    body = openravepy.RaveCreateKinBody(env, '')
    body.SetName(name)
    body.InitFromBoxes(np.array([values]), True)
    env.Add(body, True)
    return body

def add_pr2(env):
     robot = env.ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml")
     return env.Add(robot)


def add_detected_table(table, env):
    half_thickness = 0.15

    z = table.pose.pose.position.z + TABLE_TOP_PADDING - half_thickness
    x_min = table.x_min
    x_max = table.x_max
    y_min = table.y_min
    y_max = table.y_max
    
    x = (x_max-x_min)/2 + x_min
    y = (y_max-y_min)/2 + y_min
    ##OpenRave takes half-extents
    dim_x = (x_max - x_min)/2
    dim_x += 0.05
    dim_y = (y_max - y_min)/2
    dim_y += 0.05
    body = add_block(env, 'table', x, y, z, dim_x, dim_y, half_thickness)

    return body


def convert_point_cloud_to_point_cloud2(pc):
  """
  Adopted from corresponding C++ function in sensor_msgs/point_cloud_conversion.h
  Converts PointCloud message pc to PointCloud2 message. Returns PointCloud2 message
  without modifying pc.
  """
  pc2 = PointCloud2()
  
  pc2.header = pc.header
  pc2.width = len(pc.points)
  pc2.height = 1
  pc2.fields = [PointField() for i in range(len(pc.channels) + 3)]

  pc2.fields[0].name = "x"
  pc2.fields[1].name = "y"
  pc2.fields[2].name = "z"

  offset = 0
  for i in range(len(pc2.fields)):
    pc2.fields[i].offset = offset;
    pc2.fields[i].datatype = PointField.FLOAT32
    pc2.fields[i].count = 1
    offset += 4

  pc2.point_step = offset
  pc2.row_step = pc2.point_step * pc2.width

  for i in range(len(pc.channels)):
    pc2.fields[3 + i].name = pc.channels[i].name

  pc2.is_bigendian = False
  pc2.is_dense = False

  pc2.data = [0 for i in range(len(pc.points) * pc2.point_step)]
  for i in range(len(pc.points)):
    pc2.data[i * pc2.point_step + pc2.fields[0].offset] = pc.points[i].x
    pc2.data[i * pc2.point_step + pc2.fields[1].offset] = pc.points[i].y
    pc2.data[i * pc2.point_step + pc2.fields[2].offset] = pc.points[i].z
    
    for j in range(len(pc.channels)):
      if len(pc.channels[j].values) == len(pc.points):
        pc2.data[i * pc2.point_step + pc2.fields[3 + j].offset] = pc.channels[j].values[i]

  return pc2


def set_and_draw_markers(clusters):
  all_ar_markers = [] # i-th element is a list of AR markers corresponding to cluster i in clusters
  all_vis_markers = [] # i-th element is an rviz visualization marker corresponding to cluster i in clusters
  
  # set ros publishers and subscribers
  point_publisher = rospy.Publisher('points', PointCloud2)
  rviz_marker_drawer = rospy.Publisher('visualization_marker', Marker)
  rospy.Subscriber('ar_pose_markers', ARMarkers, _ar_callback)
  rospy.Subscriber('visualization_marker', Marker, _vis_callback, rviz_marker_drawer)

  for point_cloud in clusters:
    ar_done = False
    vis_done = False
    point_cloud2 = convert_point_cloud_to_point_cloud2(point_cloud)
    point_publisher.publish(point_cloud2)
    while not (ar_done and vis_done):
      #rospy.loginfo("Waiting for ar_kinect publishes...")
      continue

  return all_ar_markers, all_vis_markers


def _ar_callback(ar_markers):
  rospy.loginfo("In AR callback")
  global all_ar_markers, ar_done
  all_ar_markers += ar_markers
  ar_done = True

def _vis_callback(vis_marker, rviz_marker_drawer):
  rospy.loginfo("In vis callback")
  global all_vis_markers, vis_done
  all_vis_markers += vis_marker  
  rviz_marker_drawer.publish(vis_marker)
  vis_done = True


def add_openrave_bodies_and_ar_markers(viewer=True):
  """
  Top-level function. Detects objects and creates corresponding OpenRave
  bodies in an environment. Returns environment and corresponding
  AR Markers, and draws rviz visualization markers.

  env: the created environment
  cluster_map: dict of
    key: OpenRave object name
    value: cluster index
  clusters: array of point cloud for each detected object
  all_ar_markers: i-th element is a list of AR markers corresponding to cluster i in clusters
  all_vis_markers: i-th element is an rviz visualization marker corresponding to cluster i in clusters
  """

  env, (detector, cluster_map), clusters = add_openrave_bodies()
  if viewer:
    env.SetViewer('qtcoin')

  if clusters == None or len(clusters) == 0:
    rospy.logwarn("No clusters detected.")
    all_ar_markers, all_vis_markers = None, None
  else:
    all_ar_markers, all_vis_markers = set_and_draw_markers(clusters)
  rospy.loginfo("******AR Markers******\n" + repr(all_ar_markers))

  return env, detector, cluster_map, clusters, all_ar_markers, all_vis_markers

if __name__ == "__main__":
    rospy.init_node("workspace", anonymous=True)

    #env = openravepy.Environment() # load new environment
    #env.Load("bare_bone.dae")

    #create_openrave_bodies(env, viewer=False)
    env = add_openrave_bodies_and_ar_markers(viewer=False)[0]

    # add custom table
    #add_block(env, 'Table6', 0, -1.5, 0.5, 0.5, 0.25, 0.005)

    env.Save("created_info.dae")
 
