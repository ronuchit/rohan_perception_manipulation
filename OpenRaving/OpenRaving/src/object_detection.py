import rospy
import tabletop_actions.object_detector as object_detector
import tf 
from tf import transformations
import numpy as np
import openravepy

import sys

def create_openrave_body(env, name=None):
	""" Detects an object and creates corresponding OpenRave body in environment env with specified name.
			Returns the created OpenRave body. """
	
	detector = object_detector.GenericDetector()
	
	rospy.loginfo("Attempting to detect an object.")
	res = detector.detect()
	
	if res is None:
		rospy.logerr("No object detected.")
		return None
		
	rospy.loginfo("Object detected.")
	cluster = detector.find_biggest_cluster(res.detection.clusters)
	box_msg = detector.detect_bounding_box(cluster) # a FindClusterBoundingBoxResponse message
	
	if box_msg.error_code != box_msg.SUCCESS:
		rospy.logerr("No bounding box detected.")
		return None
	
	rospy.loginfo("Bounding box detected.")	
	values = [0,0,0, box_msg.box_dims.x, box_msg.box_dims.y, box_msg.box_dims.z]
	body = openravepy.RaveCreateKinBody(env,'')
	if name is not None:
		body.SetName(name)    
	else:
		body.SetName("random_object")
			
	body.InitFromBoxes(np.array([values]), True)
	T = body.GetTransform()
	T[:,-1] = [box_msg.pose.pose.position.x, box_msg.pose.pose.position.y, box_msg.pose.pose.position.z, 1]
	body.SetTransform(T)
	
	if name is not None:
			env.Add(body, False)
	else:
			env.Add(body, True)
	return body
	
if __name__ == "__main__":        
	env = openravepy.Environment() # load new environment
	#env.Load("bare_bone.dae")
	
	create_openrave_body(env, "detected_object")
	
	env.Save("created_info.dae")
