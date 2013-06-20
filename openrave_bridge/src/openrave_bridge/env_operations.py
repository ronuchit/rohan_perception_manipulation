import roslib
roslib.load_manifest("openrave_bridge")
from tabletop_object_detector.srv import TabletopDetectionResponse
import openravepy
import numpy as np

def add_table(table_msg, env):
    assert isinstance(table_msg, TabletopDetectionResponse)
    body = openravepy.RaveCreateKinBody(env,'')
    body.SetName('table')
    z = table_msg.detection.table.pose.pose.position.z
    x_min = table_msg.detection.table.x_min
    x_max = table_msg.detection.table.x_max
    y_min = table_msg.detection.table.y_min
    y_max = table_msg.detection.table.y_max
    
    x = (x_max-x_min)/2 + x_min
    y = (y_max-y_min)/2 + y_min
    dim_x = (x_max - x_min)/2 + 0.1
    dim_y = (y_max - y_min)/2 + 0.1
    body.InitFromBoxes(np.array([[x, y, z, dim_x, dim_y, 0.06]]), True)
    env.Add(body, True)    
    
    return body