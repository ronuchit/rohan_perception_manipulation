#! /usr/bin/python
import roslib
roslib.load_manifest("workspace")

import openravepy
import numpy as np
import rospy
         

if __name__ == "__main__":  
    rospy.init_node("workspace", anonymous=True)
    
    env = openravepy.Environment() # load new environment
    env.Load('rave_world.env.xml')

    robot = env.GetRobots()[0]
    robot.SetActiveManipulator('rightarm')
    target = env.GetKinBody('obj1')

    env.SetViewer('qtcoin')

    gmodel = openravepy.databases.grasping.GraspingModel(robot, target)
    if not gmodel.load():
         openravepy.raveLogInfo("Generating grasping model")
         gmodel.autogenerate()
    
    openravepy.raveLogInfo("Computing valid grasps")
    validgrasps, _ = gmodel.computeValidGrasps(checkcollision=False, checkik=True, checkgrasper = False)
    if len(validgrasps) < 4:
         openravepy.raveLogInfo("Valid grasps: " + str(validgrasps) + ", Num: " + str(len(validgrasps)))
    else:
         openravepy.raveLogInfo("Num valid grasps: " + str(len(validgrasps)))

    validgrasp = validgrasps[0] # choose first grasp
    #gmodel.showgrasp(validgrasp) # show the grasp
    gmodel.moveToPreshape(validgrasp) # move to the preshape of the first grasp
    Tgrasp = gmodel.getGlobalGraspTransform(validgrasp, collisionfree=True) # get the grasp transform
    basemanip = openravepy.interfaces.BaseManipulation(robot)
    basemanip.MoveToHandPosition(matrices=[Tgrasp]) # move the robot to the grasp    
    robot.WaitForController(0)
    taskmanip = openravepy.interfaces.TaskManipulation(robot)
    taskmanip.CloseFingers()
    robot.WaitForController(0)

    openravepy.raveLogInfo("EXECUTION COMPLETE")
