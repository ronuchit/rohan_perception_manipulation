import numpy as np

def transform_relative_pose_for_ik(manip, matrix4, ref_frame, targ_frame):
    robot = manip.GetRobot()

    if ref_frame == "world":
        worldFromRef = np.eye(4)
    else:
        ref = robot.GetLink(ref_frame)
        worldFromRef = ref.GetTransform()

    if targ_frame == "end_effector":
        targFromEE = np.eye(4)
    else:
        targ = robot.GetLink(targ_frame)
        worldFromTarg = targ.GetTransform()
        worldFromEE = manip.GetEndEffectorTransform()
        targFromEE = np.dot(np.linalg.inv(worldFromTarg), worldFromEE)

    refFromTarg_new = matrix4
    worldFromEE_new = np.dot(np.dot(worldFromRef, refFromTarg_new), targFromEE)

    return worldFromEE_new

def tf_for_link(T_w_link, manip, link_name):
    """
    Transforms an arbitrary link attached to the manipulator
    e.g. you might want ik for pr2 "r_gripper_tool_frame" instead of the openrave EE frame
    T_w_link: 4x4 matrix. "world frame from link frame"
    manip: openrave Manipulator
    link_name: (you know)
    filter_options: see openravepy.IkFilterOptions
    """
        
    robot = manip.GetRobot()

    link = robot.GetLink(link_name)

    if not robot.DoesAffect(manip.GetArmJoints()[-1], link.GetIndex()):
        raise Exception("link %s is not attached to end effector of manipulator %s"%(link_name, manip.GetName()))

    Tcur_w_link = link.GetTransform()
    Tcur_w_ee = manip.GetEndEffectorTransform()
    Tf_link_ee = np.linalg.solve(Tcur_w_link, Tcur_w_ee)
    
    T_w_ee = T_w_link.dot(Tf_link_ee)
    return T_w_ee