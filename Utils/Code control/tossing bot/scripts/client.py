import rospy
from open_manipulator_msgs.srv import *


def set_joint_position(th1, th2, th3, th4, time):
    service_name = '/goal_joint_space_path'
    rospy.wait_for_service(service_name)

    try:
        set_position = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        arg.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        arg.joint_position.position = [th1, th2, th3, th4]
        arg.path_time = time
        resp1 = set_position(arg)
        rospy.sleep(time+0.5)
        return resp1
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return "Failed"
    

def move_gripper(action):
    service_name = '/goal_tool_control'
    rospy.wait_for_service(service_name)
    
    try:
        gripper_position = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        arg.joint_position.joint_name = {"gripper"}
        arg.path_time = 1

        if action == "close":               
            arg.joint_position.position = {-0.01}
        elif "open":
            arg.joint_position.position = {0.01}
        
        resp1 = gripper_position(arg)
        rospy.sleep(1.5)
        return resp1
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return "Failed"




