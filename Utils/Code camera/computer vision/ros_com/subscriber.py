#!/usr/bin/env python
import rospy
from std_msgs.msg import Image


def image_callback(msg):
    # Process the received image
    # Access image data using 'msg' object
    # Example: print image width and height
    print("Image width:", msg.width)
    print("Image height:", msg.height)
    print("Image height:", msg.height)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/img", Image, image_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()