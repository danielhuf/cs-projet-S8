#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import argparse

def publisher(publisher_name, topic_name):
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    rospy.init_node(publisher_name, anonymous=True)
    return pub

class PublisherManager:

    def __init__(self, publisher_name, topic_name):
        self.publisher = publisher(publisher_name, topic_name)
        #self.publisher_angle = publisher("cup_angle_publisher", "cup_angle")

    def publish_msg(self, msg):
        self.publisher.publish(msg)
        #self.publisher_angle.publish(cup_angle)



if __name__ == "__main__":
    print("[INFO] constructing argument parser...")
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--topic", type=str, default="/cup_distance", help="publisher topic name")
    ap.add_argument("-n", "--name", type=str, default="cup_distance_publisher", help="publisher name")
    args = vars(ap.parse_args())

    pub_manager = PublisherManager(args["name"], args["topic"])    
    cup_distance = 10
    #cup_angle = 0.1
    rate = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            rospy.loginfo(cup_distance)
            #rospy.loginfo(cup_angle)
            pub_manager.publish_msg(cup_distance)
            cup_distance+=2
            rate.sleep()
    except rospy.ROSInterruptException:
        print("[INFO] Communication Interrupted")
