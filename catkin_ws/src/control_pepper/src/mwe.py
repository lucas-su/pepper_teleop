#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import geometry_msgs
import naoqi_bridge_msgs.msg as naoqi
import time

def pub_joint():
    pub = rospy.Publisher('/joint_angles', naoqi.JointAnglesWithSpeed, queue_size=10)
    rospy.init_node('cmd_joint', anonymous=True)
    msg = naoqi.JointAnglesWithSpeed()
    msg.header.stamp = rospy.Time.now() 
    msg.joint_names = ["HeadPitch"]
    msg.joint_angles = [0]
    # msg.joint_names = ["LShoulderPitch","LShoulderRoll","LElbowRoll","LElbowYaw","RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw"]
    # msg.joint_angles = [-10, 5, -20, 10, -10, 10, -20, 10]
    # print('publishing msg 1')
    # pub.publish(msg)
    time.sleep(1)
    msg.speed = 0.5
    msg.relative = 0
    print('publishing msg')
    pub.publish(msg)
    rospy.spin

if __name__ == '__main__':
    try:
        pub_joint()
    except rospy.ROSInterruptException:
        pass