#!/usr/bin/env python3
import rospy
import visualization_msgs.msg as visMsg
import naoqi_bridge_msgs.msg as naoqi
import math

def pub_joint(names, thetas):
    msg = naoqi.JointAnglesWithSpeed()
    msg.header.stamp = rospy.Time.now() 
    msg.joint_names = names
    msg.joint_angles = thetas
    msg.speed = speed
    msg.relative = 0
    pub.publish(msg)
    
def det_angle(a, b, c):
    """
    can accept any lenth vectors but should here be used in R2 to get angles for singular joints
    """
    ab = [x[0]-x[1] for x in zip(b,a)]
    bc = [x[0]-x[1] for x in zip(c,b)]
    theta = math.acos(sum(x*y for x, y in zip(ab, bc))/(math.sqrt(sum([x**2 for x in ab]))*math.sqrt(sum([x**2 for x in bc]))))
    return theta

def process_angles(data):

    # shoulder pitch R 
    theta_RShoulderPitch = det_angle((data.markers[11].pose.position.y, data.markers[11].pose.position.x), 
            (data.markers[12].pose.position.y,data.markers[12].pose.position.x),
            (data.markers[13].pose.position.y,data.markers[13].pose.position.x))

    # invert sign for right side
    # 1.56 (max angle) minus theta inverts angle, as theta=0 is obtained with the shoulder joint 
    # perpendicular to the body but results in the robot arm parallel to the body
    theta_RShoulderPitch = (1.56 - theta_RShoulderPitch)*-1

    # shoulder pitch L
    # 1.56 (max angle) minus theta inverts angle, as theta=0 is obtained with the shoulder joint 
    # perpendicular to the body but results in the robot arm parallel to the body
    theta_LShoulderPitch = det_angle((data.markers[4].pose.position.y, data.markers[4].pose.position.x), 
            (data.markers[5].pose.position.y,data.markers[5].pose.position.x),
            (data.markers[6].pose.position.y,data.markers[6].pose.position.x))
    theta_LShoulderPitch = (1.56 - theta_LShoulderPitch)

    names = ['RShoulderRoll', 'LShoulderRoll']
    thetas = [theta_RShoulderPitch,theta_LShoulderPitch]
    return names, thetas

def frame_callback(data):
    if data.markers.__len__()>0:
        names, thetas = process_angles(data)
        pub_joint(names, thetas)
    
def sub_frame():
    rospy.Subscriber("/body_tracking_data", visMsg.MarkerArray , frame_callback)

if __name__ == '__main__':
    #globals
    speed = 0.5 # between 0 and 1
    rospy.init_node('cmd_joint')
    pub = rospy.Publisher('/joint_angles', naoqi.JointAnglesWithSpeed, queue_size=10)
    sub_frame()
    rospy.spin()
