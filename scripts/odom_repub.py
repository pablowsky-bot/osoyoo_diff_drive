#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

odom_msg = Odometry()

odom_msg.header.frame_id = 'odom'

odom_msg.pose.pose.position.x = 0.0
odom_msg.pose.pose.position.y = 0.0
odom_msg.pose.pose.position.z = 0.0

odom_msg.pose.pose.orientation.x = 0.0
odom_msg.pose.pose.orientation.y = 0.0
odom_msg.pose.pose.orientation.z = 0.0
odom_msg.pose.pose.orientation.w = 0.0

# odom_msg.pose.covariance = []

odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

def callback(msg):
    odom_msg.pose.pose.position.x = msg.x
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, msg.theta)
    odom_msg.pose.pose.orientation.x = quaternion[0]
    odom_msg.pose.pose.orientation.y = quaternion[1]
    odom_msg.pose.pose.orientation.z = quaternion[2]
    odom_msg.pose.pose.orientation.w = quaternion[3]
    odom_pub.publish(odom_msg)

def odom_repub():
    rospy.init_node('odom_repub', anonymous=True)
    rospy.loginfo('pablowsky odom repub node started')
    rospy.Subscriber('lightweight_odom', Pose2D, callback)
    # prevent node from dying, but at the same time listens to callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        odom_repub()
    except rospy.ROSInterruptException:
        pass
