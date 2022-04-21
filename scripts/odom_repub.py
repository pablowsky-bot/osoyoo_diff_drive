#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from osoyoo_diff_drive.msg import lightOdom

odom_msg = Odometry()

odom_msg.header.frame_id = 'odom'

odom_msg.pose.pose.position.x = 0.0
odom_msg.pose.pose.position.y = 0.0
odom_msg.pose.pose.position.z = 0.0

odom_msg.pose.pose.orientation.x = 0.0
odom_msg.pose.pose.orientation.y = 0.0
odom_msg.pose.pose.orientation.z = 0.0
odom_msg.pose.pose.orientation.w = 1.0

odom_msg.twist.twist.linear.x = 0.0
odom_msg.twist.twist.linear.y = 0.0
odom_msg.twist.twist.linear.z = 0.0

odom_msg.twist.twist.angular.x = 0.0
odom_msg.twist.twist.angular.y = 0.0
odom_msg.twist.twist.angular.z = 0.0

# odom_msg.pose.covariance = []

odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
odom_twist_pub = rospy.Publisher('odom_twist', TwistStamped, queue_size=1)

def lightOdomCallback(msg):
    # fill robot sensed position (computed from encoder data)
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.pose.pose.position.x = msg.rx
    odom_msg.pose.pose.position.y = msg.ry
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, msg.r_theta)
    odom_msg.pose.pose.orientation.x = quaternion[0]
    odom_msg.pose.pose.orientation.y = quaternion[1]
    odom_msg.pose.pose.orientation.z = quaternion[2]
    odom_msg.pose.pose.orientation.w = quaternion[3]
    # fill sensed robot speed
    odom_msg.twist.twist.linear.x = msg.vx
    odom_msg.twist.twist.linear.y = msg.vy
    odom_msg.twist.twist.angular.z = msg.vtheta
    # publish
    odom_pub.publish(odom_msg)
    # broadcast tf from odom to base_footprint
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.rx, msg.ry, 0.0), quaternion,
                     rospy.Time.now(), 'base_footprint', 'odom')
    # publish odom twist for visualisation purposes
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = 'odom'
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.twist.linear.x = msg.vx
    twist_msg.twist.linear.y = msg.vy
    twist_msg.twist.linear.z = 0.0
    twist_msg.twist.angular.x = 0.0
    twist_msg.twist.angular.y = 0.0
    twist_msg.twist.angular.z = msg.vtheta
    odom_twist_pub.publish(twist_msg)

def odom_repub():
    rospy.init_node('odom_repub', anonymous=True)
    rospy.loginfo('pablowsky odom repub node started')
    rospy.Subscriber('lightweight_odom', lightOdom, lightOdomCallback)
    # prevent node from dying, allowing callback execution
    rospy.spin()

if __name__ == '__main__':
    try:
        odom_repub()
    except rospy.ROSInterruptException:
        pass
