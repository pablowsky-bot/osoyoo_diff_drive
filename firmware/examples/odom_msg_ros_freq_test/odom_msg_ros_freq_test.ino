/*
 * test if arduino is able to publish odom msg at 60 hz with rosserial
 */

#include <ros.h>
#include <geometry_msgs/Pose2D.h> // nav_msgs/Odometry is too big for Arduino dynamic memory

ros::NodeHandle nh;

geometry_msgs::Pose2D pose_msg;
ros::Publisher odom_pub("odom", &pose_msg);

// Interval is how long we wait
// add const if this should never change
const int interval=16666; // 60 hz aprox.

// Tracks the time since last event fired
unsigned long previousMicros = 0;

void setup()
{
    nh.initNode();
    nh.advertise(odom_pub);
}

void loop()
{
    // compute odometry at 60 hz
    unsigned long currentMicros = micros(); // Get snapshot of time
    
    // How much time has passed, accounting for rollover with subtraction
    if (currentMicros - previousMicros >= interval)
    {
        // This code is executed only when time interval has passed

        pose_msg.x = 0.0;
        pose_msg.y = 10.0;
        pose_msg.theta = 100.0;
        odom_pub.publish( &pose_msg );

        nh.spinOnce();

        // Use the snapshot to set track time until next event
        previousMicros = currentMicros;
    }
}
