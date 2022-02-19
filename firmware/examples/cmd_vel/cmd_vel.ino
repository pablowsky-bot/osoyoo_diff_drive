/*
 * WIP: subscribe to cmd_vel topic and move the motors accordingly
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[16] = "Topic received!";

void messageCb( const std_msgs::Empty& toggle_msg)
{
  chatter.publish( &str_msg );
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  str_msg.data = hello;
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
