/*
 * WIP: subscribe to cmd_vel topic and move the motors accordingly
 */

/******** INCLUDE *********/

#include <SparkFun_TB6612.h> // TB6612FNG (motor driver / H-bridge chip) library to control the motors
#include <ros.h>
#include <geometry_msgs/Twist.h>

/******** DEFINE *********/

#define LEFT_MOTOR_DIR_0 7   // motor 1 H-bridge direction
#define LEFT_MOTOR_DIR_1 6   // motor 1 H-bridge direction

#define RIGHT_MOTOR_DIR_0 13 // motor 2 H-bridge direction
#define RIGHT_MOTOR_DIR_1 12 // motor 2 H-bridge direction

#define LEFT_MOTOR_PWM 9
#define RIGHT_MOTOR_PWM 10

#define STBY 8               // Digital output signal to enable/disable motor driver

/******** GLOBALS *********/

ros::NodeHandle nh;

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int motor_offset_A = 1;
const int motor_offset_B = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(LEFT_MOTOR_DIR_0, LEFT_MOTOR_DIR_1, LEFT_MOTOR_PWM, motor_offset_A, STBY);
Motor motor2 = Motor(RIGHT_MOTOR_DIR_0, RIGHT_MOTOR_DIR_1, RIGHT_MOTOR_PWM, motor_offset_B, STBY);

/******** FUNCTIONS *********/

void cmdVelCb(const geometry_msgs::Twist& msg)
{
    nh.loginfo("received twist msg");
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCb );

/******** SETUP *********/

void setup()
{
    nh.initNode();
    nh.subscribe(sub);
}

/******** LOOP *********/

void loop()
{
    // drive motor1 forward half speed for 1 second
    motor1.drive(128,1000);
    motor1.drive(-128,1000);
    motor1.brake();
    delay(1000);

    // drive motor1 forward half speed for 1 second
    // motor2.drive(128,1000);
    motor2.drive(-128,1000);
    motor2.brake();
    delay(1000);

    // listen to callbacks from ROS
    nh.spinOnce();
}
