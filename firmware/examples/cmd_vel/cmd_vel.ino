/*
 * Subscribe to cmd_vel topic and move the motors accordingly
 */

/******** INCLUDE *********/

#include <SparkFun_TB6612.h> // TB6612FNG (motor driver / H-bridge chip) library to control the motors
#include <PinChangeInt.h>    // library to read encoders as fast as possible with Arduino

#include <diff_drive_kinematics.h> // inverse and forward kinematics library for differential drive robot

#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <math.h>
#include <PID_v1.h>          // control library to achieve motor speed as fast as possible

/******** DEFINE *********/

#define LEFT_MOTOR_DIR_0 13         // motor 1 H-bridge direction
#define LEFT_MOTOR_DIR_1 12         // motor 1 H-bridge direction

#define RIGHT_MOTOR_DIR_0 7         // motor 2 H-bridge direction
#define RIGHT_MOTOR_DIR_1 6         // motor 2 H-bridge direction

#define LEFT_MOTOR_PWM 10
#define RIGHT_MOTOR_PWM 9

#define FORWARD true
#define BACKWARD false


#define STBY 8               // Digital output signal to enable/disable motor driver

// encoders

#define left_motor_encoder 4
#define right_motor_encoder 2

#define wheel_radius 0.0325
#define distance_between_wheels  0.17
#define pulses_per_revolution 1235  // 1235 or 1236 very good already, 1236 slightly overshoots

/******** GLOBALS *********/

ros::NodeHandle nh;

// motors

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int motor_offset_A = -1;
const int motor_offset_B = -1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor left_motor = Motor(LEFT_MOTOR_DIR_0, LEFT_MOTOR_DIR_1, LEFT_MOTOR_PWM, motor_offset_A, STBY);
Motor right_motor = Motor(RIGHT_MOTOR_DIR_0, RIGHT_MOTOR_DIR_1, RIGHT_MOTOR_PWM, motor_offset_B, STBY);

//direction trigger
bool trigger;

// encoder tick counters
volatile unsigned int count_left = 0;
volatile unsigned int count_right = 0;

// measure elapsed time
unsigned long last_time;

// robot kinematics
double left_v = 0.0;        // PID input signal 1
double right_v = 0.0;       // PID input signal 2
float linear_v = 0.0;       // input robot linear speed in x direction
float angular_v = 0.0;      // input robot angular speed
float left_distance = 0.0;  // distance traveled by left wheel
float right_distance = 0.0; // distance traveled by right wheel
float tetha = 0.0;          // robot angular displacement
float rx = 0.0;             // robot linear displacement
float ry = 0.0;             // robot linear displacement
Robot pablowsky(pulses_per_revolution, wheel_radius, distance_between_wheels);

// PID

double speed_sensor_l = 0.0;
double speed_sensor_r = 0.0;

double double_pid_output_l = 0.0;
double double_pid_output_r = 0.0;

// Specify the links and initial tuning parameters
double kp = 0.045;
double ki = 150.0;
double kd = 0.1;
int ctrl_delay = 49;

PID leftPID(&speed_sensor_l, &double_pid_output_l, &left_v, kp, ki, kd, DIRECT);
PID rightPID(&speed_sensor_r, &double_pid_output_r, &right_v, kp, ki, kd, DIRECT);

/******** FUNCTIONS *********/

void cmdVelCb(const geometry_msgs::Twist& msg)
{
    linear_v = msg.linear.x;
    if (linear_v < 0)
    {
      linear_v = linear_v * -1;
      trigger = BACKWARD;
    }
    else
      trigger = FORWARD;
    angular_v = msg.angular.z;

}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCb );

/******** SETUP *********/

void setup()
{
    // register ros node
    nh.initNode();
    // subsribe to topic
    nh.subscribe(sub);

    // encoder pin configuration
    pinMode(left_motor_encoder, INPUT);
    pinMode(right_motor_encoder, INPUT);
    digitalWrite(left_motor_encoder, HIGH); //use the internal pullup resistor
    digitalWrite(right_motor_encoder, HIGH); //use the internal pullup resistor
    PCintPort::attachInterrupt(left_motor_encoder, interruptCountLeft, CHANGE);
    PCintPort::attachInterrupt(right_motor_encoder, interruptCountRight, CHANGE);  // problem!

    // initialize variable to measure speed
    last_time = millis();

    // PID controller

    // limit the output of the PID controller between 0 and 255
    leftPID.SetOutputLimits(0,255);
    rightPID.SetOutputLimits(0, 255);

    // turn the PID on
    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);

    // sets the period, in Milliseconds, at which the calculation is performed
    leftPID.SetSampleTime(ctrl_delay);
    rightPID.SetSampleTime(ctrl_delay);

    delay(300);
    nh.loginfo("setup complete");
}

void interruptCountLeft()
{
    count_left++;
}

void interruptCountRight()
{
    count_right++;
}

void measureSpeed(double *left_speed, double *right_speed)
{
    float delta_pulses_right = float(count_right);
    float delta_pulses_left = float(count_left);

    unsigned long current_time = millis();

    float delta_time = float(current_time - last_time);

    // get time for next cycle
    last_time = current_time;

    // reset count
    count_right = 0;
    count_left = 0;

    /*  speed conversion from : [pulses] / [ms]    to    [rad] / [sec]
     *
     *  delta_pulses_right [pulses]    1000 [ms]               2 Pi [rad]
     *  -------------------------- * ----------- * ---------------------------------
     *       delta_time [ms]           1 [sec]       pulses_per_revolution [pulses]
     */
    // return value is in : rad/sec
    *right_speed = (delta_pulses_right * 1000 * 2 * M_PI) / (delta_time * pulses_per_revolution);
    *left_speed = (delta_pulses_left * 1000 * 2 * M_PI) / (delta_time * pulses_per_revolution);
    if (!trigger)
    {
        //*right_speed = -*right_speed;
        //*left_speed = -*left_speed;
    }
}

/******** LOOP *********/

void loop()
{
    // map robot linear and angular velocity to motor speed
    pablowsky.InverseK(angular_v, linear_v, left_v, right_v);
    // update PID input
    measureSpeed(&speed_sensor_l, &speed_sensor_r);
    // calculate required PWM to match the desired speed
    leftPID.Compute();
    rightPID.Compute();
    // cast PID double precision floating point output to int
    // send PID output to motor
    if (!trigger)
    {
      double_pid_output_r = -(int)double_pid_output_r;
      double_pid_output_l = -(int)double_pid_output_l;
    }
    right_motor.drive((int)double_pid_output_r);
    left_motor.drive((int)double_pid_output_l);

    delay(ctrl_delay);

    // listen to callbacks from ROS
    nh.spinOnce();
}
