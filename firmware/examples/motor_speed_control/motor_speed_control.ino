/*
 * A motor speed controller example
 */

/******** INCLUDE *********/

#include <SparkFun_TB6612.h> // TB6612FNG (motor driver / H-bridge chip) library to control the motors
#include <PinChangeInt.h>    // library to read encoders as fast as possible with Arduino
#include <PID_v1.h>          // control library to achieve motor speed as fast as possible

/******** DEFINE *********/

// motors

#define LEFT_MOTOR_DIR_0 7          // motor 1 H-bridge direction
#define LEFT_MOTOR_DIR_1 6          // motor 1 H-bridge direction

#define RIGHT_MOTOR_DIR_0 13        // motor 2 H-bridge direction
#define RIGHT_MOTOR_DIR_1 12        // motor 2 H-bridge direction

#define LEFT_MOTOR_PWM 9
#define RIGHT_MOTOR_PWM 10

#define STBY 8                      // Digital output signal to enable/disable motor driver

// encoders

#define left_motor_encoder 2
#define right_motor_encoder 4

#define pulses_per_revolution 1235  // 1235 or 1236 very good already, 1236 slightly overshoots

/******** GLOBALS *********/

// motors

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int motor_offset_A = 1;
const int motor_offset_B = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor left_motor = Motor(LEFT_MOTOR_DIR_0, LEFT_MOTOR_DIR_1, LEFT_MOTOR_PWM, motor_offset_A, STBY);
Motor right_motor = Motor(RIGHT_MOTOR_DIR_0, RIGHT_MOTOR_DIR_1, RIGHT_MOTOR_PWM, motor_offset_B, STBY);

// encoder tick counters
volatile unsigned int count_left = 0;
volatile unsigned int count_right = 0;

// measure elapsed time
unsigned long last_time;

// PID

double setpoint = 6.0;
double speed_sensor = 0.0;
double double_pid_output = 0.0;

// Specify the links and initial tuning parameters
double kp = 0.045;
double ki = 150.0;
double kd = 0.1;
int ctrl_delay = 49;

PID myPID(&speed_sensor, &double_pid_output, &setpoint, kp, ki, kd, DIRECT);

/******** SETUP *********/

void setup()
{
    // encoder pin configuration
    pinMode(left_motor_encoder, INPUT);
    pinMode(right_motor_encoder, INPUT);
    digitalWrite(left_motor_encoder, HIGH); //use the internal pullup resistor
    digitalWrite(right_motor_encoder, HIGH); //use the internal pullup resistor
    PCintPort::attachInterrupt(left_motor_encoder, interruptCountLeft, CHANGE);
    PCintPort::attachInterrupt(right_motor_encoder, interruptCountRight, CHANGE);

    // initialize serial port at 9600 baud rate
    Serial.begin(9600);

    // initialize variable to measure speed
    last_time = millis();

    // PID controller

    // limit the output of the PID controller between 0 and 255
    myPID.SetOutputLimits(0, 255);

    // turn the PID on
    myPID.SetMode(AUTOMATIC);

    // not sure if this dealy is really needed
    delay(300);
    Serial.println("setup complete");
}

void interruptCountLeft()
{
    count_left++;
}

void interruptCountRight()
{
    count_right++;
}

float measureSpeed()
{
    float delta_pulses_right = float(count_right);

    unsigned long current_time = millis();

    float delta_time = float(current_time - last_time);

    // get time for next cycle
    last_time = current_time;

    // reset count
    count_right = 0;

    /*  speed conversion from : [pulses] / [ms]    to    [rad] / [sec]
     *  
     *  delta_pulses_right [pulses]    1000 [ms]               2 Pi [rad]
     *  -------------------------- * ----------- * ---------------------------------
     *       delta_time [ms]           1 [sec]       pulses_per_revolution [pulses]
     */

    // return value is in : rad/sec
    return (delta_pulses_right * 1000.0 * 2 * 3.14159265359) / ( delta_time * pulses_per_revolution);
}

/******** LOOP *********/

/*
void loop()
{
    left_motor.drive(100);

    Serial.println(measureSpeed());
    delay(300);
}
*/

// PID
void loop()
{
    // update PID input
    speed_sensor = double(measureSpeed());

    // calculate required PWM to match the desired speed
    myPID.Compute();

    // cast PID double precision floating point output to int
    int int_pid_output = int(double_pid_output);

    // send PID output to motor
    right_motor.drive(int_pid_output);

    // debug info (remove)
    // Serial.println("setpoint :");
    // Serial.println(setpoint);
    // Serial.println("speed_sensor :");
    Serial.println(speed_sensor);
    // Serial.println("PID output :");
    // Serial.println(int_pid_output);

    delay(ctrl_delay);
}
