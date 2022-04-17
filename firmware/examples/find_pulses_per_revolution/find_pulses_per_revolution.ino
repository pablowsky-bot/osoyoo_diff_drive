#include <SparkFun_TB6612.h> // TB6612FNG (motor driver / H-bridge chip) library to control the motors
#include <PinChangeInt.h>


#define LEFT_MOTOR_DIR_0 13         // motor 1 H-bridge direction
#define LEFT_MOTOR_DIR_1 12         // motor 1 H-bridge direction

#define RIGHT_MOTOR_DIR_0 7         // motor 2 H-bridge direction
#define RIGHT_MOTOR_DIR_1 6         // motor 2 H-bridge direction

#define LEFT_MOTOR_PWM 10
#define RIGHT_MOTOR_PWM 9

#define left_motor_encoder 4
#define right_motor_encoder 2

#define STBY 8   

const int motor_offset_A = -1;
const int motor_offset_B = -1;


volatile unsigned int count_left = 0;
volatile unsigned int count_right = 0;
// call the function more than once.
Motor left_motor = Motor(LEFT_MOTOR_DIR_0, LEFT_MOTOR_DIR_1, LEFT_MOTOR_PWM, motor_offset_A, STBY);
Motor right_motor = Motor(RIGHT_MOTOR_DIR_0, RIGHT_MOTOR_DIR_1, RIGHT_MOTOR_PWM, motor_offset_B, STBY);


void setup() {
  // put your setup code here, to run once:
  pinMode(left_motor_encoder, INPUT);
    pinMode(right_motor_encoder, INPUT);
    digitalWrite(left_motor_encoder, HIGH); //use the internal pullup resistor
    digitalWrite(right_motor_encoder, HIGH); //use the internal pullup resistor
  PCintPort::attachInterrupt(left_motor_encoder, interruptCountLeft, CHANGE);
   PCintPort::attachInterrupt(right_motor_encoder, interruptCountRight, CHANGE);  // problem!
  Serial.begin(9600);

  // allow some small time for setup function to stabilize
  delay(300);
}

void interruptCountLeft()
{
    count_left++;
}

void interruptCountRight()
{
    count_right++;
}

void loop() {
   if(count_right > 1183)
  {
    right_motor.brake();
    delay(1000);
    Serial.println(count_right); // 450!
    count_right = 0;
    delay(100);
  }
  else
  {
    right_motor.drive(80);
  }
}
