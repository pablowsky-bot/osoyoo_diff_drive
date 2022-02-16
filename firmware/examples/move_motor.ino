// The I2Cdev, MPU6050 class libraries need to be installed in the Arduino class library folder in advance
// #include "I2Cdev.h"

#define IN1M 7 // motor1
#define IN2M 6 // motor 1

#define IN3M 13 // motor 2
#define IN4M 12 // motor 2

#define PWMA 9 // motor 1
#define PWMB 10 // motor 2

#define STBY 8

// Encoder count signal
#define PinA_left 2  // interrupt 0
#define PinA_right 4 // interrupt 1

// the setup function runs once when you press reset or power the board
void setup() {
    // TB6612FNG driver module control signal initialization
    pinMode(IN1M, OUTPUT); // Control the direction of motor 1, 01 is forward rotation, 10 is reverse rotation
    pinMode(IN2M, OUTPUT);
    pinMode(IN3M, OUTPUT); // Control the direction of motor 2, 01 is forward rotation, 10 is reverse rotation
    pinMode(IN4M, OUTPUT);

    pinMode(PWMA, OUTPUT); // Left motor PWM
    pinMode(PWMB, OUTPUT); // Right motor PWM

    pinMode(STBY, OUTPUT); // enable TB6612FNG
    // Initialize the motor driver module
    digitalWrite(IN1M, 0);
    digitalWrite(IN2M, 1);
    digitalWrite(IN3M, 1);
    digitalWrite(IN4M, 0);
    digitalWrite(STBY, 1);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    // Speed ​​dial input
    pinMode(PinA_left, INPUT);  
    pinMode(PinA_right, INPUT);
    // Join the I2C bus
    // Wire.begin(); // Join the I2C bus sequence
    // Serial.begin(9600); // Open the serial port and set the baud rate to 115200
    // delay(1500);
    // mpu.initialize(); // Initialize MPU6050
    // delay(2);
    // balancecar.pwm1 = 0;
    // balancecar.pwm2 = 0;
    /// 5ms timer interrupt setting, use timer2. Note: using timer2 will affect the PWM output of pin3 pin11,
    /// Because PWM uses a timer to control the duty cycle, when using a timer, pay attention to check the pin port of the corresponding timer.
    // MsTimer2::set(5, inter);
    // MsTimer2::start();
    
    delay(300); // 300 ms delay
}

void moveMotor(bool motor, bool direction, int pwm) // set pwm between 0 - 255
{
    if motor
    {
        // left motor
        if direction
        {
            // CW
            digitalWrite(IN2M, 0);
            digitalWrite(IN1M, 1);
            analogWrite(PWMA, pwm);
        }
        else
        {
            // CCW
            digitalWrite(IN2M, 1);
            digitalWrite(IN1M, 0);
            analogWrite(PWMA, pwm);
        }
    }
    else
    {
        // right motor
        if direction
        {
            // CW
            digitalWrite(IN4M, 1);
            digitalWrite(IN3M, 0);
            analogWrite(PWMB, pwm);
        }
        else
        {
            // CCW
            digitalWrite(IN4M, 0);
            digitalWrite(IN3M, 1);
            analogWrite(PWMB, pwm);
        }
    }
}

// the loop function runs over and over again forever
void loop() {
  moveMotor(true, true, 100)        // move motor 1
  moveMotor(false, true, 100)       // move motor 2
  delay(1000);                      // wait for a second
  moveMotor(true, false, 100)       // move motor 1
  moveMotor(false, false, 100)      // move motor 2
  delay(1000);                      // wait for a second
}
