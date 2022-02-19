#define IN1M 7 // motor1
#define IN2M 6 // motor 1

#define IN3M 13 // motor 2
#define IN4M 12 // motor 2

#define PWMA 9 // motor 1
#define PWMB 10 // motor 2

#define STBY 8

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
    
    delay(300);
}

void moveMotor(bool motor, bool direction, int pwm) // set pwm between 0 - 255
{
    if (motor)
    {
        // left motor
        if (direction)
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
        if (direction)
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
    moveMotor(true, true, 255);        // move motor 1
    moveMotor(false, true, 255);       // move motor 2
    delay(1000);                       // wait for a second
    moveMotor(true, false, 255);       // move motor 1
    moveMotor(false, false, 255);      // move motor 2
    delay(1000);                       // wait for a second
}
