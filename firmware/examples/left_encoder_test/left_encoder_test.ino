#define IN1M 7  // motor 1
#define IN2M 6  // motor 1

#define IN3M 13 // motor 2
#define IN4M 12 // motor 2

#define PWMA 9  // motor 1
#define PWMB 10 // motor 2

#define STBY 8

// Encoder count signal
// #define encoder_left 2 // does not work
#define encoder_left 0

volatile long count_left = 0; // The volatile lon type is used to ensure that the value is valid when the external interrupt pulse count value is used in other functions

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

    // configure encoder pins as inputs
    pinMode(encoder_left, INPUT);

    // setup serial port
    Serial.begin(9600); // Open the serial port and set the baud rate to 115200
    delay(150);

    // setup interrupts for encoders
    attachInterrupt(encoder_left, interruptLeft, CHANGE);

    moveMotor(true, true, 30);        // move motor 1
    moveMotor(false, true, 30);       // move motor 2

    // moveMotor(true, true, 0);        // move motor 1
    // moveMotor(false, true, 0);       // move motor 2

    // remove if possible
    delay(150);
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
        if(direction)
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

void interruptLeft()
{
    if (count_left > 2147483646)
    {
        count_left = 0;
    }
    count_left ++;
}

void loop()
{

    Serial.print(count_left);
    delay(1000);

}
