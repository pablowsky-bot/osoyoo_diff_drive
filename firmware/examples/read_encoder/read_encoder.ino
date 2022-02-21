/*
 * Example on how to read an encoder
*/
#include <PinChangeInt.h>

// Encoder count signal
#define left_motor_encoder 2
#define right_motor_encoder 4

volatile long int burp_left = 0;     // a counter to see how many times the pin has changed
volatile long int burp_right = 0;    // a counter to see how many times the pin has changed

void setup()
{
    pinMode(left_motor_encoder, INPUT);      // set the pin to input
    pinMode(right_motor_encoder, INPUT);     // set the pin to input

    digitalWrite(left_motor_encoder, HIGH);  // use the internal pullup resistor
    digitalWrite(right_motor_encoder, HIGH); // use the internal pullup resistor

    PCintPort::attachInterrupt(left_motor_encoder, burpcountleft, CHANGE);
    PCintPort::attachInterrupt(right_motor_encoder, burpcountright, CHANGE);

    Serial.begin(9600);
    delay(300);
}
    
void loop()
{
    Serial.println(burp_left);
    delay(200);
}

void burpcountleft()
{
    burp_left++;
}

void burpcountright()
{
    burp_right++;
}
