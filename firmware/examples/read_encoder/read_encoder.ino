/*
Copyright 2011 Lex.V.Talionis at gmail
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*/
#include <PinChangeInt.h>

// Encoder count signal
#define left_motor_encoder 2
#define right_motor_encoder 4

volatile byte burp_left=0;    // a counter to see how many times the pin has changed
volatile byte burp_right=0;    // a counter to see how many times the pin has changed

void setup() {
    Serial.begin(9600);

    pinMode(left_motor_encoder, INPUT);      //set the pin to input
    pinMode(right_motor_encoder, INPUT);     //set the pin to input

    digitalWrite(left_motor_encoder, HIGH); //use the internal pullup resistor
    digitalWrite(right_motor_encoder, HIGH); //use the internal pullup resistor

    PCintPort::attachInterrupt(left_motor_encoder, burpcountleft, CHANGE);
    PCintPort::attachInterrupt(right_motor_encoder, burpcountright, CHANGE);
}
    
void loop()
{
    Serial.println(burp_left, DEC);
    delay(500);
    Serial.println(burp_right, DEC);
    delay(500);
}

void burpcountleft()
{
    burp_left++;
}

void burpcountright()
{
    burp_right++;
}
