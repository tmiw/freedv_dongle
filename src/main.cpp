#include <Arduino.h>

const int ledPin = 13;

void setup()
{ 
    pinMode(ledPin, OUTPUT);
}

void loop() 
{
    digitalWrite(ledPin, HIGH);   // set the LED on
    delay(250);                  // wait for 250ms
    digitalWrite(ledPin, LOW);    // set the LED off
    delay(250);                  // wait for 250ms
}

