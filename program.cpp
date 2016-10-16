#include "WProgram.h"

int ledPin = 13;  // LED connected to digital pin 8

void setup()
{
  pinMode(ledPin, OUTPUT);  // set ledPin pin as output
}

void loop()
{
  digitalWrite(ledPin, HIGH);  // set the LED on
  delay(1000);                 // wait for a second
  digitalWrite(ledPin, LOW);   // set the LED off
  delay(1000);                 // wait for a second
}