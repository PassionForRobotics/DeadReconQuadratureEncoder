/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <DiffDrive.h>

DiffDrive dd(RADIUS, LENGTH, 10);

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  dd.testSpeed(60.0);
}

void loop()
{
  // LED toggle
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  dd.update();


}
