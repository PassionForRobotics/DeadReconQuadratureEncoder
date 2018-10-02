/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <DiffDrive.h>

DiffDrive dd(RADIUS, LENGTH, 10);

// testing the PID response
#define MAX_SPEED_CHANGES 16
float g_speed_set_points[MAX_SPEED_CHANGES] = {-80.0, 150.0, 130.0, 190.0, 120.0, 60.0, 220.0, 80.0, 40.0, 70.0, -50, -100, -150, -210, 0, 150};
int g_speed_set_point_index=0;
bool use_variable_speeds = false;

void setup()
{
  Serial.begin(115200);
  dd.testSpeed(0.0);
  delay(1000);
  Serial.println("[Restart]");
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  dd.testSpeed(100.0);
  //dd.testDist(60.0);
}

void loop()
{
  // LED toggle
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  if(0==(millis()%5000))
  {
    dd.testSpeed( g_speed_set_points[g_speed_set_point_index]);
    g_speed_set_point_index++;
    g_speed_set_point_index = g_speed_set_point_index % MAX_SPEED_CHANGES;
  }

  dd.update();




}
