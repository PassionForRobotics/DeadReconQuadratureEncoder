# DeadReconQuadratureEncoder
Perform dead recon (heading and position estimation) based on a pair of Quadrature Encoder

**STATUS**
 - ~~class uml generation~~
 - ~~impliment basic~~
 - SpeedController has perfect PID set and response captured.
 
**TODO**
 - PositionController PID tuning evaluation
 - Forward and inverse Kinematics
 - ROS + Android + Arduino integration
 - Open for exploration
 - Test cases and Document 
 - End of Dev
  
``` c++
/*
 * DiffDrive
 * Configure the bot
 * Initialize diffdrive
 * send speed
 * keep on updating in main loop, repeatedly.
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
```
[![N|Solid](https://raw.githubusercontent.com/PassionForRobotics/DeadReconQuadratureEncoder/master/robot/img/speedcontroller_pid_response.png)](https://github.com/PassionForRobotics/DeadReconQuadratureEncoder/tree/4819c54bddf1a7e8d9cc2ce244a2f11f672f5620/robot)
