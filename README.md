# DeadReconQuadratureEncoder
Perform dead recon (heading and position estimation) based on a pair of Quadrature Encoder

**STATUS**
 - ~~class uml generation~~
 - ~~impliment basic~~
 - [~~SpeedController has perfect PID set and response captured.~~](#speed-pid-tuning-manual-way)
 - ~~PositionController PID tuning evaluation~~(#how-to-tune-pid-for-position)
 - ~~Code correction for position and speed mode switching~~
 - speed unit clarification
 
**TODO**
 - Different pids vs unit evaluation and clarification
 - Forward and inverse Kinematics
 - ROS + Android + Arduino integration
 - Open for exploration
 - Test cases and Document 
 - End of Dev
  
  ___
# **Unit clarification:**
  **Commit**: https://github.com/PassionForRobotics/DeadReconQuadratureEncoder/tree/e8dc0748d5ee53b9d5ac5b49f633550fd0cf0256/robot
  
  **Cmd**:
  
|CMD|Desc|
|---|---|
|setWhlSt L D \n| Do not update left wheel status |
|setWhlSt R E \n| Keep updating right wheel status  |
|setVel R 6.28 \n| Set right wheel speed as 6.28; if rad_p_sec unit is configured, wheel take one rev per second  |
|setDist L 6.28 \n| Set left wheel position at 6.28; if rad unit is configured, wheel will take one rev and stop |
|setPosPID L 1.0 0.0 0 \n| set left wheel position pid gains as p=1.0, i=0.0 and d=0 (0.0) |
|setSpdPID R 1.0 0.0 0 \n| set right wheel speed pid gains as p=1.0, i=0.0 and d=0 (0.0) |
|setCtrlMode S/n| acvtivate speed controller and disable position controller|
|setCtrlMode P/n| acvtivate position controller and disable soeed controller|
  
  **Code**:
  ``` c++
  void SpeedController::calculate()
{
        this->ticks_ = this->quad_encoder_->getTick();
        this->delta_ticks_ = this->ticks_ - this->last_ticks_;

        float delta_revolutions = ( (this->delta_ticks_ )) / TICKS_PER_REV; // delta // unit = revolution
        float delta_angle = delta_revolutions * 360; // delta // unit = degrees
        float delta_readians = delta_revolutions * 2.0 * PI;  // delta // unit = radians // arc dist

        this->radial_dist_ += delta_readians; // unit = radians
        this->revolutions_ += delta_revolutions; // unit = revolutions
        this->linear_dist_ = this->radial_dist_ * RADIUS; // unit = unit of RADIUS (mm)

        this->radial_speed_ = ( (delta_readians)*1000.0*1000.0) / UPDATE_TIME; // rad/seconds
        this->radial_acceleration_ = ( (this->radial_speed_)*1000.0*1000.0) / UPDATE_TIME; // unit = rad/s/s

        this->rev_speed_ = ((delta_revolutions)*1000.0*1000.0) / UPDATE_TIME; // revolution/seconds
        this->rev_acceleration_ = ((this->rev_speed_)*1000.0*1000.0) / UPDATE_TIME; // revolution/seconds/seconds

        this->linear_speed_ = this->radial_speed_ * RADIUS; // unit = mm/seconds
        this->linear_acceleration_ = ( (this->linear_speed_)*1000.0*1000.0) / UPDATE_TIME; // unit = mm/s/s

        // Serial.print((int)this, HEX);Serial.print(" sc.ticks: ");Serial.println(this->ticks_);
        // //Serial.print("radial_dist: ");Serial.println(this->radial_dist_);
        // Serial.print((int)this, HEX);Serial.print(" sc.radial_speed: ");Serial.println(this->radial_speed_);

        this->last_ticks_ = this->ticks_; 
}
  ```

  ___
  
#  **How to tune PID for position:**
  **Commit**: https://github.com/PassionForRobotics/DeadReconQuadratureEncoder/tree/42396479975dbd953a2fa32465e80b0e9517efae/robot
  
  ``` c++ 
  dd.testDist(pos)
  ```
  
  * Set P to get stable oscillation, I and D as zero.
  * The value of P is Ku
  * The oscillation cycle time is Tu
  ![How_to_determine_PID_params](https://raw.githubusercontent.com/PassionForRobotics/DeadReconQuadratureEncoder/42396479975dbd953a2fa32465e80b0e9517efae/robot/img/PID_calcs.png)
  * Set calculated PID values as per the [sheet](https://github.com/PassionForRobotics/DeadReconQuadratureEncoder/blob/42396479975dbd953a2fa32465e80b0e9517efae/robot/img/ZN_PID_Loop_Tuning.xls) here 300, 2307.7, 9.75
  * Now keep modifying setpoint to check response
  ![PID respose](https://raw.githubusercontent.com/PassionForRobotics/DeadReconQuadratureEncoder/42396479975dbd953a2fa32465e80b0e9517efae/robot/img/PosPID_Response.png)

  ___
 
  # **Speed PID tuning manual way:**
  **Commit**: https://github.com/PassionForRobotics/DeadReconQuadratureEncoder/tree/2f8891bf259f6271e4b6b499b84d00a61de1b476/robot
  
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
