
#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_

#include <Arduino.h>

class DiffDrive
{

public:
  DiffDrive(int _m1_p1, int _m1_p2, int _m2_p1, int _m2_p2, float, float);
  void begin(float, float);
  void setVelocities(float _bot_linear_vel, float _bot_angular_vel);
  void getVelocities(float *_l, float *_r)
  {
    *_l = this->bot_linear_vel_left_;
    *_r = this->bot_linear_vel_right_;
  }
  void setVelocitiesLR(float _sl, float _sr)
  {
    setLeftSpeedDir(_sl);
    setRightSpeedDir(_sr);
  }

private:
  int motor_1_pin_1_, motor_1_pin_2_, motor_2_pin_1_, motor_2_pin_2_;
  float bot_linear_vel_left_, bot_linear_vel_right_;
  float wheel_radius_, wheel_separation_;
  float max_velocity_, max_acceleration_;

  void setRightSpeedDir(float _s);
  void setLeftSpeedDir (float _s);
};

#endif
