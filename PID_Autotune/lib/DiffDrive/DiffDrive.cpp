


#include "Arduino.h"
#include "DiffDrive.h"

DiffDrive::DiffDrive(int _m1_p1, int _m1_p2, int _m2_p1, int _m2_p2, float _wheel_radius, float _wheel_separation)
{
  this->motor_1_pin_1_ = _m1_p1;
  this->motor_1_pin_2_ = _m1_p2;
  this->motor_2_pin_1_ = _m2_p1;
  this->motor_2_pin_2_ = _m2_p2;
  this->wheel_radius_ = _wheel_radius;
  this->wheel_separation_ = _wheel_separation;
}

void DiffDrive::begin(float _max_vel, float _max_acc)
{

 this->max_velocity_ = _max_vel;
 this->max_acceleration_ = _max_acc;

 pinMode(this->motor_1_pin_1_, OUTPUT);
 pinMode(this->motor_1_pin_2_, OUTPUT);
 pinMode(this->motor_2_pin_1_, OUTPUT);
 pinMode(this->motor_2_pin_2_, OUTPUT);

}

static double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DiffDrive::setVelocities(float _bot_linear_vel, float _bot_angular_vel)
{

  float bot_l_vel = _bot_linear_vel;
  float bot_a_vel = _bot_angular_vel;

  float bot_right_wheel_linear_velocity = (2.0f*bot_l_vel) + (bot_a_vel*this->wheel_separation_)/2.0f;
  float bot_left_wheel_linear_velocity  = (2.0f*bot_l_vel) - (bot_a_vel*this->wheel_separation_)/2.0f;


  // constrain(bot_left_wheel_linear_velocity, -this->max_velocity_, this->max_velocity_);
  // constrain(bot_right_wheel_linear_velocity, -this->max_velocity_, this->max_velocity_);
  //
  // int sign_right =  bot_right_wheel_linear_velocity > 0.0 ? 1 : -1;
  // int sign_left  =  bot_left_wheel_linear_velocity  > 0.0 ? 1 : -1;


  this->bot_linear_vel_left_ = bot_left_wheel_linear_velocity ;
  this->bot_linear_vel_right_ = bot_right_wheel_linear_velocity ;

  // bot_right_wheel_linear_velocity = abs(bot_right_wheel_linear_velocity);
  // bot_left_wheel_linear_velocity  = abs(bot_left_wheel_linear_velocity);
  //
  // bot_right_wheel_linear_velocity = mapf( bot_right_wheel_linear_velocity, 0, this->max_velocity_, 0, 255);
  // bot_left_wheel_linear_velocity  = mapf( bot_left_wheel_linear_velocity,  0, this->max_velocity_, 0, 255);

  setRightSpeedDir(bot_right_wheel_linear_velocity);
  setLeftSpeedDir (bot_left_wheel_linear_velocity);

  Serial.print("DiffDrive: "); Serial.print(this->bot_linear_vel_left_);
  Serial.print(" ( "); Serial.print(bot_left_wheel_linear_velocity);
  Serial.print(" ) "); Serial.print(this->bot_linear_vel_left_);
  Serial.print(" ( "); Serial.print(bot_right_wheel_linear_velocity);
  Serial.println(" ) ");

}


void DiffDrive:: setRightSpeedDir(float _s)
{
  //TODO: max_vel and acc check

  int s = constrain(_s, -this->max_velocity_, this->max_velocity_);
  int sign =  _s > 0.0 ? 1 : -1;
  s = (int)mapf( s, 0, this->max_velocity_, 0, 255);

    this->bot_linear_vel_right_ = s ;


  if(sign == 1)
  {
    analogWrite(this->motor_1_pin_1_, s );
    analogWrite(this->motor_1_pin_2_, 0 );
  }
  else
  {
    analogWrite(this->motor_1_pin_1_, 0 );
    analogWrite(this->motor_1_pin_2_, s );
  }

}

void DiffDrive:: setLeftSpeedDir (float _s)
{
  //TODO: max_vel and acc check

  int s = constrain(_s, -this->max_velocity_, this->max_velocity_);
  int sign =  _s > 0.0 ? 1 : -1;
  s = (int)mapf( s, 0, this->max_velocity_, 0, 255);

    this->bot_linear_vel_left_ = s ;

  if(sign == 1)
  {
    analogWrite(this->motor_2_pin_1_, s );
    analogWrite(this->motor_2_pin_2_, 0 );
  }
  else
  {
    analogWrite(this->motor_2_pin_1_, 0 );
    analogWrite(this->motor_2_pin_2_, s );
  }

}
