#include <Arduino.h>
#include <DiffDrive.h>

Motor::Motor(){}

void Motor::begin(int _pin_A, int _pin_B)
{

  this->pin_1_ = _pin_A;
  this->pin_2_ = _pin_B;

  pinMode(this->pin_1_, OUTPUT);
  pinMode(this->pin_2_, OUTPUT);
}

static double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void Motor:: setPWM(int _s)
{
  //TODO: max_vel and acc check

  this->pwm_ = _s;

  //int s = constrain(_s, -this->max_velocity_, this->max_velocity_);
  int sign =  _s > 0.0 ? 1 : -1;
  //s = (int)mapf( s, 0, this->max_velocity_, 0, 255);

  //  this->bot_linear_vel_right_ = s ;

  if(sign == 1)
  {
    analogWrite(this->pin_1_, _s );
    analogWrite(this->pin_2_, 0 );
  }
  else
  {
    analogWrite(this->pin_1_, 0 );
    analogWrite(this->pin_2_, _s );
  }

}
