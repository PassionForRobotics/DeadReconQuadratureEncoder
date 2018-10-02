#include <Arduino.h>
#include <DiffDrive.h>

void PositionController::begin(Motor* _motor, QuadEncoder* _quad_encoder, float _wheel_radius)
{
  this->motor_ = _motor;
  this->quad_encoder_ = _quad_encoder;
  this->wheel_radius_ = _wheel_radius;

  this->lr_ = this->motor_->getLR();

  this->pid_ = new PID((double *)&this->radial_dist_, (double *)&this->pid_output_, (double *)(&this->pid_setpoint_),0.1,0.5,0.000010,P_ON_M, DIRECT);
  this->pid_->SetSampleTime(UPDATE_TIME/1000.0);
  this->speed_controller_ = new SpeedController();
  this->speed_controller_->begin(this->motor_, this->quad_encoder_, this->wheel_radius_);

}

void PositionController::update ()
{
  unsigned long int curr_time = (unsigned long int)micros();
  if(curr_time - this->last_update_time_ >= UPDATE_TIME ) // 10ms
  {
    this->last_update_time_ = curr_time;
    this->speed_controller_->calculate(); // having calculated it here is ok for the update later
    this->radial_dist_ = this->speed_controller_->getRadialDist();

    float speed = (this->pid_output_*10.0)/UPDATE_TIME; // dist*10/t_us = dist/1000


    //Serial.print((int)this->lr_, HEX);Serial.print(" pc.speed: ");Serial.println(speed);


    // Commented for initial speed test
    //this->speed_controller_->setSpeed(speed);

    this->speed_controller_->update(); // this statement can be before compute. need to think
  }
}

void PositionController::setPosition(float _dist)
{
  this->pid_setpoint_ = _dist;
}

void PositionController::setPositionPID(float _P, float _I, float _D)
{
  this->pid_->SetTunings(_P, _I, _D, P_ON_M);
}

void PositionController::setSpeedPID(float _P, float _I, float _D)
{
  this->speed_controller_->setPID(_P, _I, _D);
}

void PositionController::testPosition(float _dist)
{
  //Serial.print((int)this->lr_, HEX);Serial.print(" pc._dist: ");Serial.println(_dist);
  this->setPosition(_dist);

}

void PositionController::testSpeed(float _speed)
{
  //Serial.print((int)this->lr_, HEX);Serial.print(" pc._speed: ");Serial.println(_speed);
  this->speed_controller_->setSpeed(_speed);
}
