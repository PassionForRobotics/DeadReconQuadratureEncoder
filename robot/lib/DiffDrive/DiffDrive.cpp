#include <DiffDrive.h>

DiffDrive *This;

DiffDrive::DiffDrive(float _wheel_radius,
           float _wheel_base,
           float _update_duration)
{
  This = this;
  this->wheel_base_ = _wheel_base;
  this->wheel_radius_ = wheel_radius_;
  this->update_duration_ = _update_duration;

  this->Mr_ = new Motor();
  this->Mr_->begin(MOTOR_1_PIN_1, MOTOR_1_PIN_2);

  this->Ml_ = new Motor();
  this->Ml_->begin(MOTOR_2_PIN_1, MOTOR_2_PIN_2);

  this->Ql_ = new QuadEncoder(ENCODER_LEFTA_PIN, ENCODER_LEFTB_PIN);

  this->Qr_ = new QuadEncoder(ENCODER_RIGHTA_PIN, ENCODER_RIGHTB_PIN);

  this->position_controller_left_ = new PositionController();
  this->position_controller_left_->begin(this->Ml_, this->Ql_, this->wheel_radius_);

  this->position_controller_right_ = new PositionController();
  this->position_controller_right_->begin(this->Mr_, this->Qr_, this->wheel_radius_);

  this->position_controller_left_->setPID(1.0, 0.0, 0.0);
  this->position_controller_right_->setPID(1.0, 0.0, 0.0);

  this->initQuadEncoders();

}

void DiffDrive::ql_pulseA()
{
  This->Ql_->pulseA();
}


void DiffDrive::ql_pulseB()
{
  This->Ql_->pulseB();
}


void DiffDrive::qr_pulseA()
{
  This->Qr_->pulseA();
}


void DiffDrive::qr_pulseB()
{
  This->Qr_->pulseB();
}


void DiffDrive::initQuadEncoders()
{
  this->Ql_->begin(this->ql_pulseA, this->ql_pulseB);
  this->Qr_->begin(this->qr_pulseA, this->qr_pulseB);
}

void DiffDrive::update()
{
  this->position_controller_left_->update();
  this->position_controller_right_->update();
}

void DiffDrive::testDist(float _dist)
{
  this->position_controller_left_->testPosition(_dist);
}

void DiffDrive::testSpeed(float _speed)
{
  this->position_controller_left_->testSpeed(_speed);
}
