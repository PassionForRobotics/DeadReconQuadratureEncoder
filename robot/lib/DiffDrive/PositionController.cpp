#include <Arduino.h>
#include <DiffDrive.h>


/*

class PositionController : Controller
{
public:
	void begin (Motor *_motor,
	            QuadEncoder *_quad_encoder,
	            float _wheel_diameter);
	void setPosition (float _dist);
	void setPID (float _P,
	             float _I,
	             float _D);
	void update ();
	void setPIDMode (int mode=0);
  void testPosition(float _dist);
private:
	// Motor motor_;
	// QuadEncoder quad_encoder_;
	// float wheel_diameter_;
	// PID pid_;
	SpeedController *speed_controller_;
};

*/



void PositionController::begin(Motor* _motor, QuadEncoder* _quad_encoder, float _wheel_radius)
{
  this->motor_ = _motor;
  this->quad_encoder_ = _quad_encoder;
  this->wheel_radius_ = _wheel_radius;

  this->pid_ = new PID((double *)&this->radial_dist_, (double *)&this->pid_output_, (double *)(&this->pid_setpoint_),0.1,0.5,0.000010,P_ON_M, DIRECT);

  this->speed_controller_ = new SpeedController();//
  this->speed_controller_->begin(this->motor_, this->quad_encoder_, this->wheel_radius_);

}

void PositionController::update ()
{
  unsigned long int curr_time = (unsigned long int)micros();
  if(curr_time - this->last_update_time_ >= UPDATE_TIME ) // 10ms
  {
    this->last_update_time_ = curr_time;
    this->speed_controller_->calculate(); // having calculate here is ok for the update later
    this->radial_dist_ = this->speed_controller_->getRadialDist();

    this->pid_->Compute();

    float speed = (this->pid_output_*10.0)/UPDATE_TIME; // dist*10/t_us = dist/1000
    this->speed_controller_->setSpeed(speed);
    this->speed_controller_->update(); // this statement can be before compute. need to think
  }
}

void PositionController::setPosition(float _dist)
{
  this->pid_setpoint_ = _dist;
}

void PositionController::setPID(float _P, float _I, float _D)
{
  this->pid_->SetTunings(_P, _I, _D, P_ON_M);
}

void PositionController::testPosition(float _dist)
{
  this->setPosition(_dist);
}

void PositionController::testSpeed(float _speed)
{
  this->speed_controller_->setSpeed(_speed);
}
