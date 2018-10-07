#include <Arduino.h>
#include <DiffDrive.h>

void PositionController::begin(Motor* _motor, QuadEncoder* _quad_encoder, float _wheel_radius)
{
        this->motor_ = _motor;
        this->quad_encoder_ = _quad_encoder;
        this->wheel_radius_ = _wheel_radius;

        this->lr_ = this->motor_->getLR();

        this->position_ = 0.0f;
        this->pid_output_ = 0.0f;
        this->pid_setpoint_ = 0.0f;

        this->speed_controller_ = new SpeedController();
        this->speed_controller_->begin(this->motor_, this->quad_encoder_, this->wheel_radius_);

        this->pid_ = new PID((double *)this->speed_controller_->getPositionPIDIn()
                             , (double *)&this->pid_output_
                             , (double *)(&this->pid_setpoint_)
                             , PID_POSITION_P, PID_POSITION_I, PID_POSITION_D, P_ON_M, DIRECT); // direct to motor
        //, 6.25, 0.0, 2500.0,P_ON_M, DIRECT); // via speed controller
        //, 1000.0, 8300.0, 0.0001,P_ON_M, DIRECT);
        this->pid_->SetSampleTime(UPDATE_TIME/1000.0);
        this->pid_->SetMode(AUTOMATIC);
        this->pid_->SetOutputLimits(-255, 255);


}

float PositionController::getPosition()
{
        return (*this->speed_controller_->getPositionPIDIn());
}

void PositionController::update (int _mode)
{
        unsigned long int curr_time = (unsigned long int)micros();
        if(curr_time - this->last_update_time_ >= UPDATE_TIME ) // 10ms
        {
                this->last_update_time_ = curr_time;
                this->speed_controller_->calculate(); // having calculated it here is ok for the update later
                this->position_ = this->getPosition();

                ////////////////
                float error = this->pid_setpoint_ - this->position_;// - this->radial_dist_;
                // this->accumulated_error_ += error;
                // float delta_error = error - this->last_error_;
                //
                // float pid = error * this->pid_->GetKp() + this->accumulated_error_ * this->pid_->GetKi()   + delta_error *  this->pid_->GetKd();
                //
                // this->last_error_ = error;
                /////////////////

                float speed = (this->pid_output_*10.0)/UPDATE_TIME; // dist*10/t_us = dist/1000

                this->pid_->Compute();

                if(POSITION_ATTANED_THRESH>abs(error))
                {
                        // position attained
                        this->has_position_attained_ = true;

                }
                //Serial.print((int)this->lr_, HEX);Serial.print(" pc.speed: ");Serial.println(speed);

                // Commented for initial speed test
                //if(true!=this->test_speed_)
                if(CONTROLLER_POSITION == _mode)
                {
                        //this->speed_controller_->setSpeed(this->pid_output_);
                        this->motor_->setPWM(this->pid_output_);
                        Serial.print((int)this->lr_, HEX);
                        Serial.print(","); Serial.print(this->position_);
                        Serial.print(","); Serial.print(this->pid_setpoint_);
                        Serial.print(","); Serial.print(this->pid_output_);
                        Serial.print(","); Serial.print(this->has_position_attained_*250);
                        Serial.print(","); Serial.println(this->speed_controller_->getRevolutions());
                }


                this->speed_controller_->update(_mode); // this statement can be before compute. need to think



        }
}

void PositionController::setPosition(float _dist)
{
        this->pid_setpoint_ = _dist;
        this->has_position_attained_ = false;
}

void PositionController::setPositionPID(float _P, float _I, float _D)
{
        this->position_ = 0;
        this->speed_controller_->reset();
        this->pid_->SetTunings(_P, _I, _D, P_ON_M);
}

void PositionController::setSpeedPID(float _P, float _I, float _D)
{
        this->speed_controller_->reset();
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
