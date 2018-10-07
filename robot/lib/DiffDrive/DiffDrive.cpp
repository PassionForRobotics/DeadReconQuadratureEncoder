#include <DiffDrive.h>
#include <SerialCommand.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

static SerialCommand g_sCmd;

DiffDrive *This;

DiffDrive::DiffDrive(float _wheel_radius, float _wheel_base,
                     float _update_duration) {
        This = this;
        // this->sCmd_ = &g_sCmd;

        this->wheel_base_ = _wheel_base;
        this->wheel_radius_ = wheel_radius_;
        this->update_duration_ = _update_duration;

        this->Mr_ = new Motor();
        this->Mr_->begin(RIGHT, MOTOR_1_PIN_1, MOTOR_1_PIN_2);

        this->Ml_ = new Motor();
        this->Ml_->begin(LEFT, MOTOR_2_PIN_1, MOTOR_2_PIN_2);

        this->Ql_ = new QuadEncoder(LEFT, ENCODER_LEFTA_PIN, ENCODER_LEFTB_PIN);

        this->Qr_ = new QuadEncoder(RIGHT, ENCODER_RIGHTA_PIN, ENCODER_RIGHTB_PIN);

        this->position_controller_left_ = new PositionController();
        this->position_controller_left_->begin(this->Ml_, this->Ql_,
                                               this->wheel_radius_);

        this->position_controller_right_ = new PositionController();
        this->position_controller_right_->begin(this->Mr_, this->Qr_,
                                                this->wheel_radius_);

        this->position_controller_left_->setPositionPID(PID_POSITION_P, PID_POSITION_I, PID_POSITION_D);
        this->position_controller_right_->setPositionPID(PID_POSITION_P, PID_POSITION_I, PID_POSITION_D);

        this->position_controller_left_->setSpeedPID(PID_SPEED_P, PID_SPEED_I, PID_SPEED_D);
        this->position_controller_right_->setSpeedPID(PID_SPEED_P, PID_SPEED_I, PID_SPEED_D);

        this->initQuadEncoders();

        g_sCmd.addCommand("setVel", this->testSpeed); // setSpeed L 0.0
        g_sCmd.addCommand("setDist", this->testPosition); // setSpeed L 0.0
        g_sCmd.addCommand("setWhlSt", this->setWheelStatus); // setWhlSt L E
        g_sCmd.addCommand("setPosPID",
                          this->parseAndSetPositionPIDValues); // setPID L 0.0 0.0 0.0
        g_sCmd.addCommand("setSpdPID",
                          this->parseAndSetSpeedPIDValues); // setPID L 0.0 0.0 0.0
        g_sCmd.addCommand("setCtrlMode",
                          this->setControllerMode);         // setCtrlMode S

        // Default mode
        this->setControllerMode(CONTROLLER_POSITION);
        this->enable_wheel_left_ = true;
        this->enable_wheel_right_ = true;
}

void DiffDrive::setWheelStatus()
{
        char *arg;
        char s, ed; // side and enable disable
        arg = g_sCmd.next();
        if (arg != NULL) {
                s = arg[0];
        } else {
                return;
                // Serial.println("No arguments");
        }

        arg = g_sCmd.next();
        if (arg != NULL)
        {
                ed = arg[0];
        }
        else
        {
                return;
        }

        if ('L' == s)
        {
                if('E'==ed)
                {
                        This->enable_wheel_left_ = true;
                }
                else if('D'==ed)
                {
                        This->enable_wheel_left_ = false;
                }
                else
                {
                        // reuturn;
                }

        }
        if ('R' == s)
        {

                if('E'==ed)
                {
                        This->enable_wheel_right_ = true;
                }
                else if('D'==ed)
                {
                        This->enable_wheel_right_ = false;
                }
                else
                {
                        // reuturn;
                }
        }

}


void DiffDrive::setControllerMode()
{
        char *arg;
        char c;
        arg = g_sCmd.next();
        if (arg != NULL) {
                c = arg[0];
        } else {
                return;
                // Serial.println("No arguments");
        }

        if ('S' == c) {
                This->setControllerMode(CONTROLLER_SPEED);
        }
        if ('P' == c) {
                This->setControllerMode(CONTROLLER_POSITION);
        }

}

void DiffDrive::testSpeed()
{
        char *arg;
        char c;
        arg = g_sCmd.next();
        if (arg != NULL) {
                c = arg[0];
        } else {
                return;
                // Serial.println("No arguments");
        }

        arg = g_sCmd.next();
        if (arg != NULL) {
                float s = atof(arg);

                if ('L' == c) {
                        This->position_controller_left_->testSpeed(s);
                }
                if ('R' == c) {
                        This->position_controller_right_->testSpeed(s);
                }
                // Serial.print("P: ");
                // Serial.print(p,6);
        } else {
                // Serial.println("No arguments");
        }
}

void DiffDrive::testPosition()
{
        char *arg;
        char c;
        arg = g_sCmd.next();
        if (arg != NULL)
        {
                c = arg[0];
        }
        else
        {
                return;
                // Serial.println("No arguments");
        }

        arg = g_sCmd.next();
        if (arg != NULL)
        {
                float s = atof(arg);

                if ('L' == c)
                {
                        This->position_controller_left_->testPosition(s);
                }
                if ('R' == c)
                {
                        This->position_controller_right_->testPosition(s);
                }
                // Serial.print("P: ");
                // Serial.print(p,6);
        }
        else
        {
                // Serial.println("No arguments");
        }
}

void DiffDrive::parseAndSetSpeedPIDValues()
{
        float p, i, d;
        char *arg;

        char c;
        // Serial.println("We're in processCommand");

        arg = g_sCmd.next();
        if (arg != NULL)
        {
                c = arg[0]; // Converts a char string to an integer

                // Serial.print("P: ");
                // Serial.print(p,6);
        }
        else
        {
                return;
                // Serial.println("No arguments");
        }

        // Serial.println("We're in processCommand");
        arg = g_sCmd.next();
        if (arg != NULL)
        {
                p = atof(arg); // Converts a char string to an integer
                               // Serial.print("P: ");
                               // Serial.print(p,6);
        }
        else
        {
                // Serial.println("No arguments");
        }

        arg = g_sCmd.next();
        if (arg != NULL)
        {
                i = atof(arg);
                // Serial.print(" I: ");
                // Serial.print(i,6);
        }
        else
        {
                // Serial.println("No second argument");
        }

        arg = g_sCmd.next();
        if (arg != NULL)
        {
                d = atof(arg);
                // Serial.print(" D: ");
                // Serial.println(d,6);
        }
        else
        {
                // Serial.println("No second argument");
        }

        if ('L' == c)
        {
                This->position_controller_left_->setSpeedPID(p, i, d);
        }
        if ('R' == c)
        {
                This->position_controller_right_->setSpeedPID(p, i, d);
        }
}

void DiffDrive::parseAndSetPositionPIDValues()
{
        float p, i, d;
        char *arg;

        char c;
        // Serial.println("We're in processCommand");

        arg = g_sCmd.next();
        if (arg != NULL)
        {
                c = arg[0]; // Converts a char string to an integer

                // Serial.print("P: ");
                // Serial.print(p,6);
        }
        else
        {
                return;
                // Serial.println("No arguments");
        }

        // Serial.println("We're in processCommand");
        arg = g_sCmd.next();
        if (arg != NULL)
        {
                p = atof(arg); // Converts a char string to an integer
                               // Serial.print("P: ");
                               // Serial.print(p,6);
        }
        else
        {
                // Serial.println("No arguments");
        }

        arg = g_sCmd.next();
        if (arg != NULL)
        {
                i = atof(arg);
                // Serial.print(" I: ");
                // Serial.print(i,6);
        }
        else
        {
                // Serial.println("No second argument");
        }

        arg = g_sCmd.next();
        if (arg != NULL)
        {
                d = atof(arg);
                // Serial.print(" D: ");
                // Serial.println(d,6);
        }
        else
        {
                // Serial.println("No second argument");
        }

        if ('L' == c)
        {
                This->position_controller_left_->setPositionPID(p, i, d);
        }
        if ('R' == c)
        {
                This->position_controller_right_->setPositionPID(p, i, d);
        }
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

        if(true == this->enable_wheel_left_)
        {
                this->position_controller_left_->update(this->controller_mode_);
        }

        if(true == this->enable_wheel_right_)
        {
                this->position_controller_right_->update(this->controller_mode_);
        }
        g_sCmd.readSerial();
}

void DiffDrive::testDist(float _dist)
{

        if(CONTROLLER_POSITION == this->controller_mode_)
        {
                // Serial.print("dd._dist: ");Serial.println(_dist);
                this->position_controller_left_->testPosition(_dist);
                this->position_controller_right_->testPosition(_dist);
        }
        else
        {
                Serial.println();
                Serial.print("WARN: "); Serial.print(__FILENAME__);  Serial.print(":");   Serial.print(__LINE__); Serial.println(" Invalid controller mode.");
        }

}

void DiffDrive::testSpeed(float _speed)
{

        if(CONTROLLER_SPEED == this->controller_mode_)
        {
                // Serial.print("dd._speed: ");Serial.println(_speed);
                this->position_controller_left_->testSpeed(_speed);
                this->position_controller_right_->testSpeed(_speed);
        }
        else
        {
                Serial.println();
                Serial.print("WARN: ");
                Serial.print(__FILENAME__); Serial.print(":");   Serial.print(__LINE__);
                Serial.println(" Invalid controller mode.");
        }

}

bool DiffDrive::hasAttained(int _side, int _type)
{
        bool has_it = false;
        if(LEFT == _side)
        {
                if(CONTROLLER_POSITION == _type)
                {
                        has_it = this->position_controller_left_->hasPositionAttained();
                }

                if(CONTROLLER_SPEED == _type)
                {
                        has_it = this->position_controller_left_->hasSpeedAttained();
                }

        }
        if(RIGHT == _side)
        {
                if(CONTROLLER_POSITION == _type)
                {
                        has_it = this->position_controller_right_->hasPositionAttained();
                }

                if(CONTROLLER_SPEED == _type)
                {
                        has_it = this->position_controller_right_->hasSpeedAttained();
                }
        }
        return has_it;
}
