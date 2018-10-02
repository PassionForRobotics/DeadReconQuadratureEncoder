#include <DiffDrive.h>
#include <SerialCommand.h>

static SerialCommand g_sCmd;

DiffDrive *This;

DiffDrive::DiffDrive(float _wheel_radius,
           float _wheel_base,
           float _update_duration)
{
  This = this;
  //this->sCmd_ = &g_sCmd;

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
  this->position_controller_left_->begin(this->Ml_, this->Ql_, this->wheel_radius_);

  this->position_controller_right_ = new PositionController();
  this->position_controller_right_->begin(this->Mr_, this->Qr_, this->wheel_radius_);

  this->position_controller_left_->setPositionPID(1.0, 0.0, 0.0);
  this->position_controller_right_->setPositionPID(1.0, 0.0, 0.0);

  this->position_controller_left_->setSpeedPID(0.0, 11.0, 0.0);
  this->position_controller_right_->setSpeedPID(0.0, 11.0, 0.0);

  this->initQuadEncoders();

  g_sCmd.addCommand("setSpeed", this->testSpeed);  //setSpeed L 0.0
  g_sCmd.addCommand("setPID", this->parseAndSetPIDValues);  //setPID L 0.0 0.0 0.0

}


void DiffDrive::testSpeed( )
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
    //Serial.println("No arguments");
  }

  arg = g_sCmd.next();
  if (arg != NULL)
  {
    float s = atof(arg);

    if('L'==c)
    {
      This->position_controller_left_->testSpeed(s);
    }
    if('R'==c)
    {
      This->position_controller_right_->testSpeed(s);
    }
    //Serial.print("P: ");
    //Serial.print(p,6);
  }
  else
  {
    //Serial.println("No arguments");
  }
}

void DiffDrive::parseAndSetPIDValues()
{
  float p,i,d;
  char *arg;

  char c;
  //Serial.println("We're in processCommand");

  arg = g_sCmd.next();
  if (arg != NULL)
  {
    c = arg[0];    // Converts a char string to an integer


    //Serial.print("P: ");
    //Serial.print(p,6);
  }
  else
  {
    return;
    //Serial.println("No arguments");
  }

  //Serial.println("We're in processCommand");
  arg = g_sCmd.next();
  if (arg != NULL)
  {
    p = atof(arg);    // Converts a char string to an integer
    //Serial.print("P: ");
    //Serial.print(p,6);
  }
  else
  {
    //Serial.println("No arguments");
  }

  arg = g_sCmd.next();
  if (arg != NULL)
  {
    i = atof(arg);
    //Serial.print(" I: ");
    //Serial.print(i,6);
  }
  else
  {
    //Serial.println("No second argument");
  }

  arg = g_sCmd.next();
  if (arg != NULL)
  {
    d = atof(arg);
    //Serial.print(" D: ");
    //Serial.println(d,6);
  }
  else
  {
    //Serial.println("No second argument");
  }

  if('L'==c)
  {
    This->position_controller_left_->setSpeedPID(p, i, d);
  }
  if('R'==c)
  {
     This->position_controller_right_->setSpeedPID(p, i, d);
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
  this->position_controller_left_->update();
  this->position_controller_right_->update();
  g_sCmd.readSerial();

}

void DiffDrive::testDist(float _dist)
{
  //Serial.print("dd._dist: ");Serial.println(_dist);
  this->position_controller_left_->testPosition(_dist);
  this->position_controller_right_->testPosition(_dist);
}

void DiffDrive::testSpeed(float _speed)
{
  //Serial.print("dd._speed: ");Serial.println(_speed);

  this->position_controller_left_->testSpeed(_speed);
  this->position_controller_right_->testSpeed(_speed);
}
