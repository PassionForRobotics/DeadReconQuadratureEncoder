#include <Arduino.h>
//#include "DeadReckoner.h"
#include "QuadEnc.h"
#include "DiffDrive.h"
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#include <SerialCommand.h>
SerialCommand sCmd;

// ENCODER PINS
#define ENCODER_LEFTA_PIN 2
#define ENCODER_LEFTB_PIN 3
#define ENCODER_RIGHTA_PIN 18
#define ENCODER_RIGHTB_PIN 19

#define MOTOR_1_PIN_1 9
#define MOTOR_1_PIN_2 10
#define MOTOR_2_PIN_1 11 //left
#define MOTOR_2_PIN_2 12


void(* resetFunc) (void) = 0; //declare reset function @ address 0

// MEASUREMENTS
// The units for all measurements must be consistent.
// You can use any length unit as desired.
#define RADIUS 35 // wheel radius in mm
#define LENGTH 150 // wheel base length in mm
#define TICKS_PER_REV 334

// TIME INTERVALS
#define POSITION_COMPUTE_INTERVAL 10 // milliseconds
#define SEND_INTERVAL 500 // milliseconds
#define MOTOR_SPEED_SEND_INTERVAL 10 // milliseconds
int motor_speed_send_interval = 10;

// Previous times for computing elapsed time.
unsigned long g_prev_position_compute_time = 0, g_prev_send_time = 0, g_prev_motor_speed_send_time, g_prev_speed_change_time;

QuadEncoder Ql(ENCODER_LEFTA_PIN, ENCODER_LEFTB_PIN);
QuadEncoder Qr(ENCODER_RIGHTA_PIN, ENCODER_RIGHTB_PIN);

//DeadReckoner deadReckoner(Ql.getPosition(), Qr.getPosition(), TICKS_PER_REV, RADIUS, LENGTH);

DiffDrive DD(MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2, RADIUS, LENGTH);
bool g_velocities_attained=false;

float g_speed_set_point = 140.0;

#define MAX_SPEED_CHANGES 10
float g_speed_set_points[MAX_SPEED_CHANGES] = {50.0, 30.0, 90.0, 20.0, 60.0, 100.0, 80.0, 40.0, 70.0};
int g_speed_set_point_index=0;
bool use_variable_speeds = false;


float left_pid_output;
float right_pid_output;
float *pid_output=&left_pid_output;

//0.1,0.5,0.000010
PID leftPID((double *)Ql.getRPM(), (double *)&left_pid_output, (double *)(&g_speed_set_point),0.1,0.5,0.000010,P_ON_M, DIRECT);
PID rightPID((double *)Qr.getRPM(), (double *)&right_pid_output, (double *)(&g_speed_set_point),0.1,0.5,0.000010,P_ON_M, DIRECT);

PID *aPID=&leftPID; // change for tuning

QuadEncoder *Qa=&Ql;

void AutoTuneHelper(boolean start);
void changeAutoTune();
void SerialSend();

void leftPulseA()
{
  Ql.pulseA();
}

void leftPulseB()
{
  Ql.pulseB();
}

void rightPulseA()
{
  Qr.pulseA();
}

void rightPulseB()
{
  Qr.pulseB();
}


void unrecognized(const char *command)
{
  Serial.println("What?");
}

void parseAndSetPIDValues()
{
  float p,i,d;
  char *arg;

  //Serial.println("We're in processCommand");
  arg = sCmd.next();
  if (arg != NULL)
  {
    p = atof(arg);    // Converts a char string to an integer
    Serial.print("P: ");
    Serial.print(p,6);
  }
  else
  {
    Serial.println("No arguments");
  }

  arg = sCmd.next();
  if (arg != NULL)
  {
    i = atof(arg);
    Serial.print(" I: ");
    Serial.print(i,6);
  }
  else
  {
    Serial.println("No second argument");
  }

  arg = sCmd.next();
  if (arg != NULL)
  {
    d = atof(arg);
    Serial.print(" D: ");
    Serial.println(d,6);
  }
  else
  {
    Serial.println("No second argument");
  }

  aPID->SetTunings(p, i, d, P_ON_M);
  //rightPID.SetTunings(p, i, d);

}

void showPIDValues()
{

  Serial.print("P: ");
  Serial.print(aPID->GetKp(),6);
  Serial.print(" I: ");
  Serial.print(aPID->GetKi(),6);
  Serial.print(" D: ");
  Serial.println(aPID->GetKd(),6);

}

void setMotorUpdateTime()
{
  char * arg = sCmd.next();
  if (arg != NULL)
  {
    motor_speed_send_interval = atoi(arg);    // Converts a char string to an integer
    Serial.print("M dt : ");
    Serial.println(motor_speed_send_interval);
  }
  else
  {
    Serial.println("No arguments");
  }

   motor_speed_send_interval = constrain(motor_speed_send_interval, 10, 2000);
}


void setLinearVel()
{
  char * arg = sCmd.next();
  if (arg != NULL)
  {
    g_speed_set_point = atof(arg);    // Converts a char string to an integer
    Serial.print("linear velocity set point : ");
    Serial.println(g_speed_set_point);
  }
  else
  {
    Serial.println("No arguments");
  }
}


bool g_to_print=true;
void changePrintState(void)
{
  g_to_print = !g_to_print;
}

void changeAutoSpeedChange(void)
{
  use_variable_speeds = !use_variable_speeds;
}

void printSpeeds(void)
{
  Serial.print(" Ql ");
  Serial.print(*Ql.getRPM());
  Serial.print(" Qr ");
  Serial.print(*Qr.getRPM());
  Serial.print(" ");
 }

bool toggle_tuning_motor = false;
 void changeTuneMotor(void)
 {
   toggle_tuning_motor = !toggle_tuning_motor;

   if(false==toggle_tuning_motor)
   {
    aPID = &leftPID;
    Qa = &Ql;
    pid_output=&left_pid_output;
   }
  else
  {
    aPID = &rightPID;
    Qa = &Qr;
    pid_output=&right_pid_output;
  }
 }

void cmdSetup()
{
  sCmd.addCommand("PID",    parseAndSetPIDValues);
  sCmd.addCommand("readPID", showPIDValues);
  sCmd.addCommand("dtM", setMotorUpdateTime);

  sCmd.addCommand("setLV", setLinearVel);
//kp: 1.830000 ki: 0.024570 kd: 0.307000
  sCmd.addCommand("toggleAutoTune", changeAutoTune);

  sCmd.addCommand("togglePrintouts", changePrintState);
  sCmd.addCommand("toggleVarSpeeds", changeAutoSpeedChange);
  sCmd.addCommand("printSpeeds", printSpeeds);

  sCmd.addCommand("toggleTuneMotor", changeTuneMotor);


  // sCmd.addCommand("P",   parseAndSetPValue);
  // sCmd.addCommand("I",   parseAndSetIValue);
  // sCmd.addCommand("D",   parseAndSetDValue);
  sCmd.addCommand("R",   resetFunc);
  sCmd.setDefaultHandler(unrecognized);
}




byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;
double kp=2,ki=0.5,kd=2;

//double kpmodel=1.5, taup=100, theta[50];
//double outputStart=5;
double aTuneStep=40, aTuneNoise=20, aTuneStartValue=200;
unsigned int aTuneLookBack=50;

boolean tuning = false;
unsigned long  modelTime, serialTime;

//PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune((double *)Qa->getRPM(), (double *)pid_output);
// g_speed_set_point
//set to false to connect to the real world
boolean useSimulation = false;

void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(g_speed_set_point); Serial.print(" ");
  Serial.print("input: ");Serial.print((double )*Qa->getRPM(),6); Serial.print(" ");
  Serial.print("output: ");Serial.print(*pid_output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(aPID->GetKp(),6);Serial.print(" ");
    Serial.print("ki: ");Serial.print(aPID->GetKi(),6);Serial.print(" ");
    Serial.print("kd: ");Serial.print(aPID->GetKd(),6);Serial.println();
  }
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    *pid_output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    aTune.SetControlType(1);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = aPID->GetMode();
  else
    aPID->SetMode(ATuneModeRemember);
}



void setup()
{
  Serial.begin(115200);

  delay(10);

  Serial.println(" [Restart] ");

  cmdSetup();

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

  DD.begin(255.0, 0.1);

  DD.setVelocities(0.0, 0.0);

  delay(1000);

  Serial.println(" [INIT] ");

  Ql.begin(leftPulseA, leftPulseB);
  Qr.begin(rightPulseA, rightPulseB);

  toggle_tuning_motor = true;
  changeTuneMotor();


  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }


}

void loop()
{


  if (millis() - g_prev_speed_change_time >= 20000)
  {
    if(use_variable_speeds)
    {
      g_speed_set_point_index = g_speed_set_point_index % (MAX_SPEED_CHANGES-1);
      g_speed_set_point = g_speed_set_points[g_speed_set_point_index++] ;//= {5.0, 3.0, 9.0, 2.0, 6.0, 10.0, 8.0, 4.0, 7.0};

      g_prev_speed_change_time = millis();

      printSpeeds();
      Serial.print("SetPoint set as ");
      Serial.println(g_speed_set_point);
    }

}


  Ql.updateRPM();
  Qr.updateRPM();



      if(tuning)
      {
        byte val = (aTune.Runtime());
        if (val!=0)
        {
          tuning = false;
        }
        if(!tuning)
        { //we're done, set the tuning parameters
          kp = aTune.GetKp();
          ki = aTune.GetKi();
          kd = aTune.GetKd();
          aPID->SetTunings(kp,ki,kd, P_ON_M);
          AutoTuneHelper(false);
        }
      }
      else
      {
        aPID->Compute();
      }

      if(useSimulation)
      {
      }
      else
      {
        if(false==toggle_tuning_motor)
        {
          DD.setVelocitiesLR(*pid_output, 0);
        }
        else
        {
          DD.setVelocitiesLR(0,*pid_output);
        }
      }

      //rightPID.Compute();


	if (millis() - g_prev_position_compute_time > POSITION_COMPUTE_INTERVAL)
  {
		// Computes the new angular velocities and uses that to compute the new position.
		// The accuracy of the position estimate will increase with smaller time interval until a certain point.
		//deadReckoner.computePosition();
		g_prev_position_compute_time = millis();


    // long double wl = deadReckoner.getWl();
		// long double wr = deadReckoner.getWr();
    //
    // float wl_sent, wr_sent;
    // DD.getVelocities(&wl_sent, &wr_sent);
    //
    // float error = -(wl_sent - wl);
    // DD.setVelocities(error, 0.0);

    // if(( ( abs(wl_sent) < abs(wl) ) || ( abs(wr_sent) < abs(wr) ) ) && (false == g_velocities_attained))
    // {
    //   DD.setVelocities(0.0, 0.0);
    //   Serial.println("DeadRecon: Velocities attained ... stopped.");
    //   g_velocities_attained = true;
    // }


	}

  if (millis() - g_prev_motor_speed_send_time > motor_speed_send_interval)
  {
    g_prev_motor_speed_send_time = millis();

    //DD.setVelocitiesLR(left_pid_output, right_pid_output);


  }

	if (millis() - g_prev_send_time > SEND_INTERVAL)
  {

    sCmd.readSerial();
		// Cartesian coordinate of latest location estimate.
		// Length unit correspond to the one specified under MEASUREMENTS.
		// double x = deadReckoner.getX();
		// double y = deadReckoner.getY();
    //
		// // Left and right angular velocities.
		// double wl = *deadReckoner.getWl();
		// double wr = *deadReckoner.getWr();
    //
		// // getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
		// // This angle is set initially at zero before the robot starts moving.
		// double theta = deadReckoner.getTheta();
    //
		// // Total distance robot has troubled.
		// double distance = sqrt(x * x + y * y);
    //
		// Serial.print("x: "); Serial.print(x); Serial.print(" mm");
		// Serial.print(" y: "); Serial.print(y); Serial.print(" mm");
		// Serial.print(" wl: "); Serial.print(wl); Serial.print(" mm/s");
		// Serial.print(" wr: "); Serial.print(wr);  Serial.print(" mm/s");
		// Serial.print(" theta: "); Serial.print(theta*RAD_TO_DEG);  Serial.print(" D"); // theta converted to degrees.
		// Serial.print(" dist: "); Serial.print(distance);  Serial.print(" mm | ");

    if(g_to_print)
    {
      Serial.print((int32_t)*Ql.getPosition()); Serial.print(" ");
  		Serial.println((int32_t)*Qr.getPosition());
  		g_prev_send_time = millis();

      float r, l;
      DD.getVelocities(&l, &r);

      Serial.print("DiffDrive: "); Serial.print(left_pid_output);
      Serial.print(" ( "); Serial.print(l);
      Serial.print(" ) "); Serial.print(right_pid_output);
      Serial.print(" ( "); Serial.print(r);
      Serial.println(" ) ");


      SerialSend();
    } 
	}
}
