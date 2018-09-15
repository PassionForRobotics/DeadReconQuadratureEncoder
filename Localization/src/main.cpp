
#include "DeadReckoner.h"
#include "QuadEnc.h"

// ENCODER PINS
#define ENCODER_LEFTA_PIN 2
#define ENCODER_LEFTB_PIN 3
#define ENCODER_RIGHTA_PIN 18
#define ENCODER_RIGHTB_PIN 19

// MEASUREMENTS
// The units for all measurements must be consistent.
// You can use any length unit as desired.
#define RADIUS 35 // wheel radius in mm
#define LENGTH 150 // wheel base length in mm
#define TICKS_PER_REV 334

// TIME INTERVALS
#define POSITION_COMPUTE_INTERVAL 50 // milliseconds
#define SEND_INTERVAL 100 // milliseconds

// Previous times for computing elapsed time.
unsigned long g_prev_position_compute_time = 0, g_prev_send_time = 0;
  
QuadEncoder Ql(ENCODER_LEFTA_PIN, ENCODER_LEFTB_PIN);
QuadEncoder Qr(ENCODER_RIGHTA_PIN, ENCODER_RIGHTB_PIN);

DeadReckoner deadReckoner(Ql.getPosition(), Qr.getPosition(), TICKS_PER_REV, RADIUS, LENGTH);


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

void setup()
{

  Ql.begin(leftPulseA, leftPulseB);
  Qr.begin(rightPulseA, rightPulseB);
  Serial.begin(115200);

}

void loop() {
	if (millis() - g_prev_position_compute_time > POSITION_COMPUTE_INTERVAL) {
		// Computes the new angular velocities and uses that to compute the new position.
		// The accuracy of the position estimate will increase with smaller time interval until a certain point.
		deadReckoner.computePosition();
		g_prev_position_compute_time = millis();
	}

	if (millis() - g_prev_send_time > SEND_INTERVAL) {
		// Cartesian coordinate of latest location estimate.
		// Length unit correspond to the one specified under MEASUREMENTS.
		double x = deadReckoner.getX();
		double y = deadReckoner.getY();

		// Left and right angular velocities.
		double wl = deadReckoner.getWl();
		double wr = deadReckoner.getWr();

		// getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
		// This angle is set initially at zero before the robot starts moving.
		double theta = deadReckoner.getTheta();

		// Total distance robot has troubled.
		double distance = sqrt(x * x + y * y);

		Serial.print("x: "); Serial.print(x); Serial.print(" mm");
		Serial.print("\ty: "); Serial.print(y); Serial.print(" mm");
		Serial.print("\twl: "); Serial.print(wl); Serial.print(" mm/s");
		Serial.print("\twr: "); Serial.print(wr);  Serial.print(" mm/s");
		Serial.print("\ttheta: "); Serial.print(theta*RAD_TO_DEG);  Serial.print(" D"); // theta converted to degrees.
		Serial.print("\tdist: "); Serial.print(distance);  Serial.print(" mm | ");
    Serial.print((int32_t)*Ql.getPosition()); Serial.print(" ");
		Serial.println((int32_t)*Qr.getPosition());
		g_prev_send_time = millis();
	}
}
