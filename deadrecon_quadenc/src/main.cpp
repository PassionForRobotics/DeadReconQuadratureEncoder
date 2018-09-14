#include <DeadReckoner.h>

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


// Number of left and right tick counts on the encoder.
volatile long long int leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;

volatile bool left_past_A, left_past_B, right_past_A, right_past_B;

DeadReckoner deadReckoner(&leftTicks, &rightTicks, TICKS_PER_REV, RADIUS, LENGTH);

// Interrupt on A changing state
void pulseLeftA() {
  // Test transition
  left_past_A = digitalRead(ENCODER_LEFTA_PIN) == HIGH;
  // and adjust counter + if A leads B
  leftTicks += (left_past_A == left_past_B) ? +1 : -1;
}

// Interrupt on B changing state
void pulseLeftB() {
  // Test transition
  left_past_B = digitalRead(ENCODER_LEFTB_PIN) == HIGH;
  // and adjust counter + if B follows A
  leftTicks += (left_past_A == left_past_B) ? +1 : -1;
}

void pulseRightA() {
  // Test transition
  right_past_A = digitalRead(ENCODER_RIGHTA_PIN) == HIGH;
  // and adjust counter + if A leads B
  rightTicks += (right_past_A == right_past_B) ? +1 : -1;
}

// Interrupt on B changing state
void pulseRightB() {
  // Test transition
  right_past_B = digitalRead(ENCODER_RIGHTB_PIN) == HIGH;
  // and adjust counter + if B follows A
  rightTicks += (right_past_A == right_past_B) ? +1 : -1;
}

/**
Attaches interrupt and disables all serial communications.
This is necessary because when interrupts are active, incoming serial communication can be lost.
*/
void attachInterrupts() {
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFTA_PIN), pulseLeftA, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHTA_PIN), pulseRightA, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFTB_PIN), pulseLeftB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHTB_PIN), pulseRightB, CHANGE);
}

void setup() {

  pinMode(ENCODER_LEFTA_PIN, INPUT);
	pinMode(ENCODER_LEFTB_PIN, INPUT);
  pinMode(ENCODER_RIGHTA_PIN, INPUT);
  pinMode(ENCODER_RIGHTB_PIN, INPUT);
 
	attachInterrupts();
	Serial.begin(115200);
}

void loop() {
	if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
		// Computes the new angular velocities and uses that to compute the new position.
		// The accuracy of the position estimate will increase with smaller time interval until a certain point.
		deadReckoner.computePosition();
		prevPositionComputeTime = millis();
	}

	if (millis() - prevSendTime > SEND_INTERVAL) {
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
    Serial.print((int32_t)leftTicks); Serial.print(" ");
		Serial.println((int32_t)rightTicks);
		prevSendTime = millis();
	}
}
