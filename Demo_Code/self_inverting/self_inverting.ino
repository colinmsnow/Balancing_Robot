/* Code by Shashank Swaminathan for the ESA class project
 * For the ESA inverted pendulum class project.
 *
 * This is the code to get the Rocky to self-invert.
 * Currently not a working version.
 * Built off the Rocky_Balancing_Starter_Code
 */

#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"

extern int32_t angle_accum;
extern int32_t speedLeft;
extern int32_t driveLeft;
extern int32_t distanceRight;
extern int32_t speedRight;
extern int32_t distanceLeft;
extern int32_t distanceRight;
float speedCont = 0;
float displacement_m = 0;
int16_t limitCount = 0;
uint32_t cur_time = 0;
float distLeft_m;
float distRight_m;

extern uint32_t delta_ms;
float measured_speedL = 0;
float measured_speedR = 0;
float desSpeedL=0;
float desSpeedR =0;
float dist_accumL_m = 0;
float dist_accumR_m = 0;
float dist_accum = 0;
float speed_err_left = 0;
float speed_err_right = 0;
float speed_err_left_acc = 0;
float speed_err_right_acc = 0;
float errAccumRight_m = 0;
float errAccumLeft_m = 0;
float prevDistLeft_m = 0;
float prevDistRight_m = 0;
float angle_rad_diff = 0;
// angle in radians
float angle_rad;
// acculumlated angle in radians
float angle_rad_accum = 0;
// previous angle measurement
float angle_prev_rad = 0;
// Unsigned upper limit on a.x
const float AX_THRESH = 1;
// signed threshold on a.g + a.y
const float AY_THRESH = -9.8;
extern int32_t displacement;
int32_t prev_displacement=0;
uint32_t prev_time;

#define G_RATIO (162.5)

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;

// Replace the 0.25 with the value obtained
// from the Gyro calibration procedure
#define FIXED_ANGLE_CORRECTION (0.24)

void BalanceRocky()
{

    // Enter the control parameters here

    float Kp = 1903.8;
    float Ki = 13248;

    float Ci = -680.34;

    float Jp = 85.71;
    float Ji = -1426.4;

		// Control velocities for motors
    float v_c_L, v_c_R;
		// Desired velocity set by controller
    float v_d = 0;

    // Variables available to you are:
    // angle_rad  - angle in radians
    // angle_rad_accum - integral of angle
    // measured_speedR - RW speed (m/s)
    // measured_speedL - LW speed (m/s)
    // distLeft_m - distance traveled by LW
    // distRight_m - distance traveled by RW
		 // Both distances are in meters
     // Integral of velocities
    // dist_accum - integral of the distance

		// Desired velocity from angle controller
		v_d = Kp*angle_rad + Ki*angle_rad_accum;

		// The next two lines implement the
		// feedback controller for the motor.
    // Two separate velocities are calculated.
    //
    // We use a trick here by criss-crossing
		// the distance from left to right
		// and right to left.
		// Ensures that the motors are balanced

    v_c_R = v_d - Jp*measured_speedR -
				Ji*distLeft_m  - dist_accum*Ci;
    v_c_L = v_d - Jp*measured_speedL -
				Ji*distRight_m - dist_accum*Ci;

    // save desired speed for debugging
    desSpeedL = v_c_L;
    desSpeedR = v_c_R;

		// motor control must be between +- 300.
		// Clips values to be within that range.
    if(v_c_L > 300) v_c_L = 300;
    if(v_c_R > 300) v_c_R = 300;
    if(v_c_L < -300) v_c_L = -300;
    if(v_c_R < -300) v_c_R = -300;

    // Set the motor speeds
    motors.setSpeeds((int16_t) (v_c_L),
(int16_t)(v_c_R));

}

void setup()
{
  // Uncomment if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  Serial.begin(9600);
  prev_time = 0;
  displacement = 0;
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  angle_accum = 0;

  ledGreen(0);
  ledYellow(0);
}



int16_t time_count = 0;
extern int16_t angle_prev;
int16_t start_flag = 0;
int16_t start_counter = 0;
int16_t invertDir = 1;
uint32_t inv_time = 0;
const uint8_t INV_TIME_MS = 650;
void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;

void UpdateSensors()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();

  // Perform the balance updates at 100 Hz.
  balanceUpdateDelayedStatus = ms -
			lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  // call functions to integrate
	// encoders and gyros
  balanceUpdateSensors();

  if (imu.a.x < 0)
  {
    lyingDown();
    isBalancingStatus = false;
  }
  else
  {
    isBalancingStatus = true;
  }
}

void GetMotorAndAngleMeasurements()
{
    // convert distance calculation into meters
    // and integrate distance
    distLeft_m = ((float)distanceLeft)/
				((float)G_RATIO)/12.0*80.0/
				1000.0*3.14159;
    distRight_m = ((float)distanceRight)/
				((float)G_RATIO)/12.0*80.0/
				1000.0*3.14159;
    dist_accum += (distLeft_m+distRight_m)*
				0.01/2.0;

    // compute left and right wheel speed in meters/s
    measured_speedL = speedLeft/((float)G_RATIO)/
				12.0*80.0/1000.0*3.14159*100.0;
    measured_speedR = speedRight/((float)G_RATIO)/
				12.0*80.0/1000.0*3.14159*100.0;

    prevDistLeft_m = distLeft_m;
    prevDistRight_m = distRight_m;


    // this integrates the angle
    angle_rad_accum += angle_rad*0.01;
    // this is the derivative of the angle
    angle_rad_diff = (angle_rad-
angle_prev_rad)/0.01;
    angle_prev_rad  = angle_rad;

}

void balanceResetAccumulators()
{
    errAccumLeft_m = 0.0;
    errAccumRight_m = 0.0;
    speed_err_left_acc = 0.0;
    speed_err_right_acc = 0.0;
}

void loop()
{
  // Serial print controller
  static uint32_t prev_print_time = 0;
	// RW vs. LW encoder distance
  int16_t distanceDiff;
  static float del_theta = 0;
  char enableLongTermGyroCorrection = 1;

  cur_time = millis(); // current time in ms


  if((cur_time - prev_time) > UPDATE_TIME_MS){
			UpdateSensors(); // update sensors

			// calculate the angle in radians.
			// The FIXED_... comes from angle calib.
			// del_theta corrects for long-term drift
			angle_rad = ((float)angle)/1000/180*3.14159 - FIXED_ANGLE_CORRECTION - del_theta;

			// check if angle is within upright bounds
			if(angle_rad > 0.1 || angle_rad < -0.1)
			{
					start_counter = 0;
			}

		if(abs(imu.a.x) < AX_THRESH &&
			 imu.a.y < AY_THRESH && !start_flag)
		{
				balanceResetEncoders();
				angle = 0;
				start_flag = 1;
				BalanceRocky();
				buzzer.playFrequency(DIV_BY_10 | 445,
1000, 15);
				Serial.println("Starting");
				ledYellow(1);
		}

  // if start_flag, do balancing
  if(start_flag)
  {
    GetMotorAndAngleMeasurements();
    if(enableLongTermGyroCorrection)
				// Handle gyro drift
				// Assumes gyro is standing
				del_theta = 0.999*del_theta+
						0.001*angle_rad;
    // Control the robot
    BalanceRocky();
  }
  else {
    if (cur_time - inv_time >= INV_TIME_MS) {
      // save the last time LED was blinked
      inv_time = cur_time;
      invertDir = -1*invertDir;
      motors.setSpeeds(invertDir*100,
invertDir*100);
    }
  }
  prev_time = cur_time;
  }
// if the robot is more than 45 degrees, ...
// OR consider: Just flip it back up
  if(start_flag &&
		 (angle_rad > .78 || angle_rad < -0.78))
  {
    start_flag = 0;
  }

// kill switch
  if(buttonA.getSingleDebouncedPress())
  {
      motors.setSpeeds(0,0);
      while(!buttonA.getSingleDebouncedPress());
  }

// Print every 105 ms
// Avoid 10ms multiples to not hog processor
if(cur_time - prev_print_time > 103)
  {
        Serial.print(angle_rad);
        Serial.print("\t");
        Serial.print(distLeft_m);
        Serial.print("\t");
        Serial.print(measured_speedL);
        Serial.print("\t");
        Serial.print(measured_speedR);
        Serial.print("\t");
       Serial.println(speedCont);
       prev_print_time = cur_time;
  }
}
