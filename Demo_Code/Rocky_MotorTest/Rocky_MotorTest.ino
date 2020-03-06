// wheels and drive around while balancing.
//
// To run this demo, you will need to install the LSM6 library:
//
// https://github.com/pololu/lsm6-arduino
//

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

void balanceDoDriveTicks();
extern int32_t displacement;
int32_t prev_displacement=0;


LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;

uint32_t prev_time;


void setup()
{
  // Uncomment these lines if your motors are reversed.
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
void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;

void newBalanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();

  // Perform the balance updates at 100 Hz.
 //if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  // call functions to integrate encoders and gyros
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


float testSpeed = 0;
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
float speed_err_left = 0;
float speed_err_right = 0;
float speed_err_left_acc = 0;
float speed_err_right_acc = 0;
float errAccumRight_m = 0;
float errAccumLeft_m = 0;
float prevDistLeft_m = 0;
float prevDistRight_m = 0;

 

void GetMotorMeasurements()
{
    distLeft_m = ((float)distanceLeft)/((float)162.5)/12.0*80.0/1000.0*3.14159;       
    distRight_m = ((float)distanceRight)/((float)162.5)/12.0*80.0/1000.0*3.14159;       

    measured_speedL = speedLeft/((float)162.5)/12.0*80.0/10.0*3.14159;
    measured_speedR = speedRight/((float)162.5)/12.0*80.0/10.0*3.14159;
    
    prevDistLeft_m = distLeft_m;
    prevDistRight_m = distRight_m;
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
  static uint32_t prev_print_time = 5;   // this variable is to control how often we print on the serial monitor

  cur_time = millis();                   // get the current time in miliseconds


  // turn motor on for 3 seconds 
  if(cur_time > 3000)
      motors.setSpeeds((int16_t) (0), (int16_t)(0));
  else
    motors.setSpeeds((int16_t) (300), (int16_t)(300));

  
  if(cur_time - prev_time > UPDATE_TIME_MS){
    newBalanceUpdate();                    // run the sensor updates 
    GetMotorMeasurements();
    prev_time = cur_time;
  }

  if((cur_time - prev_print_time > 20) && cur_time<3000)   // print data every 20 ms
  {
        Serial.print(measured_speedL);  // speed in cm/s
        Serial.print(",");      
        Serial.print(measured_speedR);  // speed in cm/s
        Serial.println(";");
       prev_print_time = cur_time;
  }

 
}
