/* Code by Shashank Swaminathan for the ESA class project
 * This code is to measure readings from the IMU to calibrate threshold values.
 */
#include <stdint.h>
#include <LSM6.h>
#include <Balboa32U4.h>

#include <Wire.h>

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);
}

void setup() {
		Serial.begin(9600);
		balanceSetup();
}

void loop(){
    imu.read();
		Serial.print("x: ");
		Serial.print(imu.a.x);
		Serial.print("y: ");
		Serial.print(imu.a.y);
		Serial.print("z: ");
		Serial.println(imu.a.z);
		Serial.println("---");
}
