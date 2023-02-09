#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <calypso_imu/buoy.h>

ros::NodeHandle  nh;

calypso_imu::buoy imu_msg;
ros::Publisher imu_pub("/calypso/calypso_imu_raw", &imu_msg);

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN) 302655
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);


void setup()
{

  if(!bno.begin())
  {
    while(true){
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000); // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(500);
    }
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  
  nh.initNode();
  nh.advertise(imu_pub);
}

void loop()
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data 
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  */
 
  // Quaternion data

  float x, y, z, w;
  imu::Quaternion quat = bno.getQuat();
  
  x = quat.x();
  y = quat.y();
  z = quat.z();
  w = quat.w();

  
  /* Display calibration status for each sensor.
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  */

  delay(BNO055_SAMPLERATE_DELAY_MS);

//  Serial.println("good!");

  imu_msg.x = x;
  imu_msg.y = y;
  imu_msg.z = z;
  imu_msg.w = w;
  
  imu_pub.publish( &imu_msg );
  nh.spinOnce();
  delay(100);
}
