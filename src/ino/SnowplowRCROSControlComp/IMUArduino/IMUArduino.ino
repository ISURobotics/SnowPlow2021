#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <string.h>
#include <HardwareSerial.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)




// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int compass_calibration = 0;


int getCalibrationStatus(void)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  return mag;
}












//*********************************************************************************************** */
void setup(void)
{
  Serial.begin(115200);
  Serial.setTimeout(50);
 

  while (!Serial) delay(10);  // wait for serial port to open!
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  bno.setExtCrystalUse(true);
  //Continnuosly spits "IAMIMU" to the serial for detection. Once it recieves a good to go back it stops and continnues on with life
  while(1){
    if(Serial.available()){
      if(Serial.read()=='A')break;
    }
    Serial.print("IAMIMU");
  }
}
String lastInput="";
void loop(void)
{
  //Serial.println("Here");
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  //Euler angle of Rotation 
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  compass_calibration = getCalibrationStatus();


  //Write to Serial Terminal Format so python can parse values back
  double magX = mag.x();
  double magY = mag.y();
  double magZ = mag.z();
  double yaw = atan2(magY, -magZ) * 180/3.14159;


  double eulx = euler.x();
  double euly = euler.y();
  double eulz = euler.z();

  Serial.print(eulx);
  Serial.print(',');
  Serial.print(euly);
  Serial.print(',');
  Serial.print(eulz);
  Serial.print(';');

  if (compass_calibration == 3) {
    Serial.print(0.0);
    Serial.print(',');
    Serial.print(0.0);
    Serial.print(',');
    Serial.println(yaw);
  }
  else {
    Serial.print(1.0);
    Serial.print(',');
    Serial.print(1.0);
    Serial.print(',');
    Serial.println(0.0);
  }

  if (eulx == 0.0 && euly == 0.0 && eulz == 0.0){
    bno.setExtCrystalUse(true);
  }

  

  delay(BNO055_SAMPLERATE_DELAY_MS); //*****************************FIX LATER************* */
}