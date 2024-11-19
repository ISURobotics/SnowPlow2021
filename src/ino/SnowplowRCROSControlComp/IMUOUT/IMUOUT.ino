
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
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

//motor pins
#define leftMotorPin 9
#define rightMotorPin 10

//pins for the input from the RC reciever
#define throttlePin 18
#define elevatorPin 19
#define aileronPin 20

#define LEDPin 13
//Motors
Servo leftMotor;
Servo rightMotor;


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










//NEW AS OF 9/23/2024***************************************************************

//Motor control values to be read from SerialComms.py.
int leftMotorInputROS = 0;
int rightMotorInputROS = 0;




//counter for how many throttle values were the same in a row, used to find if controller is disconnected
int throttleValueCounter = 0;
int throttlePreviousValue = 0;
int throttleCurrentValue = 0;
bool controllerConnect = true;

//pointers to reference above values
int *pTVC = &throttleValueCounter;
int *pTPV = &throttlePreviousValue;
int *pTCV = &throttleCurrentValue;

//tests to find out if controller is connected or not
bool controllerConnected(int *tVC, int *tPV, int *tCV) {
  //if the previous value is the same as the current value, adds one counter
  //other condition is due to the fact that throttle staying at same value for too long when at the low position on the remote
  if(throttlePreviousValue == throttleCurrentValue && throttleCurrentValue > 1600) {
    *tVC += 1;
  }
  //otherwise it resets the counter to zero
  else
    *tVC = 0;
  //if the throttle is same for 25 times in a row, return false to signify controller is disconnected
  if(throttleValueCounter > 25) {
    return false;
  }
  //otherwise return true to signify controller is connected
  return true;
}




//output values for remote control mode
int leftMotorOutputRC = 0;
int rightMotorOutputRC = 0;

//output values from RC reciever once the input values are mapped
int elevatorOutput = 0;
int alieronOutput = 0;



//start time for measuring pulse of RC reciever inputs
volatile unsigned long timerStartT;
volatile unsigned long timerStartE;
volatile unsigned long timerStartA;

//pulse time of RC reciever inputs
int pulseTimeT;
int pulseTimeE;
int pulseTimeA;

//previous time RC reciever got an input from RC remote
volatile int lastInteruptTimeT;
volatile int lastInteruptTimeE;
volatile int lastInteruptTimeA;

//calculates how long throttle pulse was
void calcSignalT() {
  //initializes current time in microseconds
  lastInteruptTimeT = micros();
  if(digitalRead(throttlePin) == HIGH)
    timerStartT = micros();
  else {
    if(timerStartT != 0) {
      pulseTimeT = ((volatile int)micros() - timerStartT);
      timerStartT = 0;
    }
  }
}

//calculates how long elevator pulse was
void calcSignalE() {
  lastInteruptTimeE = micros();
  if(digitalRead(elevatorPin) == HIGH)
    timerStartE = micros();
  else {
    if(timerStartE != 0) {
      pulseTimeE = ((volatile int)micros() - timerStartE);
      timerStartE = 0;
    }
  }
  
}

//calculates how long aileron pulse was
void calcSignalA() {
  lastInteruptTimeA = micros();
  if(digitalRead(aileronPin) == HIGH)
    timerStartA = micros();
  else {
    if(timerStartA != 0) {
      pulseTimeA = ((volatile int)micros() - timerStartA);
      timerStartA = 0;
    }
  }
  
}


//changes the direction the motor is spinning
//this is needed for our setup since the left motor's polarity needs to be reversed
int flipPolarity(int motorSpeed) {
  motorSpeed -= 1500;
  motorSpeed = -motorSpeed;
  motorSpeed += 1500;
  return motorSpeed;
}


//*********************************************************************************************** */
void setup(void)
{
  //sets pinmodes for motor pins
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(LEDPin, OUTPUT);
  //attaches the motors to defined pins
  leftMotor.attach(leftMotorPin);
  rightMotor.attach(rightMotorPin);

  //initializes start times for throttle, elevator, and aileron
  timerStartT = 0;
  timerStartE = 0;
  timerStartA = 0;

  //attaches the defined pins to call the respective functions anytime their value is changed
  attachInterrupt(digitalPinToInterrupt(throttlePin), calcSignalT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(elevatorPin), calcSignalE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(aileronPin), calcSignalA, CHANGE);


  Serial.begin(115200);
 

  while (!Serial) delay(10);  // wait for serial port to open!
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  bno.setExtCrystalUse(true);

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





    // !!!TALON MOTORCONTROLLER TAKES IN MICROSECONDS FROM 1000(FULL REVERSE) TO 2000 (FULL FORWARD)

  //on startup, set the previous throttle value to the actual value
  if(throttlePreviousValue == 0)
    throttlePreviousValue = pulseTimeT;
  //otherwise set the previous value variable to the current value, and set the current value variable to the actual value
  else {
    throttlePreviousValue = throttleCurrentValue;
    throttleCurrentValue = pulseTimeT;
  }

  //check if the controller is connected
  controllerConnect = controllerConnected(pTVC, pTPV, pTCV);
  
  //map the pulse times to the appropritate values
  /*
  Talon motor controllers take in values from 1000 to 2000, however, to account for steering left to right, the elevator will
  control the majority of the power (-60% to 60%) and the aileron will allow for small changes (-20% to 20%) which will allow for turns
  */
  elevatorOutput = map(pulseTimeE, 1280, 1700, 1200, 1800); 
  alieronOutput  = map(pulseTimeA, 1270, 1715, -100, 100);  

  //output values for the motors in RC mode, motors are controlled by both elevator and aileron as described above
  leftMotorOutputRC = elevatorOutput + alieronOutput;
  rightMotorOutputRC = elevatorOutput - alieronOutput;

   //Debug
//  if (pulseTimeT > 1600 && controllerConnect) {
//    digitalWrite(13, HIGH);
//  } else {
//    digitalWrite(13, LOW);
//  }
//  




//****leftMotorInputROS and rightMotorInputROS should be read from serial. Sent over SerialComms.py



if(Serial.available()) {


String motorInputROS = Serial.readString();
lastInput=motorInputROS;
//Serial.println("Found "+motorInputROS);
int j=0;
  int val=0;
  int neg=1;
  for(int i=0;i<=motorInputROS.length();i++){
    if(i==motorInputROS.length()||motorInputROS[i]=='|'){
      if(j==0){
        leftMotorInputROS=val*neg;
      }else if(j==1){
        rightMotorInputROS=val*neg;
      }
      j++;
      val=0;
      neg=1;
    }else{
      if(motorInputROS[i]=='-'){
        neg=-1;
      }else if(((int)(motorInputROS[i]-'0'))>=0){
        val*=10;
        val+=motorInputROS[i]-'0';
      }
    }
  }


}
Serial.print(lastInput);
if(leftMotorInputROS<=-50){
  digitalWrite(LEDPin,HIGH);
}else{
  digitalWrite(LEDPin,LOW);
}
//Serial.println(leftMotorInputROS);
//Serial.println(rightMotorInputROS);
//Serial.println("-------------");
// //read input from serial terminal 2 integers back to back for each motor
// int motor1;
// int motor2;
// while(!Serial.available()){
// motor1 = Serial.readString().toInt();
// motor2 = Serial.readString().toInt();
// }


//if the throttle is all the way up and the controller is connected, the arduino should be in ROS mode, listening to topics listed above over rosserial
  if(pulseTimeT >= 1575 && controllerConnect) {
    
    leftMotor.writeMicroseconds(flipPolarity(map(leftMotorInputROS, -100, 100, 1000, 2000)));
    rightMotor.writeMicroseconds(map(rightMotorInputROS, -100, 100, 1000, 2000));
    
  }
//If the throttle is in the middle, the arduino will be in RC mode and listen to input from elevator and aileron inputs
  else if (pulseTimeT < 1575 && pulseTimeT > 1450 && controllerConnect){
    
    //Applying dead zones to both motor controllers since the RC remote being used isn't exact
    if(leftMotorOutputRC > 1460 && leftMotorOutputRC < 1540) 
      leftMotorOutputRC = 1500;
    if(rightMotorOutputRC > 1460 && rightMotorOutputRC < 1540)
      rightMotorOutputRC = 1500;
    leftMotor.writeMicroseconds(flipPolarity(leftMotorOutputRC));
    rightMotor.writeMicroseconds(rightMotorOutputRC);
    
  }
//Otherwise, if the throttle is all the way down, the controller will be in dead mode and will not move
  else {
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
  }



}
