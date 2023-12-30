/*
  *Make sure arduino library rossserial is version 0.7.9, does not work with newer versions

  Allows for seamless switching of control over motor controllers between autonomous(ROS - listens to topics) control and human(RC) control
  To do so, arduino takes in input from a typical RC car/plane controller and uses the throttle, elevator and aileron
    -We are using a Turnigy controller with a Turnigy TCY-5RX reciever
  There are 3 different modes as described below

  *3 Modes
  * 1. Dead Mode - Controller must be connected with the throttle in any position EXCEPT all the way up
                 - Arduino will send no speed signals to motor controllers while in this mode
  * 2. RC Mode   - Controller must be connected with the throttle all the way up
                 - Arduino will use the right joystick (elevator - up/down, & aileron - left/right) to control steering
  * 3. ROS Mode  - Controller must be be connected and in FULL THROTTLE POSITION, then disconnected/turned off to go into ROS mode
                 - Arduino will listen for inputs from (-100 - full reverse)  
  Try uploading this to the motor arduino, don't use the other sketches.
*/

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int8.h>

//motors
Servo leftMotor;
Servo rightMotor;

//motor pins
#define leftMotorPin 9
#define rightMotorPin 10




//setting node name
ros::NodeHandle arduino;

//input data for motor speeds given by ros
int leftMotorInputROS = 0;
int rightMotorInputROS = 0;



//reads in left motor speed and verifies the value is -100 < x < 100
void readInLeft(const std_msgs::Int8 &msg) {
  //assigns data from msg to motor left motor input value
  leftMotorInputROS = msg.data;

  //Checks to make sure motor inputs are between bounds of -100 and 100
  if (leftMotorInputROS > 100)
    leftMotorInputROS = 100;
  else if(leftMotorInputROS < -100)
    leftMotorInputROS = -100;
}

//reads in right motor speed and verifies the value is -100 < x < 100
void readInRight(const std_msgs::Int8 &msg) {
  //assigns data from msg to right motor input value
  rightMotorInputROS = msg.data;

  //Checks to make sure motor inputs are between bounds of -100 and 100
  if (rightMotorInputROS > 100)
    rightMotorInputROS = 100;
  else if(rightMotorInputROS < -100)
    rightMotorInputROS = -100;
}

//creates subscriber object listening to /left_motor/speed topic and uses readInLeft as callback function
ros::Subscriber<std_msgs::Int8> subLeft("/left_motor/speed", &readInLeft);
//creates subscriber object listening to /right_motor/speed topic and uses readInRight as callback function
ros::Subscriber<std_msgs::Int8> subRight("/right_motor/speed", &readInRight);





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

//pins for the input from the RC reciever
#define throttlePin 18
#define elevatorPin 19
#define aileronPin 20

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




void setup() {
  //sets pinmodes for motor pins
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);

  //attaches the motors to defined pins
  leftMotor.attach(leftMotorPin);
  rightMotor.attach(rightMotorPin);


  //initialized a ros node on the arduino
  arduino.initNode();

  //subscribest the node to the two subscriber objects
  arduino.subscribe(subLeft);
  arduino.subscribe(subRight);


  //initializes start times for throttle, elevator, and aileron
  timerStartT = 0;
  timerStartE = 0;
  timerStartA = 0;

  //attaches the defined pins to call the respective functions anytime their value is changed
  attachInterrupt(digitalPinToInterrupt(throttlePin), calcSignalT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(elevatorPin), calcSignalE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(aileronPin), calcSignalA, CHANGE);
}





void loop() {
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








/*
  //if the throttle is all the way up and the controller is connected, the arduino will be in RC mode and listen to input from elevator and aileron inputs
  if(pulseTimeT >= 1600 && controllerConnect) {
    //Applying dead zones to both motor controllers since the RC remote being used isn't exact
    if(leftMotorOutputRC > 1460 && leftMotorOutputRC < 1540) 
      leftMotorOutputRC = 1500;
    if(rightMotorOutputRC > 1460 && rightMotorOutputRC < 1540)
      rightMotorOutputRC = 1500;
    leftMotor.writeMicroseconds(flipPolarity(leftMotorOutputRC));
    rightMotor.writeMicroseconds(rightMotorOutputRC);
  }
  //otherwise, if the throttle is not all the way up and the controller is connected, the arduino will be in dead mode, sending no movement commands
  else if (pulseTimeT < 1600 && controllerConnect){
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
  }
  //otherwise, the controller must be disconnected so the arduino should be in ROS mode, listening to topics listed above over rosserial
  else {
    leftMotor.writeMicroseconds(flipPolarity(map(leftMotorInputROS, -100, 100, 1000, 2000)));
    rightMotor.writeMicroseconds(map(rightMotorInputROS, -100, 100, 1000, 2000));
  }
  */







  
  //calls all callbacks waiting to be called
  arduino.spinOnce();

  //Debug
//  delay(10);
//  if ((leftMotorInputROS != 0 || rightMotorInputROS != 0) && (!controllerConnect || pulseTimeT <= 1600)) {
//      digitalWrite(13, HIGH);
//  }
//  delay(10);

}


//changes the direction the motor is spinning
//this is needed for our setup since the left motor's polarity needs to be reversed
int flipPolarity(int motorSpeed) {
  motorSpeed -= 1500;
  motorSpeed = -motorSpeed;
  motorSpeed += 1500;
  return motorSpeed;
}
