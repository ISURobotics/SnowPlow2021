#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int8.h>


ros::NodeHandle arduino;

int leftMotorInputROS = 0;
int rightMotorInputROS = 0;




void readInLeft(const std_msgs::Int8 &msg) {
  //assigns data from msgIn to motor input values
  leftMotorInputROS = msg.data;

  //Checks to make sure motor inputs are between bounds of -100 and 100
  if (leftMotorInputROS > 100)
    leftMotorInputROS = 100;
  else if(leftMotorInputROS < -100)
    leftMotorInputROS = -100;
}

void readInRight(const std_msgs::Int8 &msg) {
  //assigns data from msgIn to motor input values
  rightMotorInputROS = msg.data;

  //Checks to make sure motor inputs are between bounds of -100 and 100
  if (rightMotorInputROS > 100)
    rightMotorInputROS = 100;
  else if(rightMotorInputROS < -100)
    rightMotorInputROS = -100;
}

ros::Subscriber<std_msgs::Int8> subLeft("/left_motor/speed", &readInLeft);
ros::Subscriber<std_msgs::Int8> subRight("/right_motor/speed", &readInRight);






int throttleValueCounter = 0;
int throttlePreviousValue = 0;
int throttleCurrentValue = 0;
bool controllerConnect = true;

int *pTVC = &throttleValueCounter;
int *pTPV = &throttlePreviousValue;
int *pTCV = &throttleCurrentValue;


bool controllerConnected(int *tVC, int *tPV, int *tCV) {
  if(throttlePreviousValue == throttleCurrentValue && throttleCurrentValue > 1600) {
    *tVC += 1;
  }
  else
    *tVC = 0;
  if(throttleValueCounter > 25) {
    return false;
  }
  return true;
}







int leftMotorOutputRC = 0;
int rightMotorOutputRC = 0;

Servo leftMotor;
Servo rightMotor;

#define throttlePin 18
#define elevatorPin 19
#define aileronPin 20


int elevatorOutput = 0;
int alieronOutput = 0;

#define leftMotorPin 9
#define rightMotorPin 10



volatile unsigned long timerStartT;
volatile unsigned long timerStartE;
volatile unsigned long timerStartA;

int pulseTimeT;
int pulseTimeE;
int pulseTimeA;

volatile int lastInteruptTimeT;
volatile int lastInteruptTimeE;
volatile int lastInteruptTimeA;



void calcSignalT() {
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
  // put your setup code here, to run once:
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);

  leftMotor.attach(leftMotorPin);
  rightMotor.attach(rightMotorPin);



  arduino.initNode();

  arduino.subscribe(subLeft);
  arduino.subscribe(subRight);



  timerStartT = 0;
  timerStartE = 0;
  timerStartA = 0;

  
  attachInterrupt(digitalPinToInterrupt(throttlePin), calcSignalT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(elevatorPin), calcSignalE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(aileronPin), calcSignalA, CHANGE);
}





void loop() {
  // !!!TALON MOTORCONTROLLER TAKES IN MICROSECONDS FROM 1000(FULL REVERSE) TO 2000 (FULL FORWARD)

  if(throttlePreviousValue == 0)
    throttlePreviousValue = pulseTimeT;
  else {
    throttlePreviousValue = throttleCurrentValue;
    throttleCurrentValue = pulseTimeT;
  }

  controllerConnect = controllerConnected(pTVC, pTPV, pTCV);
  

  elevatorOutput = map(pulseTimeE, 1280, 1700, 1100, 1900); 
  alieronOutput  = map(pulseTimeA, 1270, 1715, -100, 100);  

  leftMotorOutputRC = elevatorOutput + alieronOutput;
  rightMotorOutputRC = elevatorOutput - alieronOutput;




  //turns on the Arduino's LED if, throttle is all the way up
  if(pulseTimeT >= 1600 && controllerConnect) {
    leftMotor.writeMicroseconds(leftMotorOutputRC);
    rightMotor.writeMicroseconds(rightMotorOutputRC);
  }
  else if (pulseTimeT < 1600 && controllerConnect){
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
  }
  else if(!controllerConnect) {
    leftMotor.writeMicroseconds(map(leftMotorInputROS, -100, 100, 1000, 2000));
    rightMotor.writeMicroseconds(map(rightMotorInputROS, -100, 100, 1000, 2000));
  }
  

  arduino.spinOnce();
  delay(20);

}