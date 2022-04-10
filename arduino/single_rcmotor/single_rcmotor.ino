// C++ code
//
/*
  Single Motor Control Test Code
  using Servo library
*/

#include <Servo.h>
 
 //User configuration:
 int pins[] = {9, 6}; //the signal output pins (as many as you'd like)
 
 int sensorValue1 = 0;
 int outputValue1 = 0;

 int sensorValue2 = 0;
 int outputValue2 = 0;
 
 const int arraySize = sizeof(pins)/sizeof(int);
 Servo controllers[arraySize];
 
 void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  
   Serial.begin(9600);
   for (int i=0; i<arraySize; i++)
     controllers[i].attach(pins[i]); //associate the object to a pin
   delay(1000);
  
 }
 
 
 void loop() {
  sensorValue1 = analogRead(A0);
  sensorValue2 = analogRead(A1);
  
  outputValue1 = map(sensorValue1, 0, 1023, 0, 200) - 100; //convert to a "percentage"
  outputValue2 = map(sensorValue2, 0, 1023, 0, 200) - 100;

  outputValue1 = outputValue1 * 5 + 1500;  //scale up to 1000-2000
  outputValue2 = outputValue2 * 5 + 1500;

  controllers[0].writeMicroseconds(outputValue1); //output values to pins
  controllers[0].writeMicroseconds(outputValue2);

    // print the results to the serial monitor:
  Serial.print("sensor1 = ");
  Serial.print(sensorValue1);
  Serial.print("\t output1 = ");
  Serial.print(outputValue1);
  
  Serial.print("\t sensor2 = ");
  Serial.print(sensorValue2);
  Serial.print("\t output2 = ");
  Serial.println(outputValue2);
  
  // wait 2 milliseconds before the next loop for the
  // analog-to-digital converter to settle after the
  // last reading:
  delay(2); // Wait for 2 millisecond(s)
 

 }
