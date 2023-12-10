#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

//sets node name
ros::NodeHandle arduino;
 
//initialized motors as servo objects
Servo motor1;
Servo motor2;

//pins that the 2 motors will recieve pwm values from
int motor1Pin = 8;
int motor2Pin = 9;

//inputed values 
int motor1Input = 0;
int motor2Input = 0;
int motor1Output = 0;
int motor2Output = 0;


/**Reads in int array from topic motorSpeed and assigns values to motor inputs
  *Values should range from -100 for full reverse and 100 for full forward
  *
*/
void readInMsg(const std_msgs::Int16MultiArray &msgIn) {
  //assigns data from msgIn to motor input values
  motor1Input = msgIn.data[0];
  motor2Input = msgIn.data[1];

  //Checks to make sure motor inputs are between bounds of -100 and 100
  if (motor1Input > 100)
    motor1Input = 100;
  else if(motor1Input < -100)
    motor1Input = -100;
  if (motor2Input > 100)
    motor2Input = 100;
  else if(motor2Input < -100)
    motor2Input = -100;
}

//subscribes to the topic motorSpeed for message type of Int16MultiArray will a callback function of readInMsg
ros::Subscriber<std_msgs::Int16MultiArray> sub("motorSpeed", &readInMsg);

void setup() {
  //sets motor pins to output
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  //sets each motor to their respective pins
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  
  //initializes node
  arduino.initNode();
  //arduino node set to a subscriber
  arduino.subscribe(sub);
}

void loop() {

  //maps the motor input values that range from -100 to 100 to a value that the motor controllers can use, values from 1000 to 2000
  motor1Output = map(motor1Input, -100, 100, 1000, 2000);
  motor2Output = map(motor2Input, -100, 100, 1000, 2000);

  //writes pwm signals to the motors
  motor1.writeMicroseconds(motor1Output);
  motor2.writeMicroseconds(motor2Output);
  
  //calls all callbacks waiting to be called
  arduino.spinOnce();

  delay(20);

}

