/*
 * Dual Motor RC code
 * 
 * Uses hardwear interrupts to read pwm signals from rc receiver
 * PWM signal is mapped to appropiate values in microseconds
 * Mapped values are written to motor controllers with arduino Servo library
 */

#include <Servo.h>

int outputValue1 = 0;
int outputValue2 = 0;

Servo controller1;
Servo controller2;

//assume that pin 2 is receiving PWM input
#define CHANNEL_1_PIN 2

//assume that pin 3 is receiving PWM input
#define CHANNEL_2_PIN 3

//micros when the pin goes HIGH
volatile unsigned long timer_start1;
int pulse_time1;
//difference between timer_start1 and micros() is the length of time that the pin 
//was HIGH - the PWM pulse length. volatile int pulse_time1; 
//this is the time that the last interrupt occurred. 
//you can use this to determine if your receiver has a signal or not. 
volatile int last_interrupt_time1; //calcSignal1 is the interrupt handler

//micros when the pin goes HIGH
volatile unsigned long timer_start2;
int pulse_time2;
//difference between timer_start2 and micros() is the length of time that the pin 
//was HIGH - the PWM pulse length. volatile int pulse_time2; 
//this is the time that the last interrupt occurred. 
//you can use this to determine if your receiver has a signal or not. 
volatile int last_interrupt_time2; //calcSignal2 is the interrupt handler 
 
void calcSignal1() 
{
    //record the interrupt time so that we can tell if the receiver has a signal from the transmitter 
    last_interrupt_time1 = micros(); 
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(CHANNEL_1_PIN) == HIGH) 
    { 
        timer_start1 = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(timer_start1 != 0)
        { 
            //record the pulse time
            pulse_time1 = ((volatile int)micros() - timer_start1);
            //restart the timer
            timer_start1 = 0;
        }
    } 
}

void calcSignal2() 
{
    //record the interrupt time so that we can tell if the receiver has a signal from the transmitter 
    last_interrupt_time2 = micros(); 
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(CHANNEL_2_PIN) == HIGH) 
    { 
        timer_start2 = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(timer_start2 != 0)
        { 
            //record the pulse time
            pulse_time2 = ((volatile int)micros() - timer_start2);
            //restart the timer
            timer_start2 = 0;
        }
    } 
} 
 
//this is all normal arduino stuff 
void setup() 
{
    timer_start1 = 0; 
    attachInterrupt(digitalPinToInterrupt(CHANNEL_1_PIN), calcSignal1, CHANGE);

    timer_start2 = 0; 
    attachInterrupt(digitalPinToInterrupt(CHANNEL_2_PIN), calcSignal2, CHANGE);

    // Attach motor controllers to output pins
    controller1.attach(9);
    controller2.attach(6);
    
    Serial.begin(115200);

    delay(1000);
} 
 
void loop()
{
    outputValue1 = map(pulse_time1, 1300, 1700, 1000, 2000); // map for elevon channel
    outputValue2 = map(pulse_time2, 1000, 1700, 1000, 2000); // map for throttle channel

    controller1.writeMicroseconds(outputValue1);
    controller2.writeMicroseconds(outputValue2);

    // Serial output for Channel 1 and Controller 1
    Serial.print("Pulse Time 1: ");
    Serial.print(pulse_time1);
    Serial.print("\t Output 1: ");
    Serial.print(outputValue1);

    // Serial output for Channel 2 and Controller 2
    Serial.print("\t Pulse Time 2: ");
    Serial.print(pulse_time2);
    Serial.print("\t Output 2: ");
    Serial.println(outputValue2);
    
    delay(20);
}
