/* This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include <Servo.h>

//User configuration:
int percent = 0;  //between -100 and 100, indicates how fast the motor 
      //will be moving when the arduino boots
int pins[] = {11, 9}; //the signal output pins (as many as you'd like)


const int arraySize = sizeof(pins)/sizeof(int);
Servo controllers[arraySize];

void setup() {
  Serial.begin(9600);
  Serial.println("you called, master?\n");
  for (int i=0; i<arraySize; i++)
    controllers[i].attach(pins[i]); //associate the object to a pin
  delay(1000);
  Serial.println("type in a percent, and I will output your PWM.\n");
  
}


void loop() {
  int PWMvalue = percent * 5 + 1500; //scale up to 1000-2000

  for (int i=0; i<arraySize; i++)
    controllers[i].writeMicroseconds(PWMvalue);

  if (Serial.available() > 1) {
    long proposedValue = Serial.parseInt();
    if (proposedValue == 0) {
      Serial.println("ah... stop!  yes, right on it.");
      percent = 0;
    } else if (proposedValue >= -100 && proposedValue <= 100) {
      percent = proposedValue;
      Serial.print("of course. value set to ");
      Serial.print(percent);
      Serial.println("%");
    } else {
      Serial.println("oh dear. that won't do.");
    }
  }
}