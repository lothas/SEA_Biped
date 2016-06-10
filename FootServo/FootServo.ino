
#include <Servo.h>

Servo footServo;  // create servo object to control a servo

int servoValIn = 0;

void setup() {
  // TODO: check which pin the servo is connected to.
  footServo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  footServo.write(val);                  // sets the servo position according to the scaled value
  delay(50);                           // waits for the servo to get there
  servo.detach()                       // detach the servo (check if neccessary??)
}


  //val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  //val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)

  //TODO: 1) write a function to convert servo angle to actual foot extension ([mm])
  //      2) make this code a function to be used in the main loop.
