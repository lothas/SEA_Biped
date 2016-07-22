#define RIGHT_FOOT_SERVO_PIN 9
#define LEFT_FOOT_SERVO_PIN 10

Servo rightFootServo;
Servo leftFootServo;
  
void setup_feet_servos() {
  // TODO: check which pin the servo is connected to.
  delay(1);
  
  rightFootServo.attach(RIGHT_FOOT_SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  leftFootServo.attach(LEFT_FOOT_SERVO_PIN);
}


void moveFootServo(Servo footServo, int dist) {
  
  footServo.write(dist);                  // sets the servo position according to the scaled value
  delay(10);                           // waits for the servo to get there
  //footServo.detach();                       // save power by detaching the servo (check if neccessary??)
}

  //TODO: 1) write a function to convert servo angle to actual foot extension ([mm])