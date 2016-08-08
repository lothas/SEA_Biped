#define MIDDLE_FOOT_SERVO_PIN 5
#define RIGHT_FOOT_SERVO_PIN 9
#define LEFT_FOOT_SERVO_PIN 10

Servo middleFootServo;
Servo rightFootServo;
Servo leftFootServo;
  
void setup_feet_servos() {
  // TODO: check which pin the servo is connected to.
  delay(1);
  
  middleFootServo.attach(MIDDLE_FOOT_SERVO_PIN);  // attaches the servo on pin 5 to the servo object
  rightFootServo.attach(RIGHT_FOOT_SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  leftFootServo.attach(LEFT_FOOT_SERVO_PIN);
}


void moveFootServo(Servo footServo, int dist) {
  
  footServo.write(dist);                  // sets the servo position according to the scaled value
  //footServo.detach();                       // save power by detaching the servo (check if neccessary??)
}

