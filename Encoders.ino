#include "MyVector.h"

// Motor encoder definitions
#define    ENC1_A       2
#define    ENC1_B       3
#define    ENC1_CPR    2249.0  // counts per revolution
#define    ENC1_CPD    6.25    // counts per degree

// Output encoder definitions
#define    ENC2_A       7
#define    ENC2_B       5
#define    ENC2_CPD     1   // counts per degree

//Encoder 1: -152.96
//Encoder 2: -978.40


// Motor angle variable
volatile int enc1_count = 0;
volatile float m1_angle = 0;
volatile float m1_angle_prev = 0;
volatile unsigned long t_cur = 0;
volatile unsigned long t_prev = 0;

// Output angle variables
volatile int enc2_count = 0;
volatile float out_angle = 0;
volatile float out_angle_prev = 0;

// Encoder setup
void setup_encoders() {
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  digitalWrite(ENC1_A, HIGH);                      // turn on pullup resistor
  digitalWrite(ENC1_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), read_enc1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), read_enc1_B, CHANGE);
  
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
//  digitalWrite(ENC2_A, HIGH);                      // turn on pullup resistor
//  digitalWrite(ENC2_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), read_enc2_A, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ENC2_B), read_enc2_B, CHANGE);
  
  t_cur = micros();
}

void read_enc1_A() {
  // ISR called when ENC1_A changes
  int A_val = digitalRead(ENC1_A);
  int B_val = digitalRead(ENC1_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) enc1_count++;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) enc1_count--;
}

void read_enc1_B() {
  // ISR called when ENC1_B changes
  int A_val = digitalRead(ENC1_A);
  int B_val = digitalRead(ENC1_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) enc1_count--;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) enc1_count++;
}

void read_enc2_A() {
  // ISR called when ENC2_A
  int A_val = digitalRead(ENC2_A);
  int B_val = digitalRead(ENC2_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) enc2_count++;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) enc2_count--;
}

void read_enc2_B() {
  // ISR called when ENC1_B changes
  int A_val = digitalRead(ENC2_A);
  int B_val = digitalRead(ENC2_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) enc2_count--;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) enc2_count++;
}

float update_encoders() {
  // Keep Enc1_count safe from overflow
  if (abs(enc1_count)>20000) {
    // This shouldn't happen
    Serial.println("Encoder 1 count close to overflow, shutting down");
    emergency_stop();
  }
  
  // Keep Enc2_count safe from overflow
  if (abs(enc2_count)>20000) {
    // This shouldn't happen
    Serial.println("Encoder 2 count close to overflow, shutting down");
    emergency_stop();
  }

  // Save previous angle
  m1_angle_prev = m1_angle;
  // Get new angle
  m1_angle = enc1_count/ENC1_CPD;
  
  // Save previous angle
  out_angle_prev = out_angle;
  // Get new angle
  out_angle = enc2_count/ENC2_CPD;

  // Save previous time
  t_prev = t_cur;
  // Get new time
  t_cur = micros();
}
