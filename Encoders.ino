#include "MyVector.h"

// Motor encoder definitions
#define    ENC1_A       2
#define    ENC1_B       3
#define    ENC1_CPD     6.25    // counts per degree (2249.0 counts per revolution)

// Output encoder definitions
#define    ENC2_A       7
#define    ENC2_B       4
#define    ENC2_CPD     15.667   // counts per degree

extern int error_type;

unsigned long t_cur = 0;
unsigned long t_prev = 0;

// Motor angle variable
volatile int enc1_count = 0;
float m1_angle = 0;
float m1_angle_diff = 0;
MyVector m1_angle_vec(10);

// Output angle variables
volatile int enc2_count = 0;
float out_angle = 0;
float out_angle_diff = 0;
MyVector out_angle_vec(10);

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

  m1_angle_vec.fill_with(0);
  out_angle_vec.fill_with(0);
  t_cur = micros();
}

void read_enc1_A() {
  // ISR called when ENC1_A changes
  int A_val = digitalRead(ENC1_A);
  int B_val = digitalRead(ENC1_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) enc1_count++;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) enc1_count--;
  if (abs(enc1_count)>600) error_type = 5, emergency_stop();
}

void read_enc1_B() {
  // ISR called when ENC1_B changes
  int A_val = digitalRead(ENC1_A);
  int B_val = digitalRead(ENC1_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) enc1_count--;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) enc1_count++;
  if (abs(enc1_count)>600) error_type = 5, emergency_stop();
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

void update_encoders() {
  // Keep Enc1_count safe from overflow
  if (abs(enc1_count)>20000) {
    // This shouldn't happen
    Serial.println("Encoder 1 count close to overflow, shutting down");
    error_type = 3;
    emergency_stop();
    enc1_count = 0;
  }
  
  // Keep Enc2_count safe from overflow
  if (abs(enc2_count)>20000) {
    // This shouldn't happen
    Serial.println("Encoder 2 count close to overflow, shutting down");
    error_type = 4;
    emergency_stop();
    enc2_count = 0;
  }

  // Add current reading and update angles
  m1_angle_vec.push(enc1_count/ENC1_CPD);
  m1_angle = m1_angle_vec.get_avg();
  m1_angle_diff = m1_angle_vec.get_avg_diff()/float(t_cur - t_prev);

  // Add current reading and update angles
  out_angle_vec.push(enc2_count/ENC2_CPD);
  out_angle = out_angle_vec.get_avg();
  out_angle_diff = out_angle_vec.get_avg_diff()/float(t_cur - t_prev);

  // Save previous time
  t_prev = t_cur;
  // Get new time
  t_cur = micros();
}

void reset_encoders() {
  enc1_count = 0;
  enc2_count = 0;
  m1_angle_vec.fill_with(0);
  out_angle_vec.fill_with(0);
  m1_angle = 0;
  out_angle = 0;
}

