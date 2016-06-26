#include "MyVector.h"

// Motor encoder definitions
#define    ENC1_A       2
#define    ENC1_B       3
#define    ENC1_CPR    2249.0  // counts per revolution
#define    ENC1_CPD    6.25    // counts per degree

// Output encoder definitions
#define    ENC2        A0
#define    ENC2_CPD    1   // counts per degree

//Encoder 1: -152.96
//Encoder 2: -978.40


// Motor angle variable
volatile int enc1_count = 0;
volatile float m1_angle = 0;
volatile float m1_angle_prev = 0;
volatile unsigned long t_cur = 0;
volatile unsigned long t_prev = 0;

// Output angle variables
MyVector out_angle_vec(5);
volatile unsigned int prev_reading = 0;
const unsigned int min_reading = 53;
const unsigned int max_reading = 979;
const int min_max_diff = int(max_reading-min_reading);
volatile int enc2_revs = 0;
volatile int enc2_jump_flag = 0;
volatile float out_angle = 0;

// Encoder setup
void setup_encoders() {
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  digitalWrite(ENC1_A, HIGH);                      // turn on pullup resistor
  digitalWrite(ENC1_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), read_enc1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), read_enc1_B, CHANGE);
  
  pinMode(ENC2, INPUT);
//  digitalWrite(ENC2, HIGH);
  
  t_cur = micros();

  delay(200);
  prev_reading = analogRead(ENC2);
  out_angle_vec.fill_with(0);
}

void read_enc1_A() {
  // ISR called when ENC1_A or ENC1_B changes
  int A_val = digitalRead(ENC1_A);
  int B_val = digitalRead(ENC1_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) enc1_count++;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) enc1_count--;
}

void read_enc1_B() {
  // ISR called when ENC1_A or ENC1_B changes
  int A_val = digitalRead(ENC1_A);
  int B_val = digitalRead(ENC1_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) enc1_count--;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) enc1_count++;
}

float read_enc2() {
  unsigned int reading = analogRead(ENC2);
  int diff = uns_int_diff(reading, prev_reading);
  prev_reading = reading;

  out_angle_vec.push(out_angle_vec.get_by_id(-1)+diff);
  // Use simpler filter
  out_angle = -out_angle_vec.get_avg()/ENC2_CPD;
}

//float read_enc2() {
//  // Get current reading
//  unsigned int reading = analogRead(ENC2);
//  int reading_diff = int(reading)-int(prev_reading);
//  // Add/Remove a full-revolution when the reading jumps
//  if (reading_diff > 0.1*min_max_diff) {
//    if (enc2_jump_flag == 0) enc2_revs--;
//    enc2_jump_flag = -4;
//  }
//  else {
//    if (reading_diff < -0.1*min_max_diff) {
//      if (enc2_jump_flag == 0) enc2_revs++;
//      enc2_jump_flag = 4;
//    }
//    else enc2_jump_flag /= 2;
//  }
//
//  // Avoid "double" jumps
////  int limit = 10;
////  float new_val = 0;
////  while (limit--) {
////    new_val = enc2_revs*min_max_diff + int(reading) - int(init_reading);
////    float val_diff = new_val - out_angle_vec.get_by_id(-1);
//////    Serial.println("new_val: " + String(new_val) + " | prev_val: " + String(out_angle_vec.get_by_id(-1)));
////    if (val_diff > 0.7*min_max_diff) enc2_revs--;
////    else {
////      if (val_diff < -0.7*min_max_diff) enc2_revs++;
////      else break;
////    }
////  }
//
//  float new_val = 0;
//  if (enc2_jump_flag>0) new_val = enc2_revs*min_max_diff;
//  if (enc2_jump_flag<0) new_val = (enc2_revs+1)*min_max_diff;
//  if (enc2_jump_flag==0) new_val = enc2_revs*min_max_diff + int(reading) - int(init_reading);
//  
//  out_angle_vec.push(new_val);
//  prev_reading = reading;
//
//  // Use simpler filter
//  out_angle = -out_angle_vec.get_avg()/ENC2_CPD;
//}

float update_encoders() {
  // Keep Enc1_count safe from overflow
  if (abs(enc1_count)>20000) {
    // This shouldn't happen
    Serial.println("Encoder 1 count close to overflow, shutting down");
    emergency_stop();
  }

  // Save previous angle
  m1_angle_prev = m1_angle;
  // Get new angle
  m1_angle = enc1_count/ENC1_CPD;

  // Save previous time
  t_prev = t_cur;
  // Get new time
  t_cur = micros();
  
  read_enc2(); // also update the reading for encoder 2
}
