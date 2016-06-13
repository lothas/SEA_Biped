#include "MyVector.h"

// Motor encoder definitions
#define    ENC1_A       2
#define    ENC1_B       3
#define    ENC1_CPR    2249.0  // counts per revolution
#define    ENC1_CPD    6.25    // counts per degree

// Output encoder definitions
#define    ENC2        A1
#define    ENC2_CPD    18.26   // counts per degree

// Motor angle variable
volatile int enc1_count = 0;
volatile float m1_angle = 0;
volatile float m1_angle_prev = 0;
volatile float m1_angle_delta = 0;
volatile unsigned long t_cur = 0;
volatile unsigned long t_prev = 0;

// Output angle variables
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
  digitalWrite(ENC2, HIGH);

  out_angle_vec.fill_with(analogRead(ENC2));
  
  t_cur = micros();
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
  // Get current reading
  unsigned int reading = analogRead(ENC2);
  out_angle_vec.push(reading);

  // Use simpler filter
  out_angle = out_angle_vec.get_avg();
}

float update_encoders() {
  // Keep Enc1_count safe from overflow
  if (enc1_count>4000) {
    m1_angle_delta += 4000.0/ENC1_CPD;
    enc1_count -= 4000;
  }
  if (enc1_count<-4000) {
    m1_angle_delta -= 4000.0/ENC1_CPD;
    enc1_count += 4000;
  }
  
  // Save previous angle
  m1_angle_prev = m1_angle;
  // Get new angle
  m1_angle = enc1_count/ENC1_CPD + m1_angle_delta;

  // Save previous time
  t_prev = t_cur;
  // Get new time
  t_cur = micros();
  
  read_enc2(); // also update the reading for encoder 2
}
