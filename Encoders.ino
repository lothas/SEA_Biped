#include "MyVector.h"

// Motor encoder definitions
#define    ENC1_A       7
// Pin 3 connects to interrupt  0
#define    ENC1_B       6
#define    ENC1_CPR    2249.0  // counts per revolution
#define    ENC1_CPD    6.0  // counts per degree 2.515

// Leg encoder definitions
#define    ENC2        A1
#define    ENC2_CPD    18.26

// Motor angle variable
volatile int Enc1_count = 0;
volatile float M1_angle = 0;
volatile float M1_angle_prev = 0;
volatile float M1_angle_delta = 0;
volatile unsigned long T_cur = 0;
volatile unsigned long T_prev = 0;

// Output angle variables
volatile unsigned int Out_angle_prev1 = 0;
volatile unsigned int Out_angle_prev2 = 0;
volatile float Out_angle = 0;
volatile float Out_angle_delta = 0;

// Encoder setup
void encoder_setup() {
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  digitalWrite(ENC1_A, HIGH);                      // turn on pullup resistor
  digitalWrite(ENC1_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), Enc1_Read, CHANGE);
  
  pinMode(ENC2, INPUT);
  digitalWrite(ENC2, HIGH);

  Out_angle_vec.fill_with(analogRead(ENC2));
  Out_angle_prev1 = analogRead(ENC2);
  Out_angle_prev2 = Out_angle_prev1;
  
  T_cur = micros();
}

void Enc1_Read() {
  // ISR called when ENC1_A changes
  int A_val = digitalRead(ENC1_A);
  int B_val = digitalRead(ENC1_B);

  if ((A_val == LOW && B_val == HIGH) || (A_val == HIGH && B_val == LOW)) Enc1_count++;
  if ((A_val == HIGH && B_val == HIGH) || (A_val == LOW && B_val == LOW)) Enc1_count--;
}

float Enc2_Read() {
  // Get current reading
  unsigned int Reading = analogRead(ENC2);
  Out_angle_vec.add_element(Reading);

  // Use simpler filter
//  Out_angle = (Reading + Out_angle_prev1 + Out_angle_prev2) / 3.0;
  Out_angle = Out_angle_vec.get_avg();

  // Update previous reads
  Out_angle_prev2 = Out_angle_prev1;
  Out_angle_prev1 = Reading;
}

float enc_update() {
  // Keep Enc1_count safe from overflow
  if (Enc1_count>4000) {
    M1_angle_delta += 4000.0/ENC1_CPD;
    Enc1_count -= 4000;
  }
  if (Enc1_count<-4000) {
    M1_angle_delta -= 4000.0/ENC1_CPD;
    Enc1_count += 4000;
  }
  
  // Save previous angle
  M1_angle_prev = M1_angle;
  // Get new angle
  M1_angle = Enc1_count/ENC1_CPD+M1_angle_delta;

  // Save previous time
  T_prev = T_cur;
  // Get new time
  T_cur = micros();
  
  Enc2_Read(); // also update the reading for encoder 2
}
