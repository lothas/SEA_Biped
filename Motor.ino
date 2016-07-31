#include "MyVector.h"

// Motor controller definitions
#define    INA          8
#define    INB         14
#define    M1_PWM       6
#define    CURRENT_SENSE_PIN A0   // PIN for current sensor
#define CURRENT_SENSE_SLOPE 140. // K = V_read/I_out [mV]/[A]  ==> I_out = V_read/K

// Closed loop definitions
#define    IN_P          0.06    // Inner loop proportional gain for closing the motor angle error (0.1)
#define    IN_D         900.0    // Inner loop derivative gain for closing the motor angle error (5000)

extern int error_type;

float u_P = 0;
float u_D = 0;
int stuck = 0;

// Motor setup
void setup_motor() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(CURRENT_SENSE_PIN, INPUT);
  
  // PWM frequency
//  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR1B = _BV(CS00); // change the PWM frequency to 31.25kHz   - pins 9 & 10 
}

float get_current_sense() {
  float CurrentSenseAmper = 0;
  int CurrentSense = 0;
  
  CurrentSense = analogRead(CURRENT_SENSE_PIN); // a value from 0 to 1023
  CurrentSenseAmper = ( (float(CurrentSense)*48.9) / CURRENT_SENSE_SLOPE ); // 1000 is to convert [mV] to [V]

  return CurrentSenseAmper;
}

void emergency_stop() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
  analogWrite(M1_PWM, 0);
  mode = ALL_OFF;
//  Serial.println("Emergency stop!!!");
}

void set_motor_speed(float cycle) {
  if (error_type > 0) return emergency_stop();
  
  // Turn the motor FWD, BWD or off
  if (cycle == 0) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
  }
  if (cycle > 0) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  }
  if (cycle < 0) {
    cycle = -cycle;
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  }
  float pwm_val = 0;
  if (cycle<0.04) {
    // Deadzone
    pwm_val = 0;
  }
  else {
    if (cycle<0.25) pwm_val = 400*cycle;
    else pwm_val = 100 + 100*(cycle - 0.25); // Max val = 237
  }
  analogWrite(M1_PWM, pwm_val);
}


void m1_pid() {
  update_encoders();
  
//  float error = m1_angle - m1_des_angle;
  float error = m1_angle_vec.get_avg() - m1_des_angle;
//  float er_dt = (m1_angle - m1_angle_prev)/float(t_cur - t_prev);
  float er_dt = m1_angle_vec.get_avg_diff()/float(t_cur - t_prev);
  
  u_P = -IN_P*error;;
//  if (abs(error)<=2) {
//    U = -2*IN_P*error;
//  }
//  else {
//    if (error>2) {
//      U = -4*IN_P - (error-2)*0.8*IN_P;
//    }
//    else {
//      U = 4*IN_P - (error-2)*0.8*IN_P;
//    }
//  }

  u_D = -IN_D*er_dt;
//  Serial.print("U = ");
//  Serial.println(U);
//  Serial.print("U_deriv = ");
//  Serial.println(U_deriv);
//  if (abs(u_deriv)>0.5) u_deriv *= 0.5/abs(u_deriv); // Limit U_deriv to +/-0.5
  
  float u = u_P + u_D;
  if (u>1) u = 1;
  if (u<-1) u = -1;

  if (abs(er_dt)<0.2) ++stuck;
  if (stuck>20) {
//    m1_angle_vec.fill_with(m1_des_angle);
    stuck = 0;
  }
  
  set_motor_speed(u);
}
