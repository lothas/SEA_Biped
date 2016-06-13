// Motor controller definitions
#define    INA          8
#define    INB         12
#define    M1_PWM       6

// Closed loop definitions
#define    IN_P           0.1    // Inner loop proportional gain for closing the motor angle error (0.05)
#define    IN_D        5000.0    // Inner loop derivative gain for closing the motor angle error (1000)

// Motor setup
void setup_motor() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  
  // PWM frequency
//  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR1B = _BV(CS00); // change the PWM frequency to 31.25kHz   - pins 9 & 10 
}

void set_motor_speed(float cycle) {
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
    if (cycle<0.25) pwm_val = 500*cycle;
    else pwm_val = 125 + 150*(cycle - 0.25); // Max val = 237
  }
  analogWrite(M1_PWM, pwm_val);
}


void m1_pid() {
  update_encoders();
  
  float error = m1_angle - m1_des_angle;
  float er_dt = (m1_angle - m1_angle_prev)/float(t_cur - t_prev);
  
  float u = -IN_P*error;;
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

  float u_deriv = -IN_D*er_dt;
//  Serial.print("U = ");
//  Serial.println(U);
//  Serial.print("U_deriv = ");
//  Serial.println(U_deriv);
  if (abs(u_deriv)>0.5) u_deriv *= 0.5/abs(u_deriv); // Limit U_deriv to +/-0.5
  u += u_deriv;
  
  if (u>1) u = 1;
  if (u<-1) u = -1;
  
  set_motor_speed(u);
}
