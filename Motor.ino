// Motor controller definitions
#define    INA          8
#define    INB         12
#define    M1_PWM       6

// Closed loop definitions
#define    IN_P           0.01   // Inner loop proportional gain for closing the motor angle error (0.05)
#define    IN_D        0000.0    // Inner loop derivative gain for closing the motor angle error (1000)

// Motor setup
void motor_setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  
  // PWM frequency
//  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR1B = _BV(CS00); // change the PWM frequency to 31.25kHz   - pins 9 & 10 
}

void SetMotorSpeed(float cycle) {
  // Turn the motor FWD, BWD or off
  if (cycle == 0) {
    digitalWrite(INA,LOW);
    digitalWrite(INB,LOW);
  }
  if (cycle > 0) {
    digitalWrite(INA,HIGH);
    digitalWrite(INB,LOW);
  }
  if (cycle < 0) {
    cycle = -cycle;
    digitalWrite(INA,LOW);
    digitalWrite(INB,HIGH);
  }
  float PWMval = 0;
  if (cycle<0.04) {
    // Deadzone
    PWMval = 0;
  }
  else {
    if (cycle<0.25) PWMval=500*cycle;
    else PWMval=125+150*(cycle-0.25); // Max val = 237
  }
  analogWrite(M1_PWM,PWMval);
}


void M1_PID() {
  enc_update();
  
  float error = M1_des_angle - M1_angle;
  float er_dt = (M1_angle-M1_angle_prev)/float(T_cur-T_prev);
  
  float U = -IN_P*error;;
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

  float U_deriv = IN_D*er_dt;
//  Serial.print("U = ");
//  Serial.println(U);
//  Serial.print("U_deriv = ");
//  Serial.println(U_deriv);
  if (abs(U_deriv)>0.5) U_deriv*=0.5/abs(U_deriv); // Limit U_deriv to +/-0.5
  U += U_deriv;
  
  if (U>1) U = 1;
  if (U<-1) U = -1;
  
  SetMotorSpeed(U);
}
