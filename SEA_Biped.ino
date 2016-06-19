// TODO Rea:
// Write code for reading current sensor (output Amp) : [done]
// Add code for foot servo: [done]
// Work on ways of adding the current reading to over current protection: [done]
// Work on ways of adding the current reading to the control loop (Motor): [....TODO...]

  
#include "MyVector.h"
#include "Servo.h"

#define VERSION     "\n\n3D Printed Bio-Inspired Actuator"

//#define MOTOR_DEBUG
#define ENCODER_DEBUG
#define INNER_LOOP_DEBUG
#define PC_COMM_DEBUG
#define CUR_SENSE_DEBUG
//#define CL_DEBUG

#define    PC_COMM_SPEED   115200
#define    PC_STX          's'
#define    PC_ETX          'e'

// Closed loop definitions
#define    OUT_P         0.2    // Proportional gain for closing the output angle error
#define    OUT_D        -2.0    // Derivative gain for closing the output angle error (-2)
#define    OUT_HOME    610.0    // Output angle where torque = 0
#define    OUT_DEAD      7.0    // Dead zone for output angle
#define    MAX_DELTA    22.0    // Maximum angle error to apply

#define    MAX_ALLOWED_MOTOR_CURRENT   4 

//Setting    Divisor    Frequency
//0x01           1        31250
//0x02           8        3906.25
//0x03          64        488.28125
//0x04         256        122.0703125
//0x05        1024        30.517578125

// SEA variables
volatile float m1_des_angle = 0; // Motor angle variable
volatile float des_torque = 0;

extern volatile float m1_angle;
extern volatile float m1_angle_prev;
extern volatile float m1_angle_delta;
extern volatile unsigned long t_cur;
extern volatile unsigned long t_prev;
extern volatile float out_angle;
volatile unsigned int prev_reading = 0;

float m1_cycle = 0;
float m1_cycle_delta = 0.05;

// Control Modes
char pc_comm[8] = {0,0,0,0,0,0,0,0};
int comm_idx = 0;
int op_mode = 1;
int pc_input = 0;

// Current sensing
float I_motor;

void setup()  {
  Serial.begin(PC_COMM_SPEED);
  
//  setup_bluetooth();

  setup_motor();
  
  setup_feet_servos();

  setup_encoders();
  
  pinMode(13,OUTPUT);
  for (int i = 0; i<3; ++i) {
    digitalWrite(13,HIGH);
    delay(170);
    digitalWrite(13,LOW);
    delay(170);
  }
}

void loop() {
#ifdef MOTOR_DEBUG
  if (op_mode > 0) {
    m1_cycle += m1_cycle_delta;
    if (abs(m1_cycle)>0.6) m1_cycle_delta *= -1;

    if (m1_cycle>0) set_motor_speed(0.5);
    else set_motor_speed(-0.5);
    delay(100);
  }
  else {
    set_motor_speed(0);
  }
#endif

#ifdef ENCODER_DEBUG
  update_encoders();
//  Serial.print("Encoder 1: ");
//  Serial.println(m1_angle);
//  Serial.print("Encoder 2: ");
//  Serial.println(String(millis()) + " " + String(out_angle) + " " + String(prev_reading));
  delay(10);
#endif

#ifdef INNER_LOOP_DEBUG
  if (op_mode > 0) {
    m1_cycle += 0.02*m1_cycle_delta;
    if (m1_cycle>1) {
      m1_cycle_delta *= -1;
//      delay(5000);
    }
    if (m1_cycle<0) {
      m1_cycle_delta *= -1;
//      delay(5000);
    }

    m1_des_angle = 360*m1_cycle;
    
//    Serial.print("Desired M1 angle: ");
//    Serial.println(m1_des_angle);

    m1_pid();
  }
#endif

#ifdef PC_COMM_DEBUG
  if(Serial.available()) {
    pc_comm[comm_idx] = Serial.read();
    if (comm_idx == 0) {
      if (pc_comm[comm_idx] == PC_STX) {
        // Start reading command
        comm_idx++;
      }
    }
    else {
      if (pc_comm[comm_idx] == PC_ETX || comm_idx>=7) {
        // Command received
        read_pc_command(pc_comm);
        comm_idx = 0;
      }
      else {
        comm_idx++;
      }
    }
  }
#endif

#ifdef CL_DEBUG
  if (op_mode == 1) {
    float out_angle = out_angle_vec.get_avg();
    float p_comp = 0;
    float d_comp = 0;
    if (out_angle > OUT_HOME + des_torque + OUT_DEAD) {
      p_comp = -OUT_P*(out_angle - OUT_HOME - des_torque - OUT_DEAD);
      d_comp = OUT_D*out_angle_vec.get_avg_diff();
      
      m1_des_angle = m1_angle + p_comp + d_comp;
    }
    else {
      if (out_angle < OUT_HOME + des_torque - OUT_DEAD) {
        p_comp = -OUT_P*(out_angle - OUT_HOME - des_torque + OUT_DEAD);
        d_comp = OUT_D*out_angle_vec.get_avg_diff();
        
        m1_des_angle = m1_angle + p_comp + d_comp;
      }
      else {
        m1_des_angle = m1_angle;
      }
    }

    if (m1_des_angle > m1_angle + MAX_DELTA) m1_des_angle = m1_angle + MAX_DELTA;
    if (m1_des_angle < m1_angle - MAX_DELTA) m1_des_angle = m1_angle - MAX_DELTA;

    m1_pid();
  }
  else {
    set_motor_speed(0);
  }
#endif
  
  //send_bluetooth_data(); 

#ifdef CUR_SENSE_DEBUG
  // Current sensing and emergency stop:
  I_motor = get_current_sense();
  Serial.println(String(millis()) + " " + String(I_motor));
  
//  if (I_motor > MAX_ALLOWED_MOTOR_CURRENT) {
//    emergency_stop();
//  }
#endif
}

int uns_int_diff(unsigned int A, unsigned int B) {
  int diff = A - B;
  if (diff<-512) diff+=1024;
  if (diff>512) diff-=1024;
  return diff;
}

void read_pc_command(char cmd[8]) {
  Serial.print("Received command: ");
  pc_input = 0;
  
  int idx = 1;
  while (true) {
    if (cmd[idx] == PC_ETX || idx>=7) {
      Serial.println("");
      break;
    }
    pc_input = 10*pc_input + cmd[idx] - 48;
    Serial.print(cmd[idx++]);
  }
  
  if (idx == 1) {
    set_motor_speed(0);
    op_mode = 0;
    digitalWrite(13,LOW);
    Serial.println("EMERGENCY STOP");
  }
  else {
    if (idx == 4) {
      op_mode = 1;
      digitalWrite(13,HIGH);
      Serial.print("New desired speed: ");
      Serial.println(pc_input-200);
      des_torque = pc_input-200;
    }
  }
}

