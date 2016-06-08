#include "MyVector.h"

#define VERSION     "\n\n3D Printed Bio-Inspired Actuator"

#define PC_COMM_DEBUG
#define CL_DEBUG

#define    PC_COMM_SPEED   115200
#define    PC_STX          's'
#define    PC_ETX          'e'

// Closed loop definitions
#define    OUT_P         0.2    // Proportional gain for closing the output angle error
#define    OUT_D        -2.0    // Derivative gain for closing the output angle error (-2)
#define    OUT_HOME    610.0    // Output angle where torque = 0
#define    OUT_DEAD      7.0    // Dead zone for output angle
#define    MAX_DELTA    22.0    // Maximum angle error to apply

// Random weights
#define    MAX_RANDOM    220
#define    MIN_RANDOM     30

//Setting    Divisor    Frequency
//0x01           1        31250
//0x02           8        3906.25
//0x03          64        488.28125
//0x04         256        122.0703125
//0x05        1024        30.517578125

// SEA variables
volatile float M1_des_angle = 0; // Motor angle variable
volatile float des_Torque = 0;
MyVector Out_angle_vec(5);

// Control Modes
char pc_comm[8] = {0,0,0,0,0,0,0,0};
int comm_idx = 0;
int op_mode = 0;
int pc_input = 0;

// Pull-return variables
float pull_base = 0;
float pull_limit = -40;

volatile int pull_status = 0;
volatile int pull_counter = 0;

// double random(min, max)
unsigned long init_delay = 2000000;
int initializing = 1;

extern volatile float M1_angle;
extern volatile float M1_angle_prev;
extern volatile float M1_angle_delta;
extern volatile unsigned long T_cur;
extern volatile unsigned long T_prev;

void setup()  {
  Serial.begin(PC_COMM_SPEED);

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
  BT_setup();

  motor_setup();
  
  encoder_setup();
}

void loop() {
  if (initializing) {
    if (T_cur-init_delay<0) {
      op_mode = 1;
      des_Torque = 80;
    }
    else {
      op_mode = 1;
      des_Torque = 80;

      pull_base = M1_angle;
      pull_limit -= M1_angle;
      
      initializing = 0;
    }
  }
  
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
        get_pc_command(pc_comm);
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
    // Check pull status
    if (pull_status == 0) {
      // Waiting for pull
      if (M1_angle-M1_angle_prev<0) { // M1 angle decreases while pulling
        pull_counter++;
      }
      else {
        pull_counter--;
      }
      if (pull_counter>10) {
        pull_status = 1; // going up
        pull_counter = 0;
        Serial.println("Going up");
      }
    }
    if (pull_status == 1) { // going up
      if (M1_angle<pull_limit) { // M1 angle decreases while pulling
        if (M1_angle-M1_angle_prev>0) { // M1 angle increases while returning
          pull_counter++;
        }
        else {
          pull_counter--;
        }
        if (pull_counter>10) {
          pull_status = -1; // going down
          pull_counter = 0;
          des_Torque += 0.5*(0.8*MAX_RANDOM-des_Torque);
          Serial.println("Going down");
        }
      }
    }
    if (pull_status == -1) { // going down
      if (M1_angle>pull_base-5) { // M1 angle increases while returning
        pull_counter++;
      }
      else {
        pull_counter--;
      }
      if (pull_counter>10) {
        pull_status = 0; // stopped
        pull_counter = 0;
        des_Torque = random(MIN_RANDOM,MAX_RANDOM);
        Serial.print("New weight: ");
        Serial.println(des_Torque);
      }
    }
    if (pull_counter<0) pull_counter = 0;
    
    float Out_angle = Out_angle_vec.get_avg();
    float P_comp = 0;
    float D_comp = 0;
    if (Out_angle > OUT_HOME + des_Torque + OUT_DEAD) {
      P_comp = -OUT_P*(Out_angle - OUT_HOME - des_Torque - OUT_DEAD);
      D_comp = OUT_D*Out_angle_vec.get_avg_diff();
      
      M1_des_angle = M1_angle + P_comp + D_comp;
    }
    else {
      if (Out_angle < OUT_HOME + des_Torque - OUT_DEAD) {
        P_comp = -OUT_P*(Out_angle - OUT_HOME - des_Torque + OUT_DEAD);
        D_comp = OUT_D*Out_angle_vec.get_avg_diff();
        
        M1_des_angle = M1_angle + P_comp + D_comp;
      }
      else {
        M1_des_angle = M1_angle;
      }
    }

    if (M1_des_angle > M1_angle + MAX_DELTA) M1_des_angle = M1_angle + MAX_DELTA;
    if (M1_des_angle < M1_angle - MAX_DELTA) M1_des_angle = M1_angle - MAX_DELTA;

    M1_PID();
  }
  else {
    SetMotorSpeed(0);
  }
#endif
  
  //sendBlueToothData(); 
}

int UnsIntDiff(unsigned int A, unsigned int B) {
  int diff = A - B;
  if (diff<-512) diff+=1024;
  if (diff>512) diff-=1024;
  return diff;
}

void get_pc_command(char cmd[8]) {
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
    SetMotorSpeed(0);
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
      des_Torque = pc_input-200;
    }
  }
}

