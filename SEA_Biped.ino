// TODO Rea:
// Work on ways of adding the current reading to the control loop (Motor): [....TODO...]

  
#include "MyVector.h"
#include "Servo.h"

#define VERSION     "\n\n3D Printed Bio-Inspired Actuator"

#define PC_COMM_DEBUG
//#define MOTOR_DEBUG
//#define SERVO_DEBUG
//#define ENCODER_DEBUG
//#define INNER_LOOP_DEBUG
#define CUR_SENSE_DEBUG
#define CL_DEBUG

#define    PC_COMM_SPEED   115200
#define    PC_STX          's'
#define    PC_ETX          'e'

// Closed loop definitions
#define    OUT_P           1.5    // Proportional gain for closing the output angle error
#define    OUT_D          -2.0    // Derivative gain for closing the output angle error (-2)
#define    OUT_HOME        0.0    // Output angle where torque = 0
#define    OUT_DEAD        0.5    // Dead zone for output angle
#define    MAX_DELTA      22.0    // Maximum angle error to apply
#define    MAX_M1_ANGLE   90.0    // Maximum m1 angle (from vertical) allowed

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
extern volatile unsigned long t_cur;
extern volatile unsigned long t_prev;
extern volatile float out_angle;

float m1_cycle = 0;
float m1_cycle_delta = 0.05;

// Control Modes
char pc_comm[8] = {0,0,0,0,0,0,0,0};
int comm_idx = 0;
int op_mode = 1;
int pc_input = 0;

extern Servo rightFootServo;
extern Servo leftFootServo;

// Current sensing
float I_motor;

// Output variables
int out_count = 0;
const int out_steps = 40;

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
  check_limits();
  
  if (op_mode > 0) {
//    m1_des_angle = 30*sin(float(millis())/400.);
    if (op_mode == 2) m1_des_angle = 45.*sin(millis()/1000.);
    m1_pid();
  }
  
  if (++out_count > out_steps) {
    Serial.println(String(millis()) + " " + String(m1_angle) + " " + String(out_angle));
    out_count = 0;
  }
  
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

#ifdef SERVO_DEBUG
  if (op_mode > 0) {
    m1_cycle += 0.5*m1_cycle_delta;
    if (abs(m1_cycle)>1) m1_cycle_delta *= -1;

    if (m1_cycle>0) moveFootServo(rightFootServo,0);
    else moveFootServo(rightFootServo,180);
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
    float p_comp = 0;
    float d_comp = 0;
    if (out_angle > OUT_HOME + des_torque + OUT_DEAD) {
      p_comp = OUT_P*(out_angle - OUT_HOME - des_torque - OUT_DEAD);
//      d_comp = OUT_D*out_angle_vec.get_avg_diff();
      
      m1_des_angle = m1_angle + p_comp + d_comp;
    }
    else {
      if (out_angle < OUT_HOME + des_torque - OUT_DEAD) {
        p_comp = OUT_P*(out_angle - OUT_HOME - des_torque + OUT_DEAD);
//        d_comp = OUT_D*out_angle_vec.get_avg_diff();
        
        m1_des_angle = m1_angle + p_comp + d_comp;
      }
      else {
        m1_des_angle = m1_angle;
      }
    }

    if (m1_des_angle > m1_angle + MAX_DELTA) m1_des_angle = m1_angle + MAX_DELTA;
    if (m1_des_angle < m1_angle - MAX_DELTA) m1_des_angle = m1_angle - MAX_DELTA;
    
    if (m1_des_angle > 90) m1_des_angle = 90;
    if (m1_des_angle < -90) m1_des_angle = -90;

    m1_pid();
    moveFootServo(rightFootServo, 90+m1_des_angle);
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
  int diff = int(A) - int(B);
  if (diff<-512) diff+=1024;
  if (diff>512) diff-=1024;
  return diff;
}

void check_limits() {
  if (m1_angle > MAX_M1_ANGLE) {
    if (m1_des_angle > m1_angle) emergency_stop();
  }
  if (m1_angle < -MAX_M1_ANGLE) {
    if (m1_des_angle < m1_angle) emergency_stop();
  }
}

void read_pc_command(char cmd[8]) {
  Serial.print("Received command: ");
  pc_input = 0;

  if (cmd[0] != PC_STX) Serial.println("Transmission error");

  switch (cmd[1]) {
    case PC_ETX:
      {
      // Emergency stop command
      emergency_stop();
      }
      break;
    case 'm':
      {
      // Read motor motion command
      int dir = 0;
      if (cmd[2] == 'f') dir = 1; // move forward
      if (cmd[2] == 'b') dir = -1; // move backward
      float delta_m1 = dir*(cmd[3]-48); // angle difference
  
      if (cmd[4] == PC_ETX) {
        op_mode = 1;
        m1_des_angle += delta_m1;
        Serial.println("Move motor by "+String(delta_m1));
      }
      else Serial.println("Transmission error");
      }
      break;
    case 's':
      {
      // Received motor sine command
      if (cmd[4] == PC_ETX) {
        op_mode = 2;
        Serial.println("Start sinusoidal motion");
      }
      else Serial.println("Transmission error");
      }
      break;
    default:
      int idx = 1;
      while (true) {
        if (cmd[idx] == PC_ETX || idx>=7) {
          // transmission ended
          break;
        }
        pc_input = 10*pc_input + cmd[idx] - 48;
        Serial.print(cmd[idx++]);
      }
      
      if (idx == 4) {
        op_mode = 1;
        digitalWrite(13,HIGH);
        Serial.print("New desired speed: ");
        Serial.println(pc_input-200);
        des_torque = pc_input-200;
      }
      break;
  }
}

