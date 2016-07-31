// TODO Rea:
// Work on ways of adding the current reading to the control loop (Motor): [....TODO...]

  
#include "MyVector.h"
#include "Servo.h"

#define VERSION     "\n\n3D Printed Bio-Inspired Actuator"

#define PC_COMM_DEBUG
#define BT_COMM_DEBUG
//#define MOTOR_DEBUG
//#define SERVO_DEBUG
//#define ENCODER_DEBUG
//#define INNER_LOOP_DEBUG
//#define CUR_SENSE_DEBUG
#define CL_DEBUG
#define CPG_DEBUG

#define    PC_COMM_SPEED   115200
#define    PC_STX          'z'
#define    PC_ETX          'x'

// Closed loop definitions
#define    OUT_P           1.0    // Proportional gain for closing the output angle error
#define    OUT_D          -0.0    // Derivative gain for closing the output angle error (-2)
#define    OUT_HOME        0.0    // Output angle where torque = 0
#define    OUT_DEAD        0.2    // Dead zone for output angle
#define    MAX_DELTA      22.0    // Maximum angle error to apply
#define    MAX_M1_ANGLE   90.0    // Maximum m1 angle (from vertical) allowed

#define    MAX_ALLOWED_MOTOR_CURRENT   4 

#define SERVO_OUT             10
#define SERVO_IN              160

//Setting    Divisor    Frequency
//0x01           1        31250
//0x02           8        3906.25
//0x03          64        488.28125
//0x04         256        122.0703125
//0x05        1024        30.517578125

// Operating modes
enum op_mode { ALL_OFF, INNER_LOOP, SEA_LOOP, TEST_LOOP };

// SEA variables
volatile float m1_des_angle = 0; // Motor angle variable
volatile float des_torque = 0;

extern volatile float m1_angle;
extern volatile unsigned long t_cur;
extern volatile unsigned long t_prev;
extern volatile float out_angle;

extern MyVector m1_angle_vec;
extern MyVector out_angle_vec;

float m1_cycle = 0;
float m1_cycle_delta = 0.05;

extern float u_P;
extern float u_D;

// Control Modes
char pc_comm[8] = {0,0,0,0,0,0,0,0};
int comm_idx = 0;
char bt_comm[8] = {0,0,0,0,0,0,0,0};
int bt_comm_idx = 0;
op_mode mode = SEA_LOOP;
int pc_input = 0;
int sm1_go = 1;
int sm2_go = 1;

extern Servo middleFootServo;
extern Servo rightFootServo;
extern Servo leftFootServo;

// Current sensing
float I_motor;

// Output variables
int error_type = 0;
int out_count = 0;
const int out_steps = 10;

// CPG variables
unsigned long t_reset = 0;

void setup()  {
  Serial.begin(PC_COMM_SPEED);
  
  setup_bluetooth();

  delay(10);
  setup_motor();
  
  delay(10);
  setup_feet_servos();

  delay(10);
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
  
  // Output data to PC
  if (++out_count > out_steps) {
//    Serial.println(String(millis()) + " " + String(des_torque) + " " + String(m1_angle_vec.get_avg()));
//    Serial.println(String(millis()) + " " + String(m1_angle_vec.get_avg()) + " " + String(out_angle));
//    Serial.println(String(millis()) + " " + String(u_P) + " " + String(u_D) + " " + String(u_P+u_D));
    out_count = 0;
    switch (error_type) {
      case 1:
        Serial.println("ERROR! Manual stop called from terminal.");
        break;
      case 2:
        Serial.println("ERROR! Motor angle is outside allowed bounds.");
        break;
      case 3:
        Serial.println("ERROR! Encoder 1 count close to overflow, shutting down");
        break;
      case 4:
        Serial.println("ERROR! Encoder 2 count close to overflow, shutting down");
        break;
      case 5:
        Serial.println("ERROR! Motor angle is outside allowed bounds (called from ISR).");
        break;
      case 6:
        Serial.println("ERROR! Wrong arguments passed to SM function.");
        break;
    }
  }
  
#ifdef MOTOR_DEBUG
  if (mode != ALL_OFF) {
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
  if (mode != ALL_OFF) {
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
//  Serial.println(String(millis()) + " " + String(m1_angle) + " " + String(out_angle));
//  delay(10);
#endif

#ifdef INNER_LOOP_DEBUG
  if (mode == INNER_LOOP) {
    m1_cycle += 0.03*m1_cycle_delta;
    if (m1_cycle>1) {
      m1_cycle_delta *= -1;
//      delay(5000);
    }
    if (m1_cycle<0) {
      m1_cycle_delta *= -1;
//      delay(5000);
    }

    m1_des_angle = -80+160*m1_cycle;
    
//    Serial.print("Desired M1 angle: ");
//    Serial.println(m1_des_angle);

    m1_pid();
  }
  if (mode == TEST_LOOP) {
    m1_des_angle = 45.*sin(millis()/1000.);
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

#ifdef BT_COMM_DEBUG
  if(Serial1.available()) {
    bt_comm[bt_comm_idx] = Serial1.read();
    if (bt_comm_idx == 0) {
      if (bt_comm[bt_comm_idx] == PC_STX) {
        // Start reading command
        bt_comm_idx++;
      }
    }
    else {
      if (bt_comm[bt_comm_idx] == PC_ETX || bt_comm_idx>=7) {
        // Command received
        read_pc_command(bt_comm);
        bt_comm_idx = 0;
      }
      else {
        bt_comm_idx++;
      }
    }
  }
#endif

#ifdef CPG_DEBUG
//  unsigned long phase = micros() - t_reset;
//  unsigned long period = 2000000;
//  unsigned long start1 = 100000;
//  unsigned long end1 = 300000;
//  float amp1 = 10;
//  unsigned long start2 = 1100000;
//  unsigned long end2 = 1300000;
//  float amp2 = -0;
//  if (phase > period) {
//    t_reset = t_reset+period;
//    Serial.println("CPG reset");
//  }
//  des_torque = 0;
//  if (mode != ALL_OFF) {
//    if (phase > start1 && phase < end1) des_torque += amp1, moveFootServo(middleFootServo, 10);
//    if (phase > start2 && phase < end2) des_torque += amp2, moveFootServo(middleFootServo, 120);
//  }
  unsigned long t_stamp = micros();
  if (sm1_go) {
  //  sm1_condition(t_stamp);
  //  sm1_action();
    des_torque = 10.*sin(millis()/250.);
  }
  if (sm2_go) {
    sm2_condition(t_stamp, m1_angle_vec.get_avg()+out_angle_vec.get_avg());
  //  sm2_action();
  }
#endif

#ifdef CL_DEBUG
  if (mode == SEA_LOOP) {
    float p_comp = 0;
    float d_comp = 0;
    out_angle = out_angle_vec.get_avg();
    if (out_angle > OUT_HOME + des_torque + OUT_DEAD) {
      p_comp = OUT_P*(out_angle - OUT_HOME - des_torque - OUT_DEAD);
      d_comp = OUT_D*out_angle_vec.get_avg_diff();
    }
    else {
      if (out_angle < OUT_HOME + des_torque - OUT_DEAD) {
        p_comp = OUT_P*(out_angle - OUT_HOME - des_torque + OUT_DEAD);
        d_comp = OUT_D*out_angle_vec.get_avg_diff();
      }
    }
    m1_des_angle = m1_angle + p_comp + d_comp;

    if (m1_des_angle > m1_angle + MAX_DELTA) m1_des_angle = m1_angle + MAX_DELTA;
    if (m1_des_angle < m1_angle - MAX_DELTA) m1_des_angle = m1_angle - MAX_DELTA;
    
    if (m1_des_angle > 90) m1_des_angle = 90;
    if (m1_des_angle < -90) m1_des_angle = -90;

    m1_pid();
//    moveFootServo(rightFootServo, 90+m1_des_angle);
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
    if (m1_des_angle > m1_angle) {
      error_type = 2;
      emergency_stop();
    }
  }
  if (m1_angle < -MAX_M1_ANGLE) {
    if (m1_des_angle < m1_angle) {
      error_type = 2;
      emergency_stop();
    }
  }
}

void read_pc_command(char cmd[8]) {
  Serial.print("Received command: ");
  pc_input = 0;

  if (cmd[0] != PC_STX) Serial.println("Transmission error");

  switch (cmd[1]) {
    // ----------------- GLOBAL CONTROL COMMANDS -----------------
    case PC_ETX: // ///// Emergency stop command /////////////////
      {
      error_type = 1;
      emergency_stop();
      }
      break;
    case 'e': // ///////// Clear error command //////////////////
      {
      if (cmd[2] == PC_ETX) {
        mode = SEA_LOOP;
        error_type = 0;
        Serial.println("Error cleared");
      }
      else Serial.println("Transmission error");
      }
      break;
    case 'r': // //////// Clear angles command ///////////////////
      {
      if (cmd[2] == PC_ETX) {
        mode = SEA_LOOP;
        reset_encoders();
        Serial.println("Angles cleared");
      }
      else Serial.println("Transmission error");
      }
      break;
    case '9': // //////////// Rest command //////////////////////
      {
      if (cmd[2] == PC_ETX) {
        Serial.println("Go to rest position");
        // Stop state machines
        sm1_go = 0;
        sm2_go = 0;

        // Spread legs
        mode = INNER_LOOP;
        m1_des_angle = 15;

        // Extend feet
        moveFootServo(rightFootServo, SERVO_OUT);
        moveFootServo(leftFootServo, SERVO_OUT);
        moveFootServo(middleFootServo, SERVO_OUT);
      }
      else Serial.println("Transmission error");
      }
      break;
      
    // ---------------- STATE MACHINE COMMANDS ------------------
    case '1': // /////// Toggle SM1 command /////////////////////
      {
      if (cmd[2] == PC_ETX) {
        sm1_go = !sm1_go;
        Serial.println(sm1_go ? "Turned State Machine 1 ON" : "Turned State Machine 1 OFF");
      }
      else Serial.println("Transmission error");
      }
      break;
    case '2': // /////// Toggle SM2 command /////////////////////
      {
      if (cmd[2] == PC_ETX) {
        sm2_go = !sm2_go;
        Serial.println(sm2_go ? "Turned State Machine 2 ON" : "Turned State Machine 2 OFF");
      }
      else Serial.println("Transmission error");
      }
      break;
      
    // --------------------- MOTOR COMMANDS ---------------------
    case 'm': // ////// Motor set-point command /////////////////
      {
      int dir = 0;
      if (cmd[2] == 'f') dir = 1; // move forward
      if (cmd[2] == 'b') dir = -1; // move backward
      float delta_m1 = dir*(cmd[3]-48); // angle difference
  
      if (cmd[4] == PC_ETX) {
        mode = INNER_LOOP;
        m1_des_angle += delta_m1;
        Serial.println("Move motor by "+String(delta_m1));
      }
      else Serial.println("Transmission error");
      }
      break;
    case 's': // ////// Sine trajectory command /////////////////
      {
      if (cmd[2] == PC_ETX) {
        mode = TEST_LOOP;
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
        mode = SEA_LOOP;
        digitalWrite(13,HIGH);
        Serial.print("New desired speed: ");
        Serial.println(pc_input-200);
        des_torque = pc_input-200;
      }
      break;
  }
}

