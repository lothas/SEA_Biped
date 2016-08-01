// TODO Rea:
// Work on ways of adding the current reading to the control loop (Motor): [....TODO...]

#include "MyVector.h"
#include "Servo.h"

#define VERSION     "\n\n3D Printed Bio-Inspired Actuator"

// Closed loop definitions
#define    OUT_P           1.0    // Proportional gain for closing the output angle error
#define    OUT_D          -0.0    // Derivative gain for closing the output angle error (-2)
#define    OUT_DEAD        0.2    // Dead zone for output angle
#define    MAX_DELTA      22.0    // Maximum angle error to apply
#define    MAX_M1_ANGLE   90.0    // Maximum m1 angle (from vertical) allowed

#define    MAX_ALLOWED_MOTOR_CURRENT   4 

#define    SERVO_OUT        10
#define    SERVO_IN        160

//Setting    Divisor    Frequency
//0x01           1        31250
//0x02           8        3906.25
//0x03          64        488.28125
//0x04         256        122.0703125
//0x05        1024        30.517578125

// Operating modes
enum op_mode { ALL_OFF, PID_MODE, SEA_MODE, SINE_MODE };
op_mode mode = SEA_MODE;
int sm1_go = 1;
int sm2_go = 1;

// CPG variables
unsigned long t_reset = 0;

// Motor control variables
float m1_des_angle = 0; // Motor angle variable
float des_torque = 0;

float m1_cycle = 0;
float m1_cycle_delta = 0.002;

extern float u_P;
extern float u_D;

extern Servo middleFootServo;
extern Servo rightFootServo;
extern Servo leftFootServo;

// Current sensing
float I_motor;

// Encoder variables
extern MyVector m1_angle_vec;
extern float m1_angle;
extern float m1_angle_diff;
extern MyVector out_angle_vec;
extern float out_angle;
extern float out_angle_diff;

// Output variables
int error_type = 0;
int out_count = 0;
const int out_steps = 10;

void setup()  {
  setup_comms();

  delay(10);
  setup_motor();
  
  delay(10);
  setup_feet_servos();

  delay(10);
  setup_encoders();

  // Blink LED when done
  pinMode(17,OUTPUT);  // RXLED on arduino pro micro
  for (int i = 0; i<3; ++i) {
    digitalWrite(17,HIGH);
    TXLED1; //TX LED is not tied to a normally controlled pin
    delay(170);
    digitalWrite(17,LOW);
    TXLED0; //TX LED is not tied to a normally controlled pin
    delay(170);
  }
}

void loop() {
  // Update angles from encoders
  update_encoders();

  // Check limits: angle limits, current limits, etc.
  check_limits();

  // Read serial commands
  check_pc_comm();
  check_bt_comm();
  
  // Step through state machines
  unsigned long t_stamp = micros();
  if (sm1_go) {
  //  sm1_condition(t_stamp);
  //  sm1_action();
    des_torque = 10.*sin(millis()/250.);
  }
  if (sm2_go) {
    sm2_condition(t_stamp, m1_angle+out_angle);
  //  sm2_action();
  }

  // Control modes
  if (mode == SEA_MODE) {
    // Set desired m1 angle based on current out_angle and desired torque
    float p_comp = 0;
    float d_comp = 0;
    if (out_angle > des_torque + OUT_DEAD) {
      p_comp = OUT_P*(out_angle - des_torque - OUT_DEAD);
      d_comp = OUT_D*out_angle_diff;
    }
    else {
      if (out_angle < des_torque - OUT_DEAD) {
        p_comp = OUT_P*(out_angle - des_torque + OUT_DEAD);
        d_comp = OUT_D*out_angle_vec.get_avg_diff();
      }
    }
    m1_des_angle = m1_angle + p_comp + d_comp;

    // Limit m1 delta range
    if (m1_des_angle > m1_angle + MAX_DELTA) m1_des_angle = m1_angle + MAX_DELTA;
    if (m1_des_angle < m1_angle - MAX_DELTA) m1_des_angle = m1_angle - MAX_DELTA;

    // Limit angle range
    if (m1_des_angle > 90) m1_des_angle = 90;
    if (m1_des_angle < -90) m1_des_angle = -90;
  }
  if (mode == SINE_MODE) {
  // SINE MODE: Change m1 set-point based on sine function
    m1_des_angle = 45.*sin(millis()/1000.);
  }
  
  // Do motor control loop
  if (mode != ALL_OFF) {
    m1_pid();
  }
  else set_motor_speed(0);
  
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

  //send_bluetooth_data(); 
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

  // Current sensing and emergency stop:
  I_motor = get_current_sense();
//  Serial.println(String(millis()) + " " + String(I_motor));
//  if (I_motor > MAX_ALLOWED_MOTOR_CURRENT) {
//    emergency_stop();
//  }
}

