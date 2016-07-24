#define SM1_STATES 5
#define SM2_STATES 4

#define SERVO_OUT             10
#define SERVO_IN              120
#define CPG_PERIOD            1500000.0
#define DOUBLE_STANCE_DUR     100000
#define HIP_THRESH            12.0

extern int error_type;
extern volatile float des_torque;
extern Servo middleFootServo;
extern Servo rightFootServo;
extern Servo leftFootServo;

// CPG variables
unsigned int sm1_state = 0;
unsigned int sm2_state = 0;

unsigned long sm1_t0 = 0;
unsigned long sm2_t0 = 0;

float pulse1_st = 0.1;
float pulse1_dt = 0.1;
float pulse1_amp = 00;
float pulse2_st = 0.6;
float pulse2_dt = 0.1;
float pulse2_amp = -0;

// State conditions function
void sm1_condition(unsigned long t_stamp) {
  float phase = float(t_stamp - sm1_t0)/CPG_PERIOD;
//  Serial.println("SM1 state: "+String(sm1_state)+", phase: "+String(phase));

  switch (sm1_state) {
    case 0: // Free swing after CPG reset
      if (phase>pulse1_st) sm1_change(t_stamp);
      break;
    case 1: // First pulse is active
      if (phase>pulse1_st+pulse1_dt) sm1_change(t_stamp);
      break;
    case 2: // Free swing between pulses
      if (phase>pulse2_st) sm1_change(t_stamp);
      break;
    case 3: // Second pulse is active
      if (phase>pulse2_st+pulse2_dt) sm1_change(t_stamp);
      break;
    case 4: // Free swing before CPG reset
      if (phase>1) sm1_change(t_stamp);
      break;
  }
}

void sm1_action() {
  // Called all the time
  switch (sm1_state) {
    default:
      return;  
  }
}

void sm1_change(unsigned long t_stamp) {
  // Called when changing from state to state
  // Perform state switch action
  switch (sm1_state) {
    case 0: // Activate first pulse
      des_torque = pulse1_amp;
      break;
    case 1: // Stop first pulse
      des_torque = 0;
      break;
    case 2: // Activate second pulse
      des_torque = pulse2_amp;
      break;
    case 3: // Second pulse is active
      des_torque = 0;
      break;
    case 4: // Reset CPG
      sm1_t0 = t_stamp;
      break;
    default:
      return;  
  }
  // Switch state
  ++sm1_state;
  if (sm1_state>=SM1_STATES) sm1_state = 0;
}

void sm2_condition(unsigned long t_stamp, float hip_angle) {
//  if (n<1) error_type = 6, return;
//  Serial.println("SM2 state: "+String(sm2_state)+", hip angle: "+String(hip_angle));

  switch (sm2_state) {
    case 0: // Inner leg is stance leg, outer is swing
      if (hip_angle>HIP_THRESH) sm2_change(t_stamp);
      break;
    case 1: // Double stance (before inner swing)
      if (t_stamp>sm2_t0) sm2_change(t_stamp);
      break;
    case 2: // Inner leg is stance leg, outer is swing
      if (hip_angle<-HIP_THRESH) sm2_change(t_stamp);
      break;
    case 3: // Double stance (before inner swing)
      if (t_stamp>sm2_t0) sm2_change(t_stamp);
      break;
  }
}

void sm2_action() {
  // Called all the time
  switch (sm2_state) {
    default:
      return;  
  }
}

void sm2_change(unsigned long t_stamp) {
  // Called when changing from state to state
  // Perform state switch action
  switch (sm2_state) {
    case 0: // Extend outer feet
      moveFootServo(rightFootServo, SERVO_OUT);
      moveFootServo(leftFootServo, SERVO_OUT);
      sm2_t0 = t_stamp+DOUBLE_STANCE_DUR;
      break;
    case 1: // Retract inner foot
      moveFootServo(middleFootServo, SERVO_IN);
      break;    
    case 2: // Extend inner foot
      moveFootServo(middleFootServo, SERVO_OUT);
      sm2_t0 = t_stamp+DOUBLE_STANCE_DUR;
      break;
    case 3: // Retract outer feet
      moveFootServo(rightFootServo, SERVO_IN);
      moveFootServo(leftFootServo, SERVO_IN);
      break;
    default:
      return;  
  }
  // Switch state
  ++sm2_state;
  if (sm2_state>=SM2_STATES) sm2_state = 0;
}
