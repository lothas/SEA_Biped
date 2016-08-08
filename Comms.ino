#define    PC_COMM_SPEED   115200
#define    BT_COMM_SPEED   9600 // 115200

#define    STX             'z'  // 0x02
#define    ETX             'x'  // 0x03

extern float out_angle;

// Serial communication variables
char pc_comm[8] = {0,0,0,0,0,0,0,0};
int comm_idx = 0;
char bt_comm[8] = {0,0,0,0,0,0,0,0};
int bt_comm_idx = 0;

// Bluetooth controller definitions
long previous_millis = 0;                                // will store last time Buttons status was updated
String display_status = "xxxx";                          // message to Android device

void setup_comms() {
  Serial.begin(PC_COMM_SPEED);
  
  Serial1.begin(BT_COMM_SPEED);
  while(Serial1.available())  Serial1.read();            // empty RX buffer
}

void check_pc_comm() {
  if(Serial.available()) {
    pc_comm[comm_idx] = Serial.read();
    if (comm_idx == 0) {
      if (pc_comm[comm_idx] == STX) {
        // Start reading command
        comm_idx++;
      }
    }
    else {
      if (pc_comm[comm_idx] == ETX || comm_idx>=7) {
        // Command received
        read_command(pc_comm);
        comm_idx = 0;
      }
      else {
        comm_idx++;
      }
    }
  }
}

void check_bt_comm() {
  if(Serial1.available()) {
    bt_comm[bt_comm_idx] = Serial1.read();
    if (bt_comm_idx == 0) {
      if (bt_comm[bt_comm_idx] == STX) {
        // Start reading command
        bt_comm_idx++;
      }
    }
    else {
      if (bt_comm[bt_comm_idx] == ETX || bt_comm_idx>=7) {
        // Command received
        read_command(bt_comm);
        bt_comm_idx = 0;
      }
      else {
        bt_comm_idx++;
      }
    }
  }
}

void read_command(char cmd[8]) {
  Serial.print("Received command: ");
  int pc_input = 0;

  if (cmd[0] != STX) Serial.println("Transmission error");

  switch (cmd[1]) {
    case '1': // ////// Go to active mode command //////////////
      {
      if (cmd[2] == ETX) {
        Serial1.println("Start walking");
        mode = SEA_MODE;
        m1_des_angle = m1_angle;
        des_torque = 0;
        
        // Start state machines
        sm1_go = 1;
        sm2_go = 1;

        // Retract middle foot
        moveFootServo(middleFootServo, SERVO_OUT);
      }
      else Serial.println("Transmission error");
      }
      break;
    case '2': // ///// Go to passive mode command //////////////
      {
      if (cmd[2] == ETX) {
        Serial1.println("Passive mode");
        mode = SEA_MODE;
        m1_des_angle = m1_angle;
        des_torque = 0;
        
        // Stop state machines
        sm1_go = 0;
        sm2_go = 0;

        // Retract middle foot
        moveFootServo(middleFootServo, SERVO_OUT);
      }
      else Serial.println("Transmission error");
      }
      break;
    case '3': // /////// Go to rest mode command ///////////////
      {
      if (cmd[2] == ETX) {
        Serial1.println("Go to rest position");
        // Stop state machines
        sm1_go = 0;
        sm2_go = 0;

        // Spread legs
        mode = PID_MODE;
        m1_des_angle = 30;

        // Extend feet
        moveFootServo(rightFootServo, SERVO_OUT);
        moveFootServo(leftFootServo, SERVO_OUT);
        moveFootServo(middleFootServo, SERVO_OUT);
      }
      else Serial.println("Transmission error");
      }
      break;
      
    // ---------------- STATE MACHINE COMMANDS ------------------
    case '4': // /////// Toggle SM1 command /////////////////////
      {
      if (cmd[2] == ETX) {
        sm1_go = !sm1_go;
        Serial1.println(sm1_go ? "Turned State Machine 1 ON" : "Turned State Machine 1 OFF");
      }
      else Serial.println("Transmission error");
      }
      break;
    case '5': // /////// Toggle SM2 command /////////////////////
      {
      if (cmd[2] == ETX) {
        sm2_go = !sm2_go;
        Serial1.println(sm2_go ? "Turned State Machine 2 ON" : "Turned State Machine 2 OFF");
      }
      else Serial.println("Transmission error");
      }
      break;
      
    // ----------------- GLOBAL CONTROL COMMANDS -----------------
    case ETX: // ///// Emergency stop command /////////////////
      {
      error_type = 1;
      emergency_stop(); // also sets mode to ALL_OFF
      }
      break;
    case 'e': // ///////// Clear error command //////////////////
      {
      if (cmd[2] == ETX) {
        error_type = 0;
        Serial.println("Error cleared");
      }
      else Serial.println("Transmission error");
      }
      break;
    case 'r': // //////// Clear angles command ///////////////////
      {
      if (cmd[2] == ETX) {
        reset_encoders();
        m1_des_angle = m1_angle;
        des_torque = 0;
        Serial.println("Angles cleared");
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
  
      if (cmd[4] == ETX) {
        mode = PID_MODE;
        m1_des_angle += delta_m1;
        Serial.println("Move motor by "+String(delta_m1));
      }
      else Serial.println("Transmission error");
      }
      break;
    case 's': // ////// Sine trajectory command /////////////////
      {
      if (cmd[2] == ETX) {
        mode = SINE_MODE;
        Serial.println("Start sinusoidal motion");
      }
      else Serial.println("Transmission error");
      }
      break;
    
    default:
      int idx = 1;
      while (true) {
        if (cmd[idx] == ETX || idx>=7) {
          // transmission ended
          break;
        }
        pc_input = 10*pc_input + cmd[idx] - 48;
        Serial.print(cmd[idx++]);
      }
      
      if (idx == 4) {
        mode = SEA_MODE;
        digitalWrite(13,HIGH);
        Serial.print("New desired speed: ");
        Serial.println(pc_input-200);
        des_torque = pc_input-200;
      }
      break;
  }
}

void send_bluetooth_data()  {
  static long previous_millis = 0;                             
  long current_millis = millis();
  if(current_millis - previous_millis > 250) {   // send data back to smartphone
    previous_millis = current_millis; 

    // Data frame transmitted back from Arduino to Android device:
    // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
    // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    Serial1.print((char)STX);                                             // Start of Transmission
    Serial1.print(" ");  
    Serial1.print((char)0x1);   // buttons status feedback
    Serial1.print(0);            
    Serial1.print((char)0x4);   // datafield #1
    Serial1.print(out_angle);          
    Serial1.print((char)0x5);   // datafield #2
    Serial1.print(display_status);                                         // datafield #3
    Serial1.print((char)ETX);                                             // End of Transmission
  }  
}
