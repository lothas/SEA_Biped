#define    STX          'z' // 0x02
#define    ETX          'x' // 0x03
#define    LEDPIN       13
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

#define    BT_COMM_SPEED   9600 // 115200

extern volatile float out_angle;

// Bluetooth controller definitions
byte cmd[8] = {
  0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte button_status = 0;                                  // first Byte sent to Android device
long previous_millis = 0;                                // will store last time Buttons status was updated
long send_interval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String display_status = "xxxx";                          // message to Android device

void setup_bluetooth() {
  Serial1.begin(BT_COMM_SPEED);                           
  pinMode(LEDPIN, OUTPUT);
  while(Serial1.available())  Serial1.read();         // empty RX buffer
}

void read_bluetooth_data() {
  if(Serial1.available())  {                           // data received from smartphone
    delay(2);
    cmd[0] =  Serial1.read();  
    if(cmd[0] == STX)  {
      int i=1;      
      while(Serial1.available())  {
        delay(1);
        cmd[i] = Serial1.read();
        if(cmd[i]>127 || i>7)                 break;     // Communication error
        if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
        i++;
      }
      if     (i==2)          get_button_state(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      else if(i==7)          get_joystick_state(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
}

void send_bluetooth_data()  {
  static long previous_millis = 0;                             
  long current_millis = millis();
  if(current_millis - previous_millis > send_interval) {   // send data back to smartphone
    previous_millis = current_millis; 

    // Data frame transmitted back from Arduino to Android device:
    // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
    // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    Serial1.print((char)STX);                                             // Start of Transmission
    Serial1.print(get_button_status_string());  
    Serial1.print((char)0x1);   // buttons status feedback
    Serial1.print(get_data_int1());            
    Serial1.print((char)0x4);   // datafield #1
    Serial1.print(out_angle);          
    Serial1.print((char)0x5);   // datafield #2
    Serial1.print(display_status);                                         // datafield #3
    Serial1.print((char)ETX);                                             // End of Transmission
  }  
}

String get_button_status_string()  {
  String b_status = "";
  for(int i=0; i<6; i++)  {
    if(button_status & (B100000 >>i))      b_status += "1";
    else                                   b_status += "0";
  }
  return b_status;
}

int get_data_int1()  {              // Data dummy values sent to Android device for demo purpose
  static int i= -30;                // Replace with your own code
  i ++;
  if(i >0)    i = -30;
  return i;  
}

float get_data_float2()  {          // Data dummy values sent to Android device for demo purpose
  static float i=50;                // Replace with your own code
  i-=.5;
  if(i <-50)    i = 50;
  return i;  
}

void get_joystick_state(byte data[8])    {
  int joy_x = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
  int joy_y = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
  joy_x = joy_x - 200;                                                 // Offset to avoid
  joy_y = joy_y - 200;                                                 // transmitting negative numbers

  if(joy_x<-100 || joy_x>100 || joy_y<-100 || joy_y>100)    return;    // commmunication error

  // Your code here ...
  Serial.print("Joystick position:  ");
  Serial.print(joy_x);  
  Serial.print(", ");  
  Serial.println(joy_y); 
//  if (joyY>=0) {
//    SetMotorSpeed(1,(float)joyY/100.0);
//  }
//  else {
//    SetMotorSpeed(-1,(float)-joyY/100.0);
//  }
//  M1_des_angle = (float)joyY;
  des_torque = 0.1*float(joy_y);
}

void get_button_state(int b_status)  {
  switch (b_status) {
    // -----------------  BUTTON #1  -----------------------
  case 'A':
    button_status |= B000001;        // ON
    Serial.println("\n** Button_1: ON **");
    // your code...      
    display_status = "LED <ON>";
    Serial.println(display_status);
    digitalWrite(LEDPIN, HIGH);
    break;
  case 'B':
    button_status &= B111110;        // OFF
    Serial.println("\n** Button_1: OFF **");
    // your code...      
    display_status = "LED <OFF>";
    Serial.println(display_status);
    digitalWrite(LEDPIN, LOW);
    break;

    // -----------------  BUTTON #2  -----------------------
  case 'C':
    button_status |= B000010;        // ON
    Serial.println("\n** Button_2: ON **");
    // your code...      
    display_status = "Button2 <ON>";
    Serial.println(display_status);
    break;
  case 'D':
    button_status &= B111101;        // OFF
    Serial.println("\n** Button_2: OFF **");
    // your code...      
    display_status = "Button2 <OFF>";
    Serial.println(display_status);
    break;

    // -----------------  BUTTON #3  -----------------------
  case 'E':
    button_status |= B000100;        // ON
    Serial.println("\n** Button_3: ON **");
    // your code...      
    display_status = "Motor #1 enabled"; // Demo text message
    Serial.println(display_status);
    break;
  case 'F':
    button_status &= B111011;      // OFF
    Serial.println("\n** Button_3: OFF **");
    // your code...      
    display_status = "Motor #1 stopped";
    Serial.println(display_status);
    break;

    // -----------------  BUTTON #4  -----------------------
  case 'G':
    button_status |= B001000;       // ON
    Serial.println("\n** Button_4: ON **");
    // your code...      
    display_status = "Datafield update <FAST>";
    Serial.println(display_status);
    send_interval = FAST;
    break;
  case 'H':
    button_status &= B110111;    // OFF
    Serial.println("\n** Button_4: OFF **");
    // your code...      
    display_status = "Datafield update <SLOW>";
    Serial.println(display_status);
    send_interval = SLOW;
    break;

    // -----------------  BUTTON #5  -----------------------
  case 'I':           // configured as momentary button
    //      buttonStatus |= B010000;        // ON
    Serial.println("\n** Button_5: ++ pushed ++ **");
    // your code...      
    display_status = "Button5: <pushed>";
    break;
    //   case 'J':
    //     buttonStatus &= B101111;        // OFF
    //     // your code...      
    //     break;

    // -----------------  BUTTON #6  -----------------------
  case 'K':
    button_status |= B100000;        // ON
    Serial.println("\n** Button_6: ON **");
    // your code...      
    display_status = "Button6 <ON>"; // Demo text message
    break;
  case 'L':
    button_status &= B011111;        // OFF
    Serial.println("\n** Button_6: OFF **");
    // your code...      
    display_status = "Button6 <OFF>";
    break;
  }
  // ---------------------------------------------------------------
}
