//#include <NewPing.h>  // imports the NewPing library for ultrasonic sensor
  #include <SPI.h>    // import the relevent library for SPI communication
  #include <nRF24L01.h>  // import the library for radio module
  #include <RF24.h>
  #include <SoftwareSerial.h>
  #include <LiquidCrystal_I2C.h>


  //          sensors

  // IR
  #define IR1 A7 //robot left
  #define IR2 A6
  #define IR3 A5
  #define IR4 A4
  #define IR5 A3
  #define IR6 A2
  #define IR7 A1
  #define IR8 A0 // robot right

  // Ultrasonic
  #define trig 38 //
  #define echo 39 //

  // RGB sensor
  #define S0 4  //yellow and orange
  #define S1 1 //orage
  #define S2 5 //purple
  #define S3 2  // ash
  #define sensorOut 3 // blue

  //NewPing ultrasonic(trig, echo); //defines a new ultrasonic variable


  //          Actuators
  // Motor A connections
  int ENA = 8;
  int IN1 = 9;
  int IN2 = 10;
  // Motor B connections
  int ENB = 13;
  int IN3 = 11;
  int IN4 = 12;


  //          Communication

  //radio
  #define ce 53
  #define csn 49
  RF24 radio(ce,csn);  //defines a new radio variable CE, CSN respectively


  // bluetooth
  #define RX1 17
  #define TX1 16
  SoftwareSerial bluetoothSerial(RX1, TX1);
  //ISSUE :- can send data from phone to mega


  // lcd display
  LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows use File > Examples > Wire > i2c_scanner to find address
  //functions -> disLcd,displayLine
  //shown lines in lcd
  String firstLine = "";
  String lastLine = "";

  //buzzer
  #define buzzerd 26

  //rgb led
  #define bled A15 //49 
  #define gled A14 //51
  #define rled A13 //53

  //keypad
  #define key1 A8
  #define key2 A9
  #define key3 A10
  #define key4 A11

  bool ks1 = false;
  bool ks2 = false;
  bool ks3 = false;
  bool ks4 = false;

  //this stores previous values got from the keystroke
  bool tp1 = false;
  bool tp2 = false;
  bool tp3 = false;
  bool tp4 = false;


  int Threshold = 200;
  int IR_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};      // IR_Bin_val[0] - left side IR sensor  // IR_Bin_val[10] - right side IR sensor
  int IR_Bin_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  double IR_weights[8] = { -4, -3, -2, -1, 1, 2, 3, 4}; //{-5000, -2000, -1000, 1000, 2000, 5000}; //{-4000, -2000, -1000, 1000, 2000, 4000};  //{-32, -16, -8, -4, -2, 0, 2, 4, 8, 16, 32}

  int LMotorSpeed = 0;
  int RMotorSpeed = 0;
  int speed_adjust = 0;
  int Left_MotorBase_speed =167;//167;//  
  int Right_MotorBase_speed =140;//150;//    
  int max_speed = 255;
  int min_speed = 50 ;

  // for color sensor readings
  int redfrequency = 0;
  int greenfrequency = 0;
  int bluefrequency = 0;

  //front values
  float Kp = 38;//25;  //22,0 wada//80 //25
  float Kd = 0;//10; // 8//50 //10                         //0.005
  float Ki = 0;

  float P, D;
  float I = 0;   // I is again and again set to zero in pid_forward()

  float error = 0;
  float previousError = 0; //46

  //back pid

  float Kpb = 22;  // 3.05, 15, 0.001 for 43 max    // 1.875*0.3=0.5625 // 1.875 is what you need to utilize the full range  //12*0.25
  float Kdb = 0; // 8
  float Kib = 0;

  float Pb, Db;
  float Ib = 0;   // I is again and again set to zero in pid_forward()

  float errorb = 0;
  float previousErrorb = 0; //46

  //slow front pid


  //slow back pid

  //turn delays
  int dR90 = 250;
  int dL90 = 250;
  int dL180 = 750;



  //detect junctions or boxes
  int waitTimeJunc = 300;

  //go certain distance
  int waitTimeDist = 1000;
  int waitArr[3] = {500, 1000, 2000};//5cm, 10cm, 20cm

  const byte address[][6] = {"00001","00002"}; //the address to which the module should listen for incoming messages

  String text = "0", txt; // these can be used as global variables in any function
  String serialTxt = "";
  String bluetoothTxt = "";

  String state = "stop";


  //functions legacy
  void set_forward(); //checked
  void set_backward();//checked
  void forward(int Left = Left_MotorBase_speed, int Right = Right_MotorBase_speed);  // sets forward and go forward //checked
  void pid_forward(int steps);  // go forward with pid output for a given number of steps //checked
  void backward(int Left = Left_MotorBase_speed, int Right = Right_MotorBase_speed);//checked
  void pid_backward(int steps);//not checked, not working( need to change back line following algo)
  void turn_left_90(int dTime = dL90);//checked
  void turn_right_90(int dTime = dR90);//checked
  void turn_left_180(int dTime = dL180);//not checked
  void turn_right_180(int dTime = dL180);
  void stop();//checked
  void motor_speed();//checked
  void read_ir();//checked
  void line_follow();//checked
  float measure_distance();//checked
  int read_color_sensor();//not giving consistant values
  void send_color_value();//not checked
  void turn_left_until_middle();
  void turn_right_until_middle();
  void radio_send();

  //not included in the code
  void start_command();
  void _to_checkpoint1();
  void checkpoint_one_to_gate_A();
  void send_box_color();
  void gate_A_to_vault();
  void vault_to_ramp_junction();
  void ramp_junction_to_ramp_start();
  void ramp_start_to_gate_B();
  void ramp();
  void gate_B_to_ramp_end();
  void ramp_end_to_T();
  void send_obstacle_removed();
  void T_to_ramp_end();
  void send_ok();
  void ramp_end_to_pole_junction();
  void pole_junction_to_pole();
  void pole_to_pole_junction();
  void pole_junction_to_finish();

  void checkpoint_follow();

  //functions--RuchchaSD
  String read_Serial(); //get String sent through serial
  String read_bluetooth(); //get String sent through bluetooth.
  void horn(int dtime = 0); //to sound buzzer positive arg for delay 0 or no arg to keep the buzzer negative to turn off

  void display_lcd(String fline = "", String sline = "", int dtime = 0); //shows whats been passed to upper and lowerlines. dtime is used to keep the line for a while and erase it
  void display_line(String txt, int row = 0); // used to display a line in a row

  void show_color(String color);// function to control led "red" "green" "blue" args work. check function to see what other colors can be show
  //to get white the led is needed be attached to pwm and tune the values until all junctions work (explain this later)

  bool update_btns();//function to update what's pressed

  int get_pressed_button(); // returns what button is pressed - holds in a loop until a button is pressed and released returns 5 if 1 and 4 is pressed at the same time otherwise(1 2 3 4)
  String look_for_junc(int wait = waitTimeJunc);//function to identify junctions identifies right lift 4way and differentiates them from boxes - box box right box left
  //(may need to update)

  //ui functions
  void menu(); //main menu can be accesed by long pressing 4 when inside of the loop
  void change_forward_linefollow_time();//ui to change the pid forward time (this is used to measure distances)
  void change_det_jun();//ui to change junction and box differentiation time
  void change_backward_pid();//ui to change back pid values
  void change_forward_pid();//ui to change forward pid values

  // turn lrft or right function you can specify the motor speed to turn the car depending on the scenario otherwise default = base speeds
  void turn_left(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed);
  void turn_right(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed);

  //State manager funcs
  void line_back();//backward line following (current algo in it does not work change to a suitable one)
  void line_front();//forward line follow press 3 to update pid while running the func
  void jun_det_line_follow();//junction detection while line following press 3 to update time to  differentiate a junction from a box(this works but there may be bugs in it. test extensively)
  //not tested for 4way junctions as well some times this may identify 4way as left or right depending on whichside crossed the line first

  void mes_dis_travel();//this function is made to measure time it takes to travel distances - not checked
  void det_jun_turn_90();//detect junctions and turn
  void turn_180();//go forward and turn 180
  void displayIr(String firstLine = "");

  int current_time = millis();
  int color = -1;
  int stage = -1;
  int t = 350;

  int t1 = 0, t2 = 0;
  int idx;
  void setup() {
    //          Actuators // Set all the motor control pins to outputs
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    // Turn off motors - Initial state
    set_forward();

    //          sensors

    //ultrasonic
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    // color sensor https://randomnerdtutorials.com/arduino-color-sensor-tcs230-tcs3200/
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(sensorOut, INPUT);
    // Setting its frequency-scaling to 20%
    digitalWrite(S0, LOW);
    digitalWrite(S1, HIGH);

    //          communication
    // bluetooth
    Serial.begin(9600);
    bluetoothSerial.begin(9600);

  radio.begin();
  radio.startListening();
  radio.openWritingPipe(address[0]); //00001
  radio.openReadingPipe(1, address[1]);  //00002
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);

    //buzzer
    pinMode(buzzerd, OUTPUT);
    horn(-1);

    //led
    pinMode(rled, OUTPUT);
    pinMode(gled, OUTPUT);
    pinMode(bled, OUTPUT);
    show_color("off");

    // initialize the lcd
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.noCursor();

    // initialize the keypad
    pinMode(key1, INPUT_PULLUP);
    pinMode(key2, INPUT_PULLUP);
    pinMode(key3, INPUT_PULLUP);
    pinMode(key4, INPUT_PULLUP);


    display_lcd("TechBots", "Beetle");
    delay(1000);

    t1 = millis();
  //   //state = "stop";  // change the starting state
  //   forward();
  // // turn_left();
  // // delay(750);
  // // stop();
  // delay(t);
  //   stop();
  //   delay(100000);

  stage = 1;
}


void loop() {

//line_follow();


// start_command();
//_to_checkpoint1();
//  checkpoint_one_to_gate_A();
//  gate_A_to_vault();
//  vault_to_ramp_junction();
//   ramp_junction_to_ramp_start();
//  ramp_start_to_gate_B();
//  gate_B_to_ramp_end();
// ramp_end_to_T();
//  T_to_ramp_end();
//  send_obstacle_removed();
// ramp_end_to_pole_junction();
// pole_junction_to_pole();
//  pole_to_pole_junction();
//  pole_junction_to_finish();

// backward();
// delay(500);

//delay(1000000);
//line_follow();
}

//bluetooth functions
String read_bluetooth() {
  String bTxt = "";
  char c = "";
  while (bluetoothSerial.available()) {
    c = bluetoothSerial.read();
    bTxt += c;
  }
  return bTxt;
}
//Serial functions
String read_Serial() {
  String sTxt = "";
  char c = "";
  while (Serial.available()) {
    c = Serial.read();
    sTxt += c;
  }
  return sTxt;
}

//RGB led functions
void show_color(String color) {
  if (color == "red") {
    color = "2";
  } else if (color == "green") {
    color = "6";
  } else if (color == "blue") {
    color = "5";
  }

  int q = (int)color[0] - 48;
  switch (q) {
    case 8:
      digitalWrite(rled, HIGH);
      digitalWrite(gled, HIGH);
      digitalWrite(bled, HIGH);
      break;
    case 7:
      digitalWrite(rled, LOW);
      digitalWrite(gled, HIGH);
      digitalWrite(bled, HIGH);
      break;
    case 6:
      digitalWrite(rled, HIGH);
      digitalWrite(gled, LOW);
      digitalWrite(bled, HIGH);
      break;
    case 5:
      digitalWrite(rled, LOW);
      digitalWrite(gled, LOW);
      digitalWrite(bled, HIGH);
      break;
    case 4:
      digitalWrite(rled, HIGH);
      digitalWrite(gled, HIGH);
      digitalWrite(bled, LOW);
      break;
    case 3:
      digitalWrite(rled, LOW);
      digitalWrite(gled, HIGH);
      digitalWrite(bled, LOW);
      break;
    case 2:
      digitalWrite(rled, HIGH);
      digitalWrite(gled, LOW);
      digitalWrite(bled, LOW);
      break;
    default:
      digitalWrite(rled, LOW);
      digitalWrite(gled, LOW);
      digitalWrite(bled, LOW);
  }
}

// buzzer functions
void horn(int dtime) { //postive delay, negative off, no arg keep horn
  if (dtime == 0) {
    digitalWrite(buzzerd, LOW);
  } else if (dtime > 0) {
    digitalWrite(buzzerd, LOW);
    delay(dtime);
    digitalWrite(buzzerd, HIGH);
  } else {
    digitalWrite(buzzerd, HIGH);
  }
}

// display functions
void display_lcd(String fline, String sLine, int dtime) {

  if (dtime == 0) {

    if (fline != firstLine && sLine != lastLine) {
      lcd.clear();
      display_line(fline, 0);
      firstLine = fline;
      display_line(sLine, 1);
      lastLine = sLine;
    } else if (fline != firstLine) {
      display_line("                ", 0);
      display_line(fline, 0);
      firstLine = fline;
    } else if (sLine != lastLine) {
      display_line("                ", 1);
      display_line(sLine, 1);
      lastLine = sLine;
    }

  } else if (dtime > 0) {
    lcd.clear();
    display_line(fline, 0);
    display_line(sLine, 1);
    delay(dtime);
    lcd.clear();
    display_lcd(firstLine, lastLine);

  } else {
    firstLine = "";
    lastLine = "";
    lcd.clear();
  }
}

void display_line(String txt, int row) {
  int start = (16 - txt.length()) / 2;
  for (int i = start; i < txt.length() + start && i < 16 ; i++) {
    lcd.setCursor(i, row);
    lcd.print(txt[i - start]);
  }
}

// ir sensors functions
void read_ir() {
  for (int i = 0; i < 8; i++)
  {
    IR_val[i] = 0;
  }
  for (int i = 0; i < 4 ; i++)
  {
    IR_val[0] += analogRead(IR1);
    IR_val[1] += analogRead(IR2);
    IR_val[2] += analogRead(IR3);
    IR_val[3] += analogRead(IR4);
    IR_val[4] += analogRead(IR5);
    IR_val[5] += analogRead(IR6);
    IR_val[6] += analogRead(IR7);
    IR_val[7] += analogRead(IR8);
  }
  for (int i = 0; i < 8; i++)
  {
    IR_val[i] = IR_val[i] / 4;
  }

  for (int i = 0; i < 8; i++)
  {
    if (IR_val[i] >= Threshold)
    {
      IR_Bin_val[i] = 0;       //change for white strips on black surface
    }
    else
    {
      IR_Bin_val[i] = 1;
    }
  }
}

void displayIr(String firstLine) {
  String t = "";
  for (int i = 0; i < 8; i++ ) {
    t += IR_Bin_val[i] + " ";
  }
  display_lcd(firstLine, t);
}

String look_for_junc(int wait) {
  read_ir();
  bool right = IR_Bin_val[0] && IR_Bin_val[1] && IR_Bin_val[2] , left = IR_Bin_val[7] && IR_Bin_val[6] && IR_Bin_val[5];
  if (right && left) {
    pid_forward(wait);
    read_ir();
    line_follow();
    right = IR_Bin_val[0] && IR_Bin_val[1] && IR_Bin_val[2];
    left = IR_Bin_val[7] && IR_Bin_val[6] && IR_Bin_val[5];
    if (right && left) {
      return "box";
    } else {
      return "4 way";
    }
  } else if (left) {
    forward();
    delay(wait);
    read_ir();
    line_follow();
    left = IR_Bin_val[7] && IR_Bin_val[6] && IR_Bin_val[5];
    if (left) {
      return "box left";
    } else {
      return "left";
    }
  } else if (right) {
    forward();
    delay(wait);
    read_ir();
    line_follow();
    right = IR_Bin_val[0] && IR_Bin_val[1] && IR_Bin_val[2];
    if (left) {
      return "box right";
    } else {
      return "right";
    }
  } else {
    return "no_junc";
  }

}

// ultrasonic sensor functions
float measure_distance() {
  digitalWrite(trig, HIGH);
  delayMicroseconds(12);
  digitalWrite(trig, LOW);
  float T = pulseIn(echo, HIGH);
  float x = T / 58.00;
  return x;
}

// color sensor functions
int read_color_sensor() {
  int i = 0;
  int R_val = 0, G_val = 0, B_val = 0, GR_val = 0, W_val = 0;
  do
  {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    redfrequency = pulseIn(sensorOut, LOW);

    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    greenfrequency = pulseIn(sensorOut, LOW);

    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    bluefrequency = pulseIn(sensorOut, LOW);

    if ((redfrequency > 30 && redfrequency < 95) && (bluefrequency > 60 && bluefrequency < 123) && (greenfrequency > 80 && greenfrequency < 160)) R_val++;
    else if ((redfrequency > 110 && redfrequency < 190) && (bluefrequency > 80 && bluefrequency < 165) && (greenfrequency > 70 && greenfrequency < 180)) G_val++;
    else if ((redfrequency > 110 && redfrequency < 190) && (bluefrequency > 30 && bluefrequency < 120) && (greenfrequency > 70 && greenfrequency < 190)) B_val++;
    else if ((redfrequency > 20 && redfrequency < 80) && (bluefrequency > 60 && bluefrequency < 123) && (greenfrequency > 80 && greenfrequency < 160)) GR_val++;
    else if (bluefrequency < 100 && redfrequency < 100 && greenfrequency < 100) W_val++;
    i++;
  }
  while (i < 50);

  if (R_val > 40) return 1;
  else if (G_val > 40) return 2;
  else if (B_val > 40) return 3;
  else if (GR_val > 40) return 4;
  else if (W_val > 40) return 5;
  
}


//keypad functions
bool update_btns() {
  // bool tp1 = false;
  // bool tp2 = false;
  // bool tp3 = false;
  // bool tp4 = false;
  bool pressed = false;
  ks1 = tp1;
  ks2 = tp2;
  ks3 = tp3;
  ks4 = tp4;
  tp1 = false;
  tp2 = false;
  tp3 = false;
  tp4 = false;
  if (digitalRead(key1) == LOW) {
    tp1 = true;
    pressed = true;
  }
  if (digitalRead(key2) == LOW) {
    tp2 = true;
    pressed = true;
  }
  if (digitalRead(key3) == LOW) {
    tp3 = true;
    pressed = true;
  }
  if (digitalRead(key4) == LOW) {
    tp4 = true;
    pressed = true;
  }
  return pressed;
}

//state manager functions
//state = back
void line_back() {
  display_lcd("backward linefollow", "Kpb: " + (String)((int)Kpb) + " Kdb: " + (String)((int)Kdb) );
  set_backward();
  line_follow_back();
  if (ks3 == true) { // to change pid
    change_backward_pid();
  }
}
//state = forward
void line_front() {
  display_lcd("Line Forward", "Kp:" + (String)(int)Kp + " Kd:" + (String)(int)Kd);
  //basic line following and pid value tuning
  set_forward();
  line_follow();

  if (ks3 == true) { // to change pid
    change_forward_pid();
  }
}
//state = det_junc
void jun_det_line_follow() {
  // display_lcd("forward linefollow","Kp: "+(String)(int)Kp);
  // horn(300);
  // delay(500);
  set_forward();
  line_follow();
  String jType = look_for_junc(waitTimeJunc); //detects four types of junctions
  display_lcd("Find Junctions", jType);
  line_follow();
  if (jType == "right") {
    display_lcd("right");
    horn(0);
    pid_forward(250);
    horn(-1);
    line_follow();
  } else if (jType == "left") {
    display_lcd("left");
    for (int i = 0; i < 3; i++  ) {
      horn(0);
      pid_forward(250);
      horn(-1);
      pid_forward(250);
    }
    line_follow();
  } else if (jType == "4way") {
    display_lcd("4way");
    for (int i = 0; i < 2; i++  ) {
      horn(0);
      pid_forward(250);
      horn(-1);
      pid_forward(250);
    }
  } else if (jType == "box" || jType == "box left" || jType == "box right") {
    display_lcd(jType);
    stop();
    delay(2000);
    // state = "stop";
    text = "found : " + jType;
    for (char c : text) {
      bluetoothSerial.write(c);
    }
    bluetoothSerial.write("\n");
    for (int i = 0; i < 5; i++  ) {
      horn(100);
      delay(300);
    }
  }

  if (ks3 == true) { // to change pid
    change_det_jun();//time to wait until cheking again
  }
}
//state = get distance
void mes_dis_travel() {
  display_lcd("measure distance", (String)(int)waitTimeDist);
  set_forward();
  line_follow();
  if (look_for_junc(waitTimeJunc) == "4way") {
    pid_forward(waitTimeDist);
  }

  if (ks3 == true) { // to change pid
    change_forward_linefollow_time();//time to go some distance  5cm 10cm 20cm
    // changebpidtime();
    // changespidtime();
  }
}

//state turn 90
void det_jun_turn_90() {
  display_lcd("Turn 90 Right", "TR:" + (String)(int)dR90 + " TL:" + (String)(int)dL90);
  set_forward();
  line_follow();
  String jType = look_for_junc(waitTimeJunc);
  if (jType == "left") {
    stop();
    horn(300);
    turn_left_90();
  } else if (jType == "right") {
    stop();
    horn(300);
    delay(500);
    horn(300);
    turn_right_90();
  }

  if (ks3 == true) { // to turning time
    change_turn90_right_time();
  } else if (ks2 == true) {
    change_turn90_left_time();
  }

}
//state = turn 180
void turn_180() {
  display_lcd("Turn 180 left", "TL180 : " + (String)(int)dL180);
  if (ks1 == true ) {
    set_forward();
    pid_forward(2000);
    turn_left_180();
  } else if (ks3 == true) {
    change_turn180_left_time();
  }
}


// motors
void motor_speed() {
  analogWrite(ENB, LMotorSpeed);
  analogWrite(ENA, RMotorSpeed);
}

void set_forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void set_backward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void forward(int Left, int Right) { // left right speeds
  Left_MotorBase_speed = Left;
  Right_MotorBase_speed = Right;

  set_forward();

  LMotorSpeed = Left_MotorBase_speed;
  RMotorSpeed = Right_MotorBase_speed;
  motor_speed();
}

void backward(int Left, int Right) {

  Left_MotorBase_speed = Left;
  Right_MotorBase_speed = Right;

  set_backward();

  LMotorSpeed = Left_MotorBase_speed;
  RMotorSpeed = Right_MotorBase_speed;

  motor_speed();
}

void pid_forward(int count) {
    I = 0;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    //encoder_pos = 0;
    // while (encoder_pos < target){do this}
    for (int i=0;i<count;i++){
      read_ir();
      line_follow();
      //delay(2);
    }
    stop();
    delay(2);
}

void pid_backword(int count) {
  I = 0;
  set_backward();
  for (int i = 0; i < count; i++) {
    read_ir();
    line_follow_back();
    delay(2);
  }
  stop();
}

void turn_left(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed) {
  digitalWrite(IN1, LOW);//left front
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);//right back
  digitalWrite(IN4, HIGH);
  LMotorSpeed = left; // may need to change this
  RMotorSpeed = right;
  motor_speed();
}

void turn_right(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed) {
  digitalWrite(IN1, HIGH);// left motor back
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);// right motor front
  digitalWrite(IN4, LOW);
  LMotorSpeed = left; // may need to change this
  RMotorSpeed = right;
  motor_speed();
}

void turn_left_90(int dTime = dL90) {
  turn_left();
  delay(dTime);  // will have to change the delay
  stop();//pass different values if necessary
  read_ir();
  do {
    read_ir();
     turn_left();
  } while(!(IR_Bin_val[4] == 1 && IR_Bin_val[3] == 1));// changed for testig - added IR_Bin_val[3] == 0; // && IR_val[0] == 0 && IR_val[1] == 0 && IR_val[6] == 0 && IR_val[7] == 0);
  stop();
}

void turn_right_90(int dTime = dR90) {
  turn_right();
  delay(dTime);
  stop();
  read_ir();
  do {
    read_ir();
      turn_right();//pass different values if necessary
  } while (!(IR_Bin_val[4] == 1 && IR_Bin_val[3] == 1));// changed for testig - added IR_Bin_val[3] == 0);
  stop();
}

void turn_left_180(int dTime = dL180) {
  turn_left();
  delay(dTime);  // will have to change the delay
  stop();
  read_ir();
  do {
    read_ir();
      turn_left();//pass different values if necessary
  } while (!(IR_Bin_val[4] == 1 && IR_Bin_val[3] == 1));// changed for testig - added IR_Bin_val[3] == 0;
  stop();
}

void turn_right_180(int dTime = dL180) {
  turn_right();
  delay(dTime);  // will have to change the delay
  stop();
  read_ir();
  do {
    read_ir();
      turn_right();//pass different values if necessary
  } while (!(IR_Bin_val[3]==1 && IR_Bin_val[4]==1));
  stop();
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

// algorithms
void line_follow() {
  error = 0;
  set_forward();
  read_ir();
  for (int i = 0; i < 8; i++)
  {
    error += IR_weights[i] * IR_Bin_val[i];  //IR_Bin_val
  }

  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speed_adjust = Kp * P + Ki * I + Kd * D;
  LMotorSpeed = Left_MotorBase_speed - speed_adjust;
  RMotorSpeed = Right_MotorBase_speed + 0.9 * speed_adjust;

  if (LMotorSpeed < min_speed)
  {
    LMotorSpeed = min_speed;
  }
  if (RMotorSpeed < min_speed)
  {
    RMotorSpeed = min_speed;
  }
  if (LMotorSpeed > max_speed)
  {
    LMotorSpeed = max_speed;
  }
  if (RMotorSpeed > max_speed)
  {
    RMotorSpeed = max_speed;
  }
  // Serial.println("L: "+(String)LMotorSpeed +"  R: "+ (String)RMotorSpeed );
  motor_speed();
}

void line_follow_back() {
  errorb = 0;
  read_ir();
  for (int i = 0; i < 8; i++)
  {
    errorb += IR_weights[i] * IR_Bin_val[i];  //IR_Bin_val
  }

  Pb = errorb;
  Ib = I + error;
  Db = errorb - previousErrorb;

  previousErrorb = errorb;

  speed_adjust = Kpb * Pb + Kib * Ib + Kdb * Db;
  LMotorSpeed = Left_MotorBase_speed - speed_adjust;
  RMotorSpeed = Right_MotorBase_speed + 0.9 * speed_adjust;

  if (LMotorSpeed < min_speed)
  {
    LMotorSpeed = min_speed;
  }
  if (RMotorSpeed < min_speed)
  {
    RMotorSpeed = min_speed;
  }
  if (LMotorSpeed > max_speed)
  {
    LMotorSpeed = max_speed;
  }
  if (RMotorSpeed > max_speed)
  {
    RMotorSpeed = max_speed;
  }
  // Serial.println("L: "+(String)LMotorSpeed +"  R: "+ (String)RMotorSpeed );
  motor_speed();
}

//ui
void menu() {
  while (update_btns()) {
    delay(2);
  }


  display_lcd("Menu", "for bac jun ult ");
  int choice = get_pressed_button();

  if (choice == 1) {
    state = "forward";
    display_lcd("Line Following", "Forward");
    delay(1000);

  } else if (choice == 2) {
    state = "back";
    display_lcd("Line Following", "backward");
    delay(1000);

  } else if (choice == 3) {

    display_lcd("Junction train", "get dis 90 180 ");
    choice = get_pressed_button();
    switch (choice) {
      case 1:
        state = "detect_junc";
        break;
      case 2:
        state = "get distance";
        break;
      case 3:
        state = "turn 90";
        break;
      case 4:
        state = "turn 180";
        break;
    }

    display_lcd("State", state);

  } else if (choice == 4) {
    state = "ultra";
    display_lcd("gate training", state);
  }
}

int get_pressed_button() {
  while (!(update_btns())) { //this is to wait until a button is pressed
    delay(5);

  }

  while ((update_btns())) { //this is to wait until the button is released
    delay(5);
    if (ks1 == true && ks4 == true) {
      display_lcd("State 5", "Entered");
      delay((1000));
      return 5;
    }
  }

  if (ks1 == true) {
    return 1;
  } else if (ks2 == true ) {
    return 2;
  } else if (ks3 == true ) {
    return 3;
  } else {
    return 4;
  }

}

void change_forward_pid() { //ui to change forward pid values
  stop();
  while (true) {
    int choice = get_pressed_button();
    update_btns();
    if (choice == 1) {
      Kp--;
    } else if (choice == 2) {
      Kp++;
    } else if (choice == 3) {
      Kd--;
    } else if (choice == 4) {
      Kd++;
    } else if (choice == 5 ) { //to get out of this you need to press 1 and 4 at the same time
      display_lcd("Kp : " + (String)Kp, "Kd : " + (String)Kd);
      update_btns();
      break;
    }
    display_lcd("Kp : " + (String)Kp, "Kd : " + (String)Kd);
  }
}

void change_backward_pid() { //ui to change backward pid values
  stop();
  while (true) {
    int choice = get_pressed_button();
    update_btns();
    if (choice == 1) {
      Kpb--;
    } else if (choice == 2) {
      Kpb++;
    } else if (choice == 3) {
      Kdb--;
    } else if (choice == 4) {
      Kdb++;
    } else if (choice == 5 ) { //to get out of this you need to press 1 and 4 at the same time
      display_lcd("Kp back : " + (String)Kpb, "Kd back : " + (String)Kdb);
      update_btns();
      break;
    }
    display_lcd("Kp back : " + (String)Kpb, "Kd back : " + (String)Kdb);
  }
}

void change_det_jun() {
  stop();
  while (true) {
    int choice = get_pressed_button();
    update_btns();
    if (choice == 1) {
      waitTimeJunc -= 50;
    } else if (choice == 2) {
      waitTimeJunc += 50;
    } else if (choice == 5 ) { //to get out of this you need to press 1 and 4 at the same time
      display_lcd("detect time boxes" , (String)waitTimeJunc);
      update_btns();
      break;
    }
    display_lcd("detect time boxes" , (String)waitTimeJunc);
  }
}

void change_forward_linefollow_time() {
  stop();
  while (true) {
    int choice = get_pressed_button();
    update_btns();
    if (choice == 1) {
      waitTimeDist -= 50;
    } else if (choice == 2) {
      waitTimeDist += 50;
    } else if (choice == 5 ) { //to get out of this you need to press 1 and 4 at the same time
      display_lcd("dist time" , (String)waitTimeDist);
      update_btns();
      break;
    }
    display_lcd("dist time" , (String)waitTimeDist);
  }
}

void change_turn90_right_time() {
  stop();
  while (true) {
    int choice = get_pressed_button();
    update_btns();
    if (choice == 1) {
      dR90 -= 50;
    } else if (choice == 2) {
      dR90 += 50;
    } else if (choice == 5 ) { //to get out of this you need to press 1 and 4 at the same time
      display_lcd("TR90 time :" , (String)dR90);
      update_btns();
      break;
    }
    display_lcd("TR90 time :" , (String)dR90);
  }
}

void change_turn90_left_time() {
  stop();
  while (true) {
    int choice = get_pressed_button();
    update_btns();
    if (choice == 1) {
      dL90 -= 50;
    } else if (choice == 2) {
      dL90 += 50;
    } else if (choice == 5 ) { //to get out of this you need to press 1 and 4 at the same time
      display_lcd("TL90 time :" , (String)dL90);
      update_btns();
      break;
    }
    display_lcd("TL90 time :" , (String)dL90);
  }
}

void change_turn180_left_time() {
  stop();
  while (true) {
    int choice = get_pressed_button();
    update_btns();
    if (choice == 1) {
      dL180 -= 50;
    } else if (choice == 2) {
      dL180 += 50;
    } else if (choice == 5 ) { //to get out of this you need to press 1 and 4 at the same time
      display_lcd("TL180 time :" , (String)dL180);
      update_btns();
      break;
    }
    display_lcd("TL180 time :" , (String)dL180);
  }
}

void turn_left_until_middle()
{
  //-83, -51, -127, 13,-108, -94,   Encoder values
  //counter = 0;
  turn_left();
  do {
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
    read_ir();
  } while (!(IR_Bin_val[4] == 1 && IR_Bin_val[3] == 1));// changed for testig - added IR_Bin_val[3] == 0
  stop();
  delay(100);
}

void turn_right_until_middle()
{
  turn_right();
  do {
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
    read_ir();
  } while (!(IR_Bin_val[4] == 1 && IR_Bin_val[3] == 1));// changed for testig - added IR_Bin_val[3] == 0
  stop();
  delay(100);
}

void radio_send()
{
  int text = 0;
  for (int i = 0; i < 5; i++)
  {
    radio.write(&text, sizeof(text));
    delay(200);
  }
}

void start_command()
{
  if (stage==0){
    radio.startListening();
    int evans = 0;
    while (evans != 6)
    {
      if (radio.available())
        {
          radio.read(&evans, sizeof(evans));
          Serial.println(evans);
        }
        //delay(1000);
      }
    stage+=1;
  }
}

void _to_checkpoint1()
{
  if (stage == 1)
  {
    backward();
  Left_MotorBase_speed =167; 
  Right_MotorBase_speed =140;

    while (!(IR_Bin_val[0]==1 || IR_Bin_val[1]==1 || IR_Bin_val[2]==1 || IR_Bin_val[3]==1 || IR_Bin_val[4]==1 || IR_Bin_val[5]==1 || IR_Bin_val[6]==1 || IR_Bin_val[7]==1))
   {
      backward();
   }
   stop();
   delay(1000000000);
    read_ir();
      if ((IR_Bin_val[0] == 1 || IR_Bin_val[1] == 1 || IR_Bin_val[2] == 1 || IR_Bin_val[3] == 1 || IR_Bin_val[4] == 1 || IR_Bin_val[5] == 1 || IR_Bin_val[6] == 1 || IR_Bin_val[7] == 1))
      {
        for (int i = 0; i < 8; i++)
        {
          if (IR_Bin_val[i] == 1)
          {
            idx = i;
          }
        }

        if (idx < 3)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Line on right");
          Serial.println("Line on right");
          stop();
          delay(100);
          turn_right_until_middle();
          delay(100);
        }

        else if (idx > 3)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Line on left");
          Serial.println("Line of left");
          stop();
          delay(100);
          turn_left_until_middle();
          delay(100);
       }
      }
   }
    stop();
    delay(100);
    forward();
    delay(t);
    stop();
    delay(100);
    turn_left_180();

    for (int i=0;i<2;i++)
    {
      while (!(IR_Bin_val[0]==1 && IR_Bin_val[1]==1 && IR_Bin_val[2]==1 && IR_Bin_val[3]==1 && IR_Bin_val[4]==1 && IR_Bin_val[5]==1 && IR_Bin_val[6]==1 && IR_Bin_val[7]==1))
      {
        line_follow();
      }
      stop();
      delay(100);
      backward();
      delay(500);
    }
    
    while (!(IR_Bin_val[0]==1 && IR_Bin_val[1]==1 && IR_Bin_val[2]==1 && IR_Bin_val[3]==1 && IR_Bin_val[4]==1 && IR_Bin_val[5]==1 && IR_Bin_val[6]==1 && IR_Bin_val[7]==1))
      {
        line_follow();
      }
    forward();
    delay(t);
    
    while (IR_Bin_val[3]!=0)
    {
      turn_right();
    }

    while (!(IR_Bin_val[0]==0 && IR_Bin_val[1]==0 && IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0 && IR_Bin_val[6]==0 && IR_Bin_val[7]==0))
    {
      checkpoint_follow();
    }

    for (int i=0;i<2;i++){
      backward();
      delay(500);

      while (!(IR_Bin_val[0]==0 && IR_Bin_val[1]==0 && IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0 && IR_Bin_val[6]==0 && IR_Bin_val[7]==0))
      {
        checkpoint_follow();
      }

    }

    forward();
    delay(t);

    while(IR_Bin_val[3]==0)
    {
      turn_right();
    }

    for (int i=0;i<2;i++)
    {
      while (!(IR_Bin_val[0]==1 && IR_Bin_val[1]==1))
      {
        checkpoint_follow();
      }

      backward();
      delay(500);
    }

    while (!(IR_Bin_val[0]==1 && IR_Bin_val[1]==1))
    {
      checkpoint_follow();
    }

    forward();
    delay(t);

    turn_right_90();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Entered the small robot path");
    stage+=1;


}

void checkpoint_follow()
{
    read_ir();
    for (int i = 0; i < 8; i++)
        {
        if (i>4 || i==3)
        {
          error += IR_weights[i] * (!IR_Bin_val[i]);  //IR_Bin_val
        }
        else{
          error += IR_weights[i] * IR_Bin_val[i];
        }
        }

    P = error;
    I = I + error;
    D = error - previousError;

    previousError = error;

    speed_adjust = Kp * P + Ki * I + Kd * D;
    LMotorSpeed = Left_MotorBase_speed - speed_adjust;
    RMotorSpeed = Right_MotorBase_speed + speed_adjust;

    if (LMotorSpeed < min_speed)
        {
        LMotorSpeed = min_speed;
        }
    if (RMotorSpeed < min_speed)
        {
        RMotorSpeed = min_speed;
        }
    if (LMotorSpeed > max_speed)
        {
        LMotorSpeed = max_speed;
        }
    if (RMotorSpeed > max_speed)
        {
        RMotorSpeed = max_speed;
        }
    
    motor_speed();
}

void checkpoint_one_to_gate_A()
{
  if (stage == 2)
  {
    current_time = millis();
    measure_distance();
    delay(20);
    float x=measure_distance();
    set_forward();
    while (x > 8)
    {
      read_ir();
      if (IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1)
      {
        Serial.println("Ramp path detected");
        continue;
      }
      else {
        if ((millis() - current_time) > 1000) {
          Serial.println("Moving Forward");
          current_time = millis();
        }
        line_follow();
      }
      delay(20);
      x=measure_distance();

    }
    stop();
    delay(100);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("At gate A");
    stage += 1;
    }
}

void send_box_color()
{
    radio.stopListening();
    for (int i=0;i<=20;i++)
    {
       radio.write(&color, sizeof(color));
       delay(100);
    }
}

void gate_A_to_vault()
{
  measure_distance();
  delay(20);
  float x = measure_distance();
  if (stage == 3 && x > 8)
  { 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("slotted line");
    int idx;
    current_time = millis();
    bool temp = false;
    delay(20);
    x = measure_distance();
    read_ir();
    while (x > 2.7)
    {      if ((IR_Bin_val[0] == 1 || IR_Bin_val[1] == 1 || IR_Bin_val[2] == 1 || IR_Bin_val[3] == 1 || IR_Bin_val[4] == 1 || IR_Bin_val[5] == 1 || IR_Bin_val[6] == 1 || IR_Bin_val[7] == 1) && temp)
      {
        for (int i = 0; i < 8; i++)
        {
          if (IR_Bin_val[i] == 1)
          {
            idx = i;
          }
        }

        if (idx < 3)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Line on right");
          Serial.println("Line on right");
          stop();
          delay(100);
          turn_right_until_middle();
          delay(100);
        }

        else if (idx > 3)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Line on left");
          Serial.println("Line of left");
          stop();
          delay(100);
          turn_left_until_middle();
          delay(100);
        }
        temp = false;
      }
      else if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0) && !temp)
      {
        temp = true;
      }
      else
      {
        if ((millis() - current_time) > 1000) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Moving Forward");
          Serial.println("Moving Forward");
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        line_follow();
      }
      delay(10);
      x = measure_distance();
    }
    stop();
    delay(100);

    //might will have to call a forward function to get closer to the box
    //   for (int i=0;i<3;i++)
    //   {
    //     set_forward();
    //     x=measure_distance();
    //     while (x>2)
    //   {
    //     line_follow();
    //     delay(20);
    //   }
    //   stop();
    //   delay(100);
    //   while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
    //   {
    //     read_ir();
    //     backward();
    //   }
    // stop();
    // delay(100);
    //   }
    color = read_color_sensor();
    delay(100);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(color);
    send_box_color();
    //delay(100000000);
//    radio_send();
//    delay(100);
    turn_left_180();
    delay(100);
    stage += 1;
  }
}

void vault_to_ramp_junction()
{
  if (stage == 4)
  {
    int white_count = 0;
    int idx;
    current_time = millis();
    bool temp = false;
    read_ir();
  //slottedline following
    while (!(IR_Bin_val[6]==1 && IR_Bin_val[7]==1))//changed to 0 for testing// changed again for fixing faulty left turn detection
    {
      if ((IR_Bin_val[0] == 1 || IR_Bin_val[1] == 1 || IR_Bin_val[2] == 1 || IR_Bin_val[3] == 1 || IR_Bin_val[4] == 1 || IR_Bin_val[5] == 1 || IR_Bin_val[6] == 1 || IR_Bin_val[7] == 1) && temp)
      {
        for (int i = 0; i < 8; i++)
        {
          if (IR_Bin_val[i] == 1)
          {
            idx = i;
          }
        }

        if (idx < 3)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Line on right");
          Serial.println("Line on right");
          stop();
          delay(100);
          turn_right_until_middle();
          delay(100);
        }

        else if (idx > 3)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Line on left");
          Serial.println("Line of left");
          stop();
          delay(100);
          turn_left_until_middle();
          delay(100);
        }else{
          
        }
        temp = false;
      }
      else if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0) && !temp)
      {
        temp = true;
      }
      else
      {
        if ((millis() - current_time) > 1000) {
          // lcd.clear();
          // lcd.setCursor(0, 0);
          // lcd.print();
          Serial.println("Moving Forward");
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow();
      }
    }
  //turn left and continue
    stop();
    delay(100);
    forward();
    delay(t);
    stop();
    delay(100);
    turn_left_90();
    delay(100);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ramp junction");
    stage += 1;
  }
}

void ramp_junction_to_ramp_start()
{
  if (stage == 5)
  {
    measure_distance();
    delay(20);
    float x= measure_distance();
    set_forward();
    Left_MotorBase_speed = 255;  // higher speed for going up the hill
    Right_MotorBase_speed = 255;
    while (x > 8)
    {
      line_follow();
      delay(20);
      x=measure_distance();
    }
    stop();
    delay(100);
    pid_forward(500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Up the ramp");
    stage += 1;
  } 
}

void ramp_start_to_gate_B()
{
  if (stage == 6)
  {
    measure_distance();
    delay(20);
    //int idx;
    float x= measure_distance();
    set_forward();
    Left_MotorBase_speed = 255;  // higher speed for going up the hill
    Right_MotorBase_speed = 255;
    while (x > 8)
    {
      line_follow();
      delay(20);
      x=measure_distance();
    }
    stop();
    delay(100);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gate B");
    stage += 1;
  }


//  if (IR_Bin_val[0] == 1 || IR_Bin_val[1] == 1 || IR_Bin_val[2] == 1 || IR_Bin_val[3] == 1 || IR_Bin_val[4] == 1 || IR_Bin_val[5] == 1 || IR_Bin_val[6] == 1 || IR_Bin_val[7] == 1)
//       {
//         for (int i = 0; i < 8; i++)
//         {
//           if (IR_Bin_val[i] == 1)
//           {
//             idx = i;
//           }
//         }

//         if (idx < 3)
//         {
//           lcd.clear();
//           lcd.setCursor(0, 0);
//           lcd.print("Line on right");
//           Serial.println("Line on right");
//           stop();
//           delay(100);
//           turn_right_until_middle();
//           delay(100);
//         }

//         else if (idx > 3)
//         {
//           lcd.clear();
//           lcd.setCursor(0, 0);
//           lcd.print("Line on left");
//           Serial.println("Line of left");
//           stop();
//           delay(100);
//           turn_left_until_middle();
//           delay(100);
//         }
//       }

}

void ramp()
{
  measure_distance();
  delay(20);
  float x = measure_distance();
  if (stage == 5 && x > 20)
  {
    set_forward();
    Left_MotorBase_speed = 167;  // higher speed for going up the hill
    Right_MotorBase_speed = 150;
    
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("pid_forward");
      pid_forward(100);
      stop();
      delay(100);
      turn_left_180();
      delay(100);

      for (int i=0;i<3;i++)
      {

        line_follow();
        delay(500);
        stop();
        delay(100);

        backward();
        delay(500);
        stop();
        delay(100);}
    Left_MotorBase_speed = 60;  // higher speed for going up the hill
    Right_MotorBase_speed = 60;      
    backward();
    delay(1000);
    stage+=1;
    }
  }


void gate_B_to_ramp_end()  //hari
{
  measure_distance();
  delay(20);
  float x = measure_distance();
  if (stage == 7 && x > 8)
  {
    Left_MotorBase_speed = 80;  // lower speed for coming down the hill
    Right_MotorBase_speed = 80;
    while (!(IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1))
    {
      line_follow();
    }
    stop();
    delay(100);
    Left_MotorBase_speed = 167;
    Right_MotorBase_speed = 150;
    forward();
    delay(1.5*t);
    stop();
    delay(100);
    turn_right_until_middle();
    delay(100); 
    stop();
    delay(100);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ramp End");
    stage += 1;
  }
}

void ramp_end_to_T()  
{
  if (stage == 8)
  { read_ir();
  set_forward();
    while (!(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
    {
      line_follow();
    }
    stop();
    delay(100);
    turn_left_180();
    delay(100);
    stage += 1;
    // turn_left();
    // delay(200); // turn the robot 180 degrees
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T Junction");
  }
}

void T_to_ramp_end()
{
  if (stage == 9)
  {
    read_ir();
    set_forward();
    while (true)
    {
      if (!(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
      {
         line_follow();
      }
      else
      {
        stop();
        delay(100);
        forward();
        delay(0.8*t);
        stop();
        delay(100);
        turn_left_until_middle();
        delay(100);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Ramp End");
        forward();
        delay(t);
        turn_left_until_middle();
        stage += 1;
        break;
      }
    }
  }
}

void send_ok()
{
  if (stage==10)
    radio.stopListening();
    int ok=100;
    for (int i=0;i<=20;i++)
    {
       radio.write(&ok, sizeof(ok));
       delay(100);
    } 
}

void ramp_end_to_pole_junction()
{
  if (stage == 11)
  {
    //int temp=0;
    while (true)
    {
      read_ir();
      set_forward();
      if (!(IR_Bin_val[6]==1 && IR_Bin_val[7]==1)){
         line_follow(); 
      }
      // else if(temp==2){
      //   stage+=1;
      //   lcd.clear();
      //   lcd.setCursor(0, 0);
      //   lcd.print("Pole Junction");
      //   break;
      //}
      else{
        stop();
        delay(100);
        forward();
        delay(t);
        stop();
        delay(100);
        turn_left_90();
        delay(100);
        stage+=1;
        break;
      }
    }
  }
}

void pole_junction_to_pole()
{
  if (stage == 12)
  {
    measure_distance();
    delay(20);
    float x=measure_distance();
    while (x > 8 && !(IR_Bin_val[1]==1 && IR_Bin_val[2]==1 && IR_Bin_val[3]==1 && IR_Bin_val[4]==1 && IR_Bin_val[5]==1 && IR_Bin_val[6]==1))
    {
      set_forward();
      read_ir();
      if (IR_Bin_val[5]==1 && IR_Bin_val[6]==1 && IR_Bin_val[4]==0 && IR_Bin_val[3]==0)
      {
        turn_left_until_middle();
        delay(100);
      }
      else{
        line_follow();
      }
      x=measure_distance();
      delay(10);    //added to prevent overloading the sonar sensor
    }
    stop();
    delay(100);
    forward();
    delay(200); // give enough delay for the robot to go and touch the pole
    stop();
    delay(100);
    backward();
    delay(200);
    stop();
    delay(100);
    turn_left_180();
    delay(100); // give enough delay for the robot to turn 180 degrees
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pole");
    stage += 1;
  }
}

void pole_to_pole_junction()
{
  if (stage == 13)
  {
    while (!(IR_Bin_val[7]==1 && IR_Bin_val[6]==1))
    {
      set_forward();
      read_ir();
      if (IR_Bin_val[2]==1 && IR_Bin_val[1]==1 && IR_Bin_val[4]==0 && IR_Bin_val[3]==0)
      {
        turn_right_until_middle();
        delay(100);
      }
      else{
        line_follow();
      }
    }
    stop();
    delay(100);
    forward();
    delay(t);
    stop();
    delay(100);
    turn_left_90();
    delay(100);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pole Junction");
    stage+=1;
  }
}

void pole_junction_to_finish()
{
  if (stage == 14)
  {
    read_ir();
    set_forward();
    while (!(IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1 && IR_Bin_val[3] == 1 && IR_Bin_val[4] == 1 && IR_Bin_val[5] == 1 ))
    {
      line_follow();
    }
    forward();
    delay(500); // enough delay for the robot to go 10 cm
    stop();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Done !!");
    stage+=1;
  }
}