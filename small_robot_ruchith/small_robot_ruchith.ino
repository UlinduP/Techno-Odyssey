#include <NewPing.h>  // imports the NewPing library for ultrasonic sensor
#include <SPI.h>    // import the relevent library for SPI communication
// #include <nRF24L01.h>  // import the library for radio module
// #include <RF24.h>
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
#define trig 41
#define echo 40

// RGB sensor
#define S0 43
#define S1 42
#define S2 44
#define S3 45
#define sensorOut 46

NewPing ultrasonic(trig, echo); //defines a new ultrasonic variable


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
//RF24 radio(7, 8);  //defines a new radio variable CE, CSN respectively

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
#define bled 49
#define gled 51
#define rled 53

//keypad
#define key1 A10
#define key2 A11
#define key3 A8
#define key4 A9

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
int IR_val[8] = {0,0,0, 0, 0, 0, 0, 0};        // IR_Bin_val[0] - left side IR sensor  // IR_Bin_val[10] - right side IR sensor
int IR_Bin_val[8] = {0,0,0, 0, 0, 0, 0, 0};
double IR_weights[8] = {-4,-3, -2, 0, 0, 2, 3,4};//{-5000, -2000, -1000, 1000, 2000, 5000}; //{-4000, -2000, -1000, 1000, 2000, 4000};  //{-32, -16, -8, -4, -2, 0, 2, 4, 8, 16, 32}

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speed_adjust = 0;
int Left_MotorBase_speed = 167;  //60
int Right_MotorBase_speed = 150;
int max_speed = 255;
int min_speed = 50 ;

// for color sensor readings
int redfrequency = 0;
int greenfrequency = 0;
int bluefrequency = 0;

//front values
float Kp = 80;  //22,0 wada
float Kd = 50; // 8
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
float previousErrorb= 0; //46

//slow front pid


//slow back pid

//turn delays
int dR90 = 700;
int dL90 = 700;
int dL180 = 1400;



//detect junctions or boxes
int waitTimeJunc = 300; 

//go certain distance
int waitTimeDist = 1000;
int waitArr[3] = {500, 1000, 2000};//5cm, 10cm, 20cm

int stage = 0;
const byte address[6] = "00001"; //the address to which the module should listen for incoming messages

String text = "0",txt;// these can be used as global variables in any function
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
void stop();//checked
void motor_speed();//checked
void read_ir();//checked
void line_follow();//checked
int measure_distance();//checked
int read_color_sensor();//not giving consistant values
void send_color_value();//not checked

//not included in the code
void checkpoint_one_to_gate_A();
void gate_A_to_vault();
void vault_to_ramp_junction();
void ramp_start_to_gate_B();
void gate_B_to_ramp_end();
void ramp_end_to_cross();
void cross_to_obstacle_box_to_cross();
void cross_to_ramp_end();
void ramp_end_to_pole_junction();
void pole_junction_to_pole();
void pole_to_pole_junction();
void pole_junction_to_finish();

//functions--RuchchaSD
String read_Serial(); //get String sent through serial
String read_bluetooth(); //get String sent through bluetooth.
void horn(int dtime = 0); //to sound buzzer positive arg for delay 0 or no arg to keep the buzzer negative to turn off

void display_lcd(String fline = "",String sline = "",int dtime = 0); //shows whats been passed to upper and lowerlines. dtime is used to keep the line for a while and erase it
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

int t1 = 0,t2 = 0;
void setup() {
  //          Actuators // Set all the motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // Turn off motors - Initial state
  stop();
  
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
  digitalWrite(S0,LOW);
  digitalWrite(S1,HIGH);

  //          communication
  // bluetooth
  Serial.begin(9600);
  bluetoothSerial.begin(9600);

  // radio
  // radio.begin();
  // radio.openWritingPipe(address);
  // radio.setPALevel(RF24_PA_MAX);
  // radio.setDataRate(RF24_250KBPS);
  // radio.stopListening();

  //buzzer
  pinMode(buzzerd,OUTPUT);
  horn(-1);
  
  //led
  pinMode(rled,OUTPUT);
  pinMode(gled,OUTPUT);
  pinMode(bled,OUTPUT);
  show_color("off");
  
  // initialize the lcd
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.noCursor();

  // initialize the keypad
  pinMode(key1,INPUT_PULLUP);
  pinMode(key2,INPUT_PULLUP);
  pinMode(key3,INPUT_PULLUP);
  pinMode(key4,INPUT_PULLUP);

  
  display_lcd("TechBots", "Beetle");
  delay(1000);
  
  display_lcd("Kp : "+(String)Kp, "Kd : "+(String)Kd); //remove if you  can
  delay(1000);

  t1 = millis();
  state = "stop";  // change the starting state
}


void loop() {
// things to do in an every cycle
  update_btns();


// things to execute in a given time  interval
  if((t1 - millis()) > 1000){
     // add code
     t1 = millis();
  }

//things look for in every cycle 
// if commanded through bluetooth
  if (bluetoothSerial.available()) {
    bluetoothTxt = read_bluetooth();
    //execute the command
  }
  
// if commanded through serial monitor
  if (Serial.available()) {
    serialTxt = read_Serial();
    //execute the command
  }
  
//menu access long press 4 to access menu
  if(ks4 == true){
    stop();
    delay(500);
    update_btns();
    if(ks4 == true){
      for(int i =0; i < 3; i++){
        horn(200);
        delay(300);
      }
      menu();
    }else{
      horn(400); //use this to on or off something
      // set_forward();
    }
  }

// state manager
  if(state == "stop"){
    display_lcd("Stopped");
    stop();
  }else if(state == "front"){
    display_lcd("forward");
    set_forward();
  }else if (state == "back"){
    line_back();
  }else if(state == "forward"){
    line_front();
  }else if (state == "detect_junc"){
    jun_det_line_follow();
  }else if(state == "get distance"){
    mes_dis_travel();
  }else if (state == "turn 90"){
    det_jun_turn_90();
  }else if(state == "turn 180"){
    turn_180();
  }else if(state == "ultra"){
    //add code to run gate detection
  }
  
//add code to check without state manager remove comment from line below before that
  //state = "stop"
}

//bluetooth functions
String read_bluetooth(){
  String bTxt = "";
  char c = "";
    while(bluetoothSerial.available()){
      c = bluetoothSerial.read();
      bTxt += c;
    }
  return bTxt;
}
//Serial functions 
String read_Serial(){
  String sTxt = "";
  char c = "";
    while(Serial.available()){
      c = Serial.read();
      sTxt += c;
    }
    return sTxt;
}

//RGB led functions
void show_color(String color){
  if(color == "red"){
    color = "2";
  }else if(color == "green"){
    color = "6";
  }else if(color == "blue"){
    color = "5";
  }

  int q = (int)color[0] - 48;
  switch(q){
    case 8:
      digitalWrite(rled,HIGH);
      digitalWrite(gled,HIGH);
      digitalWrite(bled,HIGH);
      break;
    case 7:
      digitalWrite(rled,LOW);
      digitalWrite(gled,HIGH);
      digitalWrite(bled,HIGH);
      break;
      case 6:
      digitalWrite(rled,HIGH);
      digitalWrite(gled,LOW);
      digitalWrite(bled,HIGH);
      break;
    case 5:
      digitalWrite(rled,LOW);
      digitalWrite(gled,LOW);
      digitalWrite(bled,HIGH);
      break;
    case 4:
      digitalWrite(rled,HIGH);
      digitalWrite(gled,HIGH);
      digitalWrite(bled,LOW);
      break;
    case 3:
      digitalWrite(rled,LOW);
      digitalWrite(gled,HIGH);
      digitalWrite(bled,LOW);
      break;
      case 2:
      digitalWrite(rled,HIGH);
      digitalWrite(gled,LOW);
      digitalWrite(bled,LOW);
      break;
    default:
      digitalWrite(rled,LOW);
      digitalWrite(gled,LOW);
      digitalWrite(bled,LOW);
  }
}

// buzzer functions
void horn(int dtime){ //postive delay, negative off, no arg keep horn
  if(dtime == 0){
    digitalWrite(buzzerd,LOW);
  }else if(dtime > 0){
    digitalWrite(buzzerd,LOW);
    delay(dtime);
    digitalWrite(buzzerd,HIGH);
  }else{
    digitalWrite(buzzerd,HIGH);
  }
}

// display functions
void display_lcd(String fline,String sLine, int dtime){
  
  if(dtime == 0){
    
    if(fline != firstLine && sLine != lastLine){
      lcd.clear();
      display_line(fline, 0);
      firstLine = fline;
      display_line(sLine, 1);
      lastLine = sLine;
    }else if(fline != firstLine){
      display_line("                ", 0);
      display_line(fline, 0);
      firstLine = fline;
    }else if(sLine != lastLine){
      display_line("                ", 1);
      display_line(sLine, 1);
      lastLine = sLine;
    }

  }else if(dtime > 0){
    lcd.clear(); 
    display_line(fline, 0);
    display_line(sLine, 1);
    delay(dtime);
    lcd.clear();
    display_lcd(firstLine, lastLine);

  }else{
    firstLine = "";
    lastLine = "";
    lcd.clear(); 
  }
}

void display_line(String txt, int row){
  int start = (16 - txt.length())/2; 
  for(int i = start; i < txt.length() + start && i < 16 ; i++){
    lcd.setCursor(i,row);
    lcd.print(txt[i - start]);
  }
}

// ir sensors functions
void read_ir(){
       for (int i = 0; i < 8; i++)
    {
        IR_val[i]=0;
    }
       for (int i = 0; i <4 ; i++)
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
        IR_val[i]=IR_val[i]/4;
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

String look_for_junc(int wait){
  read_ir();
  bool right = IR_Bin_val[0] && IR_Bin_val[1] && IR_Bin_val[2] ,left = IR_Bin_val[7] && IR_Bin_val[6] && IR_Bin_val[5];
  if(right && left){
    pid_forward(wait);
    read_ir(); 
    line_follow();
    right = IR_Bin_val[0] && IR_Bin_val[1] && IR_Bin_val[2];
    left = IR_Bin_val[7] && IR_Bin_val[6] && IR_Bin_val[5];
    if(right && left){
      return "box";
    }else{
      return "4 way";
    }
  }else if(left){
    forward();
    delay(wait);
    read_ir();
    line_follow(); 
    left = IR_Bin_val[7] && IR_Bin_val[6] && IR_Bin_val[5];
    if(left){
      return "box left";
    }else{
      return "left";
    }
  }else if(right){
    forward();
    delay(wait);
    read_ir();
    line_follow();
    right = IR_Bin_val[0] && IR_Bin_val[1] && IR_Bin_val[2];
    if(left){
      return "box right";
    }else{
      return "right";
    }
  }else{
    return "no_junc";
  }

}

// ultrasonic sensor functions
int measure_distance(){
    int duration = ultrasonic.ping_median(); // sends out 5 pulses and takes the median duration
    int distance = ultrasonic.convert_in(duration)*2.54;  // distance in centimeters
    return distance;
}

// color sensor functions
int read_color_sensor(){
  int i=0;
  int R_val=0,G_val=0,B_val=0,O_val =0;
  do{ 
    // looking for red value
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    redfrequency = pulseIn(sensorOut, LOW);
     
    // looking for green value
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    greenfrequency = pulseIn(sensorOut, LOW);

    // looking for blue value
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    bluefrequency = pulseIn(sensorOut, LOW);
  
    if (redfrequency>250 && redfrequency <289) R_val++;
    else if (greenfrequency>260 && greenfrequency <300) G_val++;
    else if (bluefrequency>310 && bluefrequency <350) B_val++;
    else O_val++;
    i++;
  }
  while (i<50);

    if(R_val>40) return 1;
    else if (G_val>40) return 2;
    else if (B_val>40) return 3;
    else return 4;
}


//keypad functions
bool update_btns(){
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
  if(digitalRead(key1) == LOW){
    tp1 = true;
    pressed = true;
  }
  if(digitalRead(key2) == LOW){
    tp2 = true;
    pressed = true;
  }
  if(digitalRead(key3) == LOW){
    tp3 = true;
    pressed = true;
  }
  if(digitalRead(key4) == LOW){
    tp4 = true;
    pressed = true;
  }
  return pressed;
}

//state manager functions
//state = back
void line_back(){
  display_lcd("backward linefollow","Kpb: "+(String)((int)Kpb) + " Kdb: "+(String)((int)Kdb) );
  set_backward();
    line_follow_back();
    if(ks3 == true){ // to change pid  
      change_backward_pid();
    }  
}
//state = forward
void line_front(){
  display_lcd("Line Forward", "Kp:"+(String)(int)Kp + " Kd:"+(String)(int)Kd);
  //basic line following and pid value tuning
    set_forward();
    line_follow();

    if(ks3 == true){ // to change pid  
      change_forward_pid();
    }
}
//state = det_junc
void jun_det_line_follow(){
  // display_lcd("forward linefollow","Kp: "+(String)(int)Kp);
  // horn(300);
    // delay(500);
    set_forward();
    line_follow();
    String jType= look_for_junc(waitTimeJunc);//detects four types of junctions
    display_lcd("Find Junctions",jType);
    line_follow();
    if(jType == "right"){
      display_lcd("right");
      horn(0);
      pid_forward(250);
      horn(-1);
      line_follow();
    }else if(jType == "left"){
      display_lcd("left");
      for(int i= 0;i <3; i++  ){
        horn(0);
        pid_forward(250);
        horn(-1);
        pid_forward(250);
      }
      line_follow();
    }else if(jType == "4way"){
      display_lcd("4way");
      for(int i= 0;i <2; i++  ){
        horn(0);
        pid_forward(250);
        horn(-1);
        pid_forward(250);
      }
    }else if(jType == "box" || jType == "box left" || jType == "box right"){
      display_lcd(jType);
      stop();
      delay(2000);
      // state = "stop";
      text = "found : " + jType;
      for(char c : text){
      bluetoothSerial.write(c);
      }
      bluetoothSerial.write("\n");
      for(int i= 0;i < 5; i++  ){
        horn(100);
        delay(300); 
      }
    }

    if(ks3 == true){ // to change pid  
      change_det_jun();//time to wait until cheking again 
    }
}
//state = get distance
void mes_dis_travel(){
  display_lcd("measure distance", (String)(int)waitTimeDist);
  set_forward();
    line_follow();
    if(look_for_junc(waitTimeJunc) == "4way"){
      pid_forward(waitTimeDist);
  }

  if(ks3 == true){ // to change pid  
    change_forward_linefollow_time();//time to go some distance  5cm 10cm 20cm
    // changebpidtime();
    // changespidtime(); 
  }
}

//state turn 90
void det_jun_turn_90(){
  display_lcd("Turn 90 Right","TR:" + (String)(int)dR90 + " TL:" + (String)(int)dL90);
  set_forward();
    line_follow();
    String jType = look_for_junc(waitTimeJunc);
    if(jType == "left"){
      stop();
      horn(300);
      turn_left_90();
    }else if(jType == "right"){
      stop();
      horn(300);
      delay(500);
      horn(300);
      turn_right_90();      
    }

    if(ks3 == true){ // to turning time  
      change_turn90_right_time();
    }else if(ks1 == true){
      change_turn90_left_time();
    }

}
//state = turn 180
void turn_180(){
  display_lcd("Turn 180 left","TL180 : " + (String)(int)dL180);
  if(ks1 == true ){
    set_forward();
    pid_forward(2000);
    turn_left_180();  
  }else if(ks3 == true){
      change_turn180_left_time();
  }
}


// motors 
void motor_speed(){
    analogWrite(ENB, LMotorSpeed);
    analogWrite(ENA, RMotorSpeed);
}

void set_forward(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);  
}

void set_backward(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);   
}

void forward(int Left, int Right){ // left right speeds
    Left_MotorBase_speed = Left;
    Right_MotorBase_speed = Right;

    set_forward();

    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
}

void backward(int Left, int Right){
    
    Left_MotorBase_speed = Left;
    Right_MotorBase_speed = Right;

    set_backward();

    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    
    motor_speed();
}

void pid_forward(int count) {
    I = 0;
    set_forward();
    line_follow();
    if(count>0){
      for (int i = 0; i < count; i++) {
        line_follow();
        delay(2);
      stop();
    }
    
  }
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

void turn_left(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed){
    digitalWrite(IN1, LOW);//left front
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);//right back
    digitalWrite(IN4, HIGH);
    LMotorSpeed = left; // may need to change this
    RMotorSpeed = right;
    motor_speed();
}

void turn_right(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed){
    digitalWrite(IN1, HIGH);// left motor back
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);// right motor front
    digitalWrite(IN4, LOW);
    LMotorSpeed = left; // may need to change this
    RMotorSpeed = right;
    motor_speed();
}

void turn_left_90(int dTime){
    turn_left();
    delay(dTime);  // will have to change the delay
    stop();
    turn_left();//pass different values if necessary

    do {
        read_ir();
    } while ((IR_val[3] == 1 && IR_val[4] == 1)); // && IR_val[0] == 0 && IR_val[1] == 0 && IR_val[6] == 0 && IR_val[7] == 0);
        stop();
}

void turn_right_90(int dTime){
    turn_right();
    delay(dTime);
    stop();
    turn_right();//pass different values if necessary
    do {
        read_ir();
    } while ((IR_val[3] == 1 && IR_val[4] == 1));
    stop();
}

void turn_left_180(int dTime){
    turn_left();
    delay(1000);  // will have to change the delay
    stop();
    turn_left();//pass different values if necessary

    do {
        read_ir();
    } while ((IR_val[3] == 1 || IR_val[4] == 1)  && IR_val[0] == 0 && IR_val[1] == 0 && IR_val[6] == 0 && IR_val[7] == 0);
    stop(); 
}

void stop(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// algorithms
void line_follow(){
    error = 0;
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
    RMotorSpeed = Right_MotorBase_speed +0.9 * speed_adjust;

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

void line_follow_back(){
    errorb = 0;
    read_ir();
    for (int i = 0; i < 8; i++)
        {
        errorb += IR_weights[i] * IR_Bin_val[i];  //IR_Bin_val
        }

    Pb = errorb;
    Ib = I + errorb;
    Db = errorb - previousErrorb;

    previousErrorb = errorb;

    speed_adjust = Kpb * Pb + Kib * Ib + Kdb* Db;
    LMotorSpeed = Left_MotorBase_speed + speed_adjust;
    RMotorSpeed = Right_MotorBase_speed - 0.9 * speed_adjust;

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
void menu(){
  while(update_btns()){
    delay(2);
  }


  display_lcd("Menu", "for bac jun ult ");
  int choice = get_pressed_button();
  
  if(choice == 1){
    state = "forward";
    display_lcd("Line Following", "Forward");
    delay(1000);
 
  }else if(choice == 2){
    state = "back";
    display_lcd("Line Following", "backward");
    delay(1000);
 
  }else if(choice == 3){
    
    display_lcd("Junction train", "get dis 90 180 ");
    choice = get_pressed_button();
    switch (choice){
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

    display_lcd("State",state);
    
  }else if(choice == 4){
    state = "ultra";
    display_lcd("gate training",state);
  }
}

int get_pressed_button(){
  while(!(update_btns())){//this is to wait until a button is pressed
    delay(5);
    
  }

  while((update_btns())){//this is to wait until the button is released
    delay(5);
    if(ks1 == true && ks4 == true){
      display_lcd("State 5", "Entered");
      delay((1000));
      return 5;
    }
  }

  if(ks1 == true){
    return 1;
  }else if(ks2 == true ){
    return 2;
  }else if(ks3 == true ){
    return 3;
  }else{
    return 4;
  }

}

void change_forward_pid(){//ui to change forward pid values
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      Kp--;
    }else if(choice==2){
      Kp++;
    }else if(choice == 3){
      Kd--;
    }else if(choice==4){
      Kd++;
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("Kp : "+(String)Kp, "Kd : "+(String)Kd);
      update_btns();
      break;
    }
    display_lcd("Kp : "+(String)Kp, "Kd : "+(String)Kd);
  }
}

void change_backward_pid(){//ui to change backward pid values
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      Kpb--;
    }else if(choice==2){
      Kpb++;
    }else if(choice == 3){
      Kdb--;
    }else if(choice==4){
      Kdb++;
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("Kp back : "+(String)Kpb, "Kd back : "+(String)Kdb);
      update_btns();
      break;
    }
    display_lcd("Kp back : "+(String)Kpb, "Kd back : "+(String)Kdb);
  }
}

void change_det_jun(){
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      waitTimeJunc -= 50;
    }else if(choice==2){
      waitTimeJunc += 50;
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("detect time boxes" ,(String)waitTimeJunc);
      update_btns();
      break;
    }
    display_lcd("detect time boxes" ,(String)waitTimeJunc);
  }
}

void change_forward_linefollow_time(){
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      waitTimeDist -= 50;
    }else if(choice==2){
      waitTimeDist += 50;
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("dist time" ,(String)waitTimeDist);
      update_btns();
      break;
    }
    display_lcd("dist time" ,(String)waitTimeDist);
  }
}

void change_turn90_right_time(){
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      dR90 -= 50;
    }else if(choice == 2){
      dR90 += 50;
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("TR90 time :" ,(String)dR90);
      update_btns();
      break;
    }
    display_lcd("TR90 time :" ,(String)dR90);
  }
}

void change_turn90_left_time(){
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      dL90 -= 50;
    }else if(choice == 2){
      dL90 += 50;
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("TL90 time :" ,(String)dL90);
      update_btns();
      break;
    }
    display_lcd("TL90 time :" ,(String)dL90);
  }
}

void change_turn180_left_time(){
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      dL180 -= 50;
    }else if(choice == 2){
      dL180 += 50;
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("TL180 time :" ,(String)dL180);
      update_btns();
      break;
    }
    display_lcd("TL180 time :" ,(String)dL180);
  }
}



// void back_pid(int steps)
// {

// }