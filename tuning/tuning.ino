#include <NewPing.h>  // import the NewPing library for ultrasonic sensor
//#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>    // import the relevent library for SPI communication
#include <nRF24L01.h>  // import the library for radio module
#include <RF24.h>
#include <Servo.h>

// sensors
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5
#define IR7 A6
#define IR8 A7

//buzzer
#define buzzerd 4

//RGB
#define pin_red 5
#define pin_green 6
#define pin_blue 7

//Sonar
#define trig 31
#define echo 32

//RGB
#define S0 26
#define S1 27
#define S2 28
#define S3 29
#define sensorOut 30

//keypad
#define key1 A9
#define key2 A8
#define key3 A11
#define key4 A10   //key 4 done


bool ks1 = false;
bool ks2 = false;
bool ks3 = false;
bool ks4 = false;

//this stores previous values got from the keystroke
bool tp1 = false;
bool tp2 = false;
bool tp3 = false;
bool tp4 = false;

//Encoder
// #define ENCA_left 53
// #define ENCB_left 51

NewPing ultrasonic(trig, echo); //defines a new ultrasonic variable
//LiquidCrystal_I2C lcd(0x3F, 16, 2); // I2C address 0x27, 16 column and 2 rows
Adafruit_SSD1306 oled = Adafruit_SSD1306(128,64,&Wire,-1);
RF24 radio(8, 9);  //defines a new radio variable CE, CSN respectively
Servo arm_servo;
Servo gate_servo;

// Servo connections
#define arm_servo_pin 10
#define gate_servo_pin 11

// Right Motor connections
#define EN_right 3
#define IN4 24
#define IN3 25
// Left Motor connections
#define EN_left 2
#define IN2 23
#define IN1 22

String firstLine = "";
String lastLine = "";

//const int buttonPin[] = {31, 32, 33, 34};     // the number of the pushbutton pins


int Threshold = 150;
int IR_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};        // IR_Bin_val[0] - left side IR sensor  // IR_Bin_val[10] - right side IR sensor
int IR_Bin_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double IR_weights[8] = {-16,-6,-2,-1,1,2,6,16};//{-16, -8, -4, -2, 2, 4, 8, 16}; //{-4, -3, -2, 0, 0, 2, 3, 4}  {-8, -4, -2, 0, 0, 2, 4, 8}

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speed_adjust = 0;
int Left_MotorBase_speed = 85; // base 110
int Right_MotorBase_speed = 105;   // limit 80   // base 90
int max_speed = 255;
int min_speed = 30;

// color sensor readings
int redfrequency = 0;
int greenfrequency = 0;
int bluefrequency = 0;

float Kp = 22.4; //0.05 worked   //0.3 // 3.05, 15, 0.001 for 43 max    // 1.875*0.3=0.5625 // 1.875 is what you need to utilize the full range  //12*0.25
float Kd = 100; //18 worked   // 85
float Ki = 0;

float P, D;
float I = 0;   // I is again and again set to zero in pid_forward()

int ir_to_wheels_dist = 50;  // encoder value 

float error = 0;
float previousError = 0; //46

int counter = 0; 
int aState;
int aLastState; 

int stage = 0;
const byte address[6] = "00001"; //the address to which the module should listen for incoming messages

int color = -1;

long current_time;


//turn delays
int dR90 = 700;
int dL90 = 700;
int dL180 = 1400;

String state = "stop";

void set_rgb_color();
void set_forward();
void forward();  // sets forward and go forward
void pid_forward(int steps);  // go forward with pid output for a given number of steps
void set_backword();
void backword();
void pid_backward(int steps);
void turn_left();
void turn_right(); // after turn left or turn right functions you need to specify the delay and then stop
void turn_left_90();
void turn_right_90();
void turn_right_90_using_delay();
void turn_left_180();
void turn_left_90_using_delay();
void stop();
void motor_speed();
void read_ir();
void display_ir();
void read_encoders();
void line_follow();
int measure_distance();
int read_color_sensor();
int read_received_color_value();
void start_to_checkpoint1();
void checkpoint1();
void checkpoint1_to_();
void _to_dotted_line();
void left_to_right();
void right_to_left();
void dotted_line_to_checkpoint2();
void read_received_box_color();
void checkpoint2_to_L_junction();
void L_junction_to_box_pickup();
void box_pickup_to_unload();
//void unload_to_T_junction();
void unload_to_finish();
void checkpoint_follow();
void lower_gate();
void arm_down();
void arm_up();
bool update_btns();
void display_lcd(String fline = "",String sline = "",int dtime = 0);
void display_line(String txt, int row = 0); // used to display a line in a row
int get_pressed_button();
void menu(); //main menu can be accesed by long pressing 4 when inside of the loop
void change_forward_linefollow_time();//ui to change the pid forward time (this is used to measure distances)
void change_det_jun();//ui to change junction and box differentiation time
void change_backward_pid();//ui to change back pid values
void change_forward_pid();//ui to change forward pid values
void change_turn90_right_time();
void change_turn90_left_time();
void change_turn180_left_time();
void change_speed();
void horn(int dtime);


void setup()
{
  Serial.begin(9600);

  // initialize the lcd
  // lcd.init();
  // lcd.backlight();

    //initialize the oled
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(0,0);

    //initialize servos
  gate_servo.attach(gate_servo_pin);
  arm_servo.attach(arm_servo_pin);
  arm_servo.write(180);


  // Set all the motor control pins to outputs
  pinMode(EN_right, OUTPUT);
  pinMode(EN_left, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //buzzer
  //pinMode(buzzerd, OUTPUT);
  //horn(-1);

  // initialize the keypad
  pinMode(key1,INPUT_PULLUP);
  pinMode(key2,INPUT_PULLUP);
  pinMode(key3,INPUT_PULLUP);
  pinMode(key4,INPUT_PULLUP);

  //RGB
  pinMode(pin_red, OUTPUT);
  pinMode(pin_green, OUTPUT);
  pinMode(pin_blue, OUTPUT);

  // Encoders
//  pinMode(ENCA_left, INPUT);
//  pinMode(ENCB_left, INPUT);

  // setting up the sonar sensor
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  // Set all the color sensor pin configurations
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Setting its frequency-scaling to 20%
  digitalWrite(S0,LOW);
  digitalWrite(S1,HIGH);
  
  //Setup the radio module
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  // Turn off motors - Initial state
    //stop();

  // Reads the initial state of the outputA
//  aLastState = digitalRead(ENCA_left);

  set_forward();
  current_time = millis();
}

void loop()
{
  update_btns();

 if((millis()-current_time)>1000){
   oled.clearDisplay();
   oled.setCursor(0,0);
   read_ir();
   display_ir();
   current_time = millis();
 } 

  //menu access long press 4 to access menu
 if(ks4 == true){
   stop();
   delay(500);
   update_btns();
   if(ks4 == true){
     for(int i =0; i < 3; i++){
       //horn(200);
       delay(300);
     }
     menu();
    }//else{
    //   horn(400); //use this to on or off something
    //   set_forward();
    //   continue;
    // }
 }

//state manager
 if(state == "stop"){
   display_lcd("Stopped");
   stop();
 }else if(state == "line follow"){  //"kpkd"
   display_lcd("line follow");
   set_forward();
   line_follow();
   
 }else if (state == "PID"){
   change_forward_pid();
    //line_follow();
 }else if(state == "change speed"){
       change_speed();
   }//else if (state == "detect_junc"){
  //   jun_det_line_follow();
  // }else if(state == "get distance"){
  //   mes_dis_travel();
  // }else if (state == "turn 90"){
  //   det_jun_turn_90();
  // }else if(state == "turn 180"){
  //   turn_180();
  // }else if(state == "ultra"){
  //   //add code to run gate detection
  // }
  
}

void display_lcd(String fline,String sLine, int dtime){
  
  if(dtime == 0){
    
    if(fline != firstLine && sLine != lastLine){
      oled.clearDisplay();
      display_line(fline, 0);
      firstLine = fline;
      display_line(sLine, 16);
      lastLine = sLine;
    }else if(fline != firstLine){
      display_line("                ", 0);
      display_line(fline, 16);
      firstLine = fline;
    }else if(sLine != lastLine){
      display_line("                ", 0);
      display_line(sLine, 16);
      lastLine = sLine;
    }

  }else if(dtime > 0){
    oled.clearDisplay(); 
    display_line(fline, 0);
    display_line(sLine, 16);
    delay(dtime);
    oled.clearDisplay();
    display_lcd(firstLine, lastLine);

  }else{
    firstLine = "";
    lastLine = "";
    oled.clearDisplay(); 
  }
}

void display_line(String txt, int row){
  oled.setCursor(0,row);
  oled.print(txt);
  oled.display();
//  int start = (16 - txt.length())/2; 
//  for(int i = start; i < txt.length() + start && i < 16 ; i++){
//    oled.setCursor(i,row);
//    oled.print(txt[i - start]);
//    oled.display();
  //}
}

void menu(){
  while(update_btns()){
    delay(2);
  }


  display_lcd("Menu", "stop line follow PID change ");
  int choice = get_pressed_button();
  
  if(choice == 1){
    state = "stop";
    display_lcd("stop");
    delay(1000);
 
  }else if(choice == 2){
    state = "line follow";
    display_lcd("line follow");
    delay(1000);
 
  }else if(choice == 3){
    state="PID";
    display_lcd("PID");
    delay(1000);
//    choice = get_pressed_button();
//    switch (choice){
//      case 1:
//        state = "detect_junc";
//        break;
//      case 2:
//        state = "get distance";
//        break;
//      case 3:
//        state = "turn 90";
//        break;
//      case 4:
//        state = "turn 180";
//        break;
 //   }

    //display_lcd("State",state);
    
  }else if(choice == 4){
    state = "change speed";
    display_lcd("change speed");
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

void change_speed()
{
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      Left_MotorBase_speed--;
      oled.clearDisplay();
    }else if(choice==2){
      Left_MotorBase_speed++;
      oled.clearDisplay();
    }else if(choice == 3){
      Right_MotorBase_speed--;
      oled.clearDisplay();
    }else if(choice==4){
      Right_MotorBase_speed++;
      oled.clearDisplay();
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("Left : "+(String)Left_MotorBase_speed, "Right : "+(String)Right_MotorBase_speed);
      update_btns();
      break;
    }
    display_lcd("Left : "+(String)Left_MotorBase_speed, "Right : "+(String)Right_MotorBase_speed);
  } 
}

void change_forward_pid(){//ui to change forward pid values
  stop();    
  while(true){
    int choice = get_pressed_button();
    update_btns();
    if(choice == 1){
      oled.clearDisplay();
      Kp--;
    }else if(choice==2){
      oled.clearDisplay();
      Kp++;
    }else if(choice == 3){
      oled.clearDisplay();
      Kd--;
    }else if(choice==4){
      oled.clearDisplay();
      Kd++;
    }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
      display_lcd("Kp : "+(String)Kp, "Kd : "+(String)Kd);
      update_btns();
      break;
    }
    display_lcd("Kp : "+(String)Kp, "Kd : "+(String)Kd);
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














void set_rgb_color(int r, int g, int b)
{
  analogWrite(pin_red,   r);
  analogWrite(pin_green, g);
  analogWrite(pin_blue,  b);
}

void set_forward()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);   
}

void forward()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
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

void set_backword()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void backword()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
}

void pid_backword(int count) {
    I = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    for (int i = 0; i < count; i++) {
        read_ir();
        line_follow();
        delay(2);
    }
    stop();
}

void turn_left()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
}

void turn_right()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
}

void turn_left_90()
{
  //-83, -51, -127, 13,-108, -94,   Encoder values
  //counter = 0;
  turn_left();
  delay(500);  // will have to change the delay
  //stop();
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  do {
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
    read_ir();
  } while (!(IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0));
      stop();
      delay(50);
}

void turn_left_until_middle()
{
  //-83, -51, -127, 13,-108, -94,   Encoder values
  //counter = 0;
  turn_left();
  //delay(1000);  // will have to change the delay
  //stop();
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  do {
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
    read_ir();
  } while (!(IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0));
      stop();
      delay(50);
}

void turn_right_90()
{
    turn_right();
    delay(500);
//    stop();
//    digitalWrite(IN1, HIGH);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, HIGH);
    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (!(IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0));
    stop();
    delay(50);
}

void turn_right_until_middle()
{
    turn_right();
//  delay(1000);
//    stop();
//    digitalWrite(IN1, HIGH);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, HIGH);
    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (!(IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0));
    stop();
    delay(50);
}

void turn_right_90_using_delay()
{
    turn_right();
    delay(705); // change the delay to change the amount turned
    stop();
    delay(50);
}

void turn_left_180()
{
    turn_left();
    delay(1000);  // will have to change the delay
//    stop();
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, HIGH);
//    digitalWrite(IN3, HIGH);
//    digitalWrite(IN4, LOW);
    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (!(IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0));
        stop();   
        delay(50);
}

void turn_left_90_using_delay()
{
    turn_left();
    delay(710); // change the delay to change the amount turned
    stop();
    delay(50);
}

void stop()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void motor_speed()
{
    
    /*Serial.print("Left Speed: ");
    Serial.println(LMotorSpeed);

    Serial.print("Right Speed: ");
    Serial.println(RMotorSpeed); */
    
    analogWrite(EN_left, LMotorSpeed);
    analogWrite(EN_right, RMotorSpeed);
}

void read_ir()
{
//   IR_val[0] = analogRead(IR1);
//   IR_val[1] = analogRead(IR2);
//   IR_val[2] = analogRead(IR3);
//   IR_val[3] = analogRead(IR4);
//   IR_val[4] = analogRead(IR5);
//   IR_val[5] = analogRead(IR6);
// //
//   for (int i = 0; i < 6; i++)
//    {
//      Serial.print(IR_val[i]);
//      Serial.print(" ");
//    }
//    Serial.println(" ");

       for (int i = 0; i < 8; i++)
    {
        IR_val[i]=0;
    }
       for (int i = 0; i <3 ; i++)
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
        IR_val[i]=IR_val[i]/3;
    }

    for (int i = 0; i < 8; i++)
    {
      if (IR_val[i] >= Threshold)
      {
        IR_Bin_val[i] = 1;       //change for white strips on black surface
      }
      else
      {
        IR_Bin_val[i] = 0;
      }
    }

    /*  for (int i = 0; i < 8; i++)
    {
      Serial.print(IR_val[i]);
      Serial.print(" ");
    }
    Serial.println(" "); */
//    lcd.clear(); 
//    lcd.setCursor(0, 0);  
//    for (int i = 0; i < 4; i++)
//    {     
//      lcd.print(IR_val[i]);
//      lcd.print(" ");   
//    }
//    lcd.setCursor(0, 1);
//    for (int i = 4; i < 8; i++)
//    {     
//      lcd.print(IR_val[i]);
//      lcd.print(" ");   
//    }


}

void display_ir()
{
    oled.clearDisplay(); 
    oled.setCursor(0, 0);  
    for (int i = 0; i < 4; i++)
    {     
      oled.print(IR_val[i]);
      oled.print(" ");   
    }
    oled.setCursor(0, 16);
    for (int i = 4; i < 8; i++)
    {     
      oled.print(IR_val[i]);
      oled.print(" ");   
    }  
    oled.display();
}

// void read_encoders()
// {
//   aState = digitalRead(ENCA_left); // Reads the "current" state of the outputA
//    // If the previous and the current state of the outputA are different, that means a Pulse has occured
//   if (aState != aLastState){     
//      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//     if (digitalRead(ENCB_left) != aState) { 
//       counter ++;
//     } else {
//       counter --;
//     }
// //    Serial.print("Position: ");
// //    Serial.println(counter);
//   } 
//   aLastState = aState; // Updates the previous state of the outputA with the current state
//   //return counter;
// }

void line_follow()
{
    read_ir();
    error=0;
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

int measure_distance()
{
    int duration = ultrasonic.ping_median(); // sends out 5 pulses and takes the median duration
    int distance = ultrasonic.convert_in(duration)*2.54;  // distance in centimeters
    return distance;
}

int read_color_sensor()
{
  int i=0;
  int R_val=0,G_val=0,B_val=0,O_val=0,W_val =0;
  do
  {
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    redfrequency = pulseIn(sensorOut, LOW);
     
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    greenfrequency = pulseIn(sensorOut, LOW);
  
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    bluefrequency = pulseIn(sensorOut, LOW);
  
    if ((redfrequency>30 && redfrequency <95) && (bluefrequency>60 && bluefrequency<123) && (greenfrequency>80 && greenfrequency<160)) R_val++;
    else if ((redfrequency>110 && redfrequency <190) && (bluefrequency>80 && bluefrequency<165) && (greenfrequency>70 && greenfrequency<180)) G_val++;
    else if ((redfrequency>90 && redfrequency <115) && (bluefrequency>35 && bluefrequency<95) && (greenfrequency>70 && greenfrequency<130)) B_val++;
    else if (bluefrequency<100 && redfrequency<100 && greenfrequency<100) W_val++;
    else O_val++;
    i++;
  }
  while (i<50);

    if(R_val>40) return 1;
    else if (G_val>40) return 2;
    else if (B_val>40) return 3;
    else if (W_val>40) return 4;
    else return 5;
}

int read_received_color_value()
{
  int R_val=0,G_val=0,B_val=0,O_val=0;
  for (int i=0;i<5;i++)
  {
     if (radio.available())
      {
        int text;
        radio.read(&text, sizeof(text));
        if (text==1) R_val++;
        else if (text==2) G_val++;
        else if (text==3) B_val++;
        else O_val++; 
      }
      delay(200);
  }

  if (R_val>=3) return 1;
  else if (G_val>=3) return 2;
  else if (B_val>=3) return 3;
  else if (R_val==0 && G_val==0 && B_val==0 && O_val==0) return -1;
  else return 4;
}

void start_to_checkpoint1()
{
  if (stage==0){
    current_time = millis();
    while (true) {
      read_ir();
      if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0) && (IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        //Serial.println("We are at a left Junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);  // will have to change the # steps
        oled.setCursor(0, 16);
        oled.print("Turning left");
        oled.display();
        //Serial.println("Turning left");
        stop();
        delay(50);
        turn_left_90();
      } 
      else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        //Serial.println("We are at a right junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);
        stop();
        delay(50);
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        //Serial.println("Turning right");
        turn_right_90();
      }
      else if(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0){
        delay(50);
        pid_forward(ir_to_wheels_dist);
        if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0){
          oled.clearDisplay();
          oled.setCursor(0, 0);
          oled.print("Stage 1 completed!");
          oled.display();
          stage+=1;
          break;
        }
        else{
          oled.clearDisplay();
          oled.setCursor(0, 0);
          oled.print("Turning left");
          oled.display();
          turn_left_90();
        }
      }
      else if (IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1 && IR_Bin_val[3] == 1 && IR_Bin_val[4] == 1 && IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Dead end");
        oled.display();
        //Serial.println("We are at a dead end");
        stop();
        delay(50);
        turn_left_180();
      } 
      else {
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          oled.display();
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow();
        //pid_forward(15);
      }
    }
  }
}


/*
void start_to_checkpoint1()
{
  if (stage==0){
    bool check = false;
    while (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0 && check) {
      read_ir();
      if ((IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0) && (IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1)) {
        check = true;
        lcd.clear(); 
        lcd.setCursor(0, 0);
        lcd.print("Left Junction");
        //Serial.println("We are at a left Junction");
        pid_forward(100);  // will have to change the # steps
        lcd.clear(); 
        lcd.setCursor(0, 0);
        lcd.print("Turning left");
        //Serial.println("Turning left");
        turn_left_90();
      } 
      else if ((IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1) && (IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0)){
        lcd.clear(); 
        lcd.setCursor(0, 0);
        lcd.print("Right Junction");
        //Serial.println("We are at a right junction");
        pid_forward(100);
        lcd.clear(); 
        lcd.setCursor(0, 0);
        lcd.print("Turning right");
        //Serial.println("Turning right");
        turn_left_90();
      }
      else if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1){
        lcd.clear(); 
        lcd.setCursor(0, 0);
        lcd.print("Dead end");
        //Serial.println("We are at a dead end");
        turn_left_180();
      } else {
        lcd.clear(); 
        lcd.setCursor(0, 0);
        lcd.print("Moving Forward");
        //Serial.println("Moving Forward");
        line_follow();
        //pid_forward(15);
      }
    }
  }
  else{
    stage += 1;
  }
}
*/

void checkpoint1(){
  if (stage == 1)
  {
    pid_forward(ir_to_wheels_dist);
    turn_right_90();
    while (!(IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1))
    {
      checkpoint_follow();
    }
    pid_forward(ir_to_wheels_dist);
    turn_left_90();
    while (!(IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
    {
      checkpoint_follow();
    }
    pid_forward(ir_to_wheels_dist);
    turn_left_90();
    pid_backward(3*ir_to_wheels_dist);
    //lower_gate();
    //send_bluetooth_value(1);
    //wait untill the smaller robot gets off

    pid_forward(2*ir_to_wheels_dist);
    turn_right_90();

    while (!(IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1))
    {
      checkpoint_follow();
    }
    pid_forward(ir_to_wheels_dist);
    turn_left_90();

    while (!(IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
    {
      checkpoint_follow();
    }
    pid_forward(ir_to_wheels_dist);
    turn_right_90();
    stage+=1;
    
  }
}

void checkpoint_follow()
{
    read_ir();
    for (int i = 0; i < 8; i++)
        {
        if (i<3 || i==4)
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

void checkpoint1_to_()
{
  if (stage == 2)
  {
    while (true)
    {
      if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0) && (IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        //Serial.println("We are at a left Junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);  // will have to change the # steps
        oled.setCursor(0, 16);
        oled.print("Turning left");
        oled.display();
        //Serial.println("Turning left");
        stop();
        delay(50);
        turn_left_90();
      } 
      else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        //Serial.println("We are at a right junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);
        stop();
        delay(50);
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        //Serial.println("Turning right");
        turn_right_90();
      }
      else if(IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1)
      {
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.print("Y junction");
        oled.display();
        stage+=1;
        break;
      }
    }
  }
}

void _to_dotted_line()
{
  if (stage == 3)
  {
    current_time = millis();
    String side = "L";
    while (true) {
      read_ir();
      if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0) && (IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        //Serial.println("We are at a left Junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);  // will have to change the # steps
        //Serial.println("Turning left");
        stop();
        delay(50);
        oled.setCursor(0, 16);
        oled.print("Turning Right");
        oled.display();
        turn_right_90();
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Stage completed");
        oled.display();
        stage += 1;
        break;
      } 
      else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        //Serial.println("We are at a right junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);
        stop();
        delay(50); 
        oled.setCursor(0, 16);
        oled.print("Turning left");
        oled.display();
        //Serial.println("Turning right");
        turn_left_90();
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Stage completed");
        oled.display();
        stage += 1;
        break;
      }
      else if(IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1)
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Y Junction");
        oled.display();
        stop();
        delay(50);
        turn_left_90(); // will turn until middle two sensors are on the line
      }
      // else if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1)
      // {
      //   lcd.clear(); 
      //   lcd.setCursor(0, 0);
      //   lcd.print("135 Junction");
      //   stop();
      //   delay(50);
      //   //pid_forward(100);
      //   //stop();
      //   //delay(50);
      //   lcd.clear(); 
      //   lcd.setCursor(0, 0);
      //   lcd.print("Turning left");
      //   //Serial.println("Turning right");
      //   turn_left_90();
      // }
      else if(measure_distance() < 7 && side == "L")
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Obstacle found");
        oled.display();
        stop();
        delay(50);
        turn_right_90_using_delay();
        left_to_right();
        side = "R";
      }
      else if(measure_distance() < 7 && side == "R")
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Obstacle found");
        stop();
        delay(50);
        turn_left_90_using_delay();
        right_to_left();
        side = "L";
      }
      else {
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow();
        //pid_forward(15);
      }
    }

  }
}

void left_to_right()
{
  while (true){
    if (IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0)
    {
      oled.clearDisplay(); 
      oled.setCursor(0, 0);
      oled.print("Line Detected");
      oled.display();
      stop();
      forward();
      delay(500); 
      oled.setCursor(0, 16);
      oled.print("Turning Left");
      oled.display();
      turn_left_90();
      break;
    }
    else{
      forward();
    }
  }
}

void right_to_left()
{
  while (true){
    if (IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0)
    {
      oled.clearDisplay(); 
      oled.setCursor(0, 0);
      oled.print("Line Detected");
      oled.display();
      stop();
      delay(50);
      forward();
      delay(500); 
      oled.setCursor(0, 16);
      oled.print("Turning Left");
      oled.display();
      turn_right_90();
      break;
    }
    else{
      forward();
    }
  }
}

void dotted_line_to_checkpoint2()
{
  if (stage == 4)
  {
    int idx;
    current_time = millis();
    bool temp = false;
    while (true){
    if ((IR_Bin_val[0] == 0 || IR_Bin_val[1] == 0 || IR_Bin_val[2] == 0 || IR_Bin_val[3] == 0 || IR_Bin_val[4] == 0 || IR_Bin_val[5] == 0 || IR_Bin_val[6] == 0 || IR_Bin_val[7] == 0) && temp)
    {
      for (int i=0;i<8;i++)
      {
        if (IR_Bin_val[i] == 0)
        {
          idx = i;
        }
      }

      if (idx > 4)
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Line on right");
        oled.display();
        stop();
        delay(50);
        turn_right_until_middle();
      }

      else if (idx < 4)
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Line on left");
        oled.display();
        stop();
        delay(50);
        turn_left_until_middle();
      }
      temp = false;
    }
    else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1 && IR_Bin_val[3] == 1 && IR_Bin_val[4] == 1 && IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1) && !temp)
    {
      temp = true;
    }
    else if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)
    {
      oled.clearDisplay(); 
      oled.setCursor(0, 0);
      oled.print("Checkpoint 2 reached");
      oled.display();
      stage+=1;
      break;
    }
    else
    {
      if ((millis() - current_time) > 1000){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Moving Forward");
        oled.display();
        current_time = millis();
      }
      //Serial.println("Moving Forward");
      set_forward();
      line_follow();
    }
  }
  }
}

void read_received_box_color()
{
  if (stage == 5)
  {
    while (color == -1)
    {
      color = read_received_color_value();
    }
    stage+=1;
  }
}

void checkpoint2_to_L_junction()
{
  if (stage == 6)
  {
    current_time = millis();
    while (true)
    {
      if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        //Serial.println("We are at a right junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);
        stop();
        delay(50);
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        //Serial.println("Turning right");
        turn_right_90();
        stage+=1;
        break;
      }
      else{
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          oled.display();
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow();
      }
    }
  }
}

void L_junction_to_box_pickup()
{
  if (stage == 7)
  {
    current_time = millis();
    while (true)
    {
      if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        //Serial.println("We are at a right junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);
        stop();
        delay(50); 
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        //Serial.println("Turning right");
        turn_right_90();
        arm_down();
        if (color == read_color_sensor())
        {
          turn_left_90();
          stage+=1;
          break;
        }
        else
        {
          arm_up();
          turn_left_90();
        }
      }
      else
      {
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          oled.display();
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow();        
      }      
    }
  }
}

void box_pickup_to_unload()
{
  if (stage == 8){
  current_time = millis();
  while (true)
  {
      if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0) && (IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        forward();
        delay(100); // enough delay to go past the junction
      } 
      else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        forward();
        delay(100); //enough delay to go past the junction
      }
      else if (IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0)
      {
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.print("Box unloaded");
        oled.display();
        stop();
        delay(50);
        //pid_backward(1000);
        //delay(1000);
        turn_left_180();
        stage+=1;
        break;
      }
      else
      {
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          oled.display();
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow(); 
      }
  }
  }
}

//void unload_to_T_junction()
//{
//  if (stage == 9)
//  {
//    current_time = millis();
//    if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
//      lcd.clear(); 
//      lcd.setCursor(0, 0);
//      lcd.print("Right Junction");
//      stop();
//      delay(50);
//      pid_forward(100);
//      stop();
//      delay(50);
//      lcd.clear(); 
//      lcd.setCursor(0, 0);
//      lcd.print("Turning right");
//      //Serial.println("Turning right");
//      turn_right_90();
//    }
//    else if (IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0)
//    {
//      lcd.clear(); 
//      lcd.setCursor(0, 0);
//      lcd.print("Cross Junction");
//      forward();
//      delay(100); // enough delay to go past the junction
//    }
//    else if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0) && (IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
//        lcd.clear(); 
//        lcd.setCursor(0, 0);
//        lcd.print("Left Junction");
//        //Serial.println("We are at a left Junction");
//        stop();
//        delay(50);
//        pid_forward(100);  // will have to change the # steps
//        lcd.clear(); 
//        lcd.setCursor(0, 0);
//        lcd.print("Turning left");
//        //Serial.println("Turning left");
//        stop();
//        delay(50);
//        turn_left_90();
//        stage+=1;
//        break;
//      } 
//    else{
//        if ((millis() - current_time) > 1000){
//          lcd.clear(); 
//          lcd.setCursor(0, 0);
//          lcd.print("Moving Forward");
//          current_time = millis();
//        }
//        //Serial.println("Moving Forward");
//        set_forward();
//        line_follow(); 
//      }
//  }
//}

void unload_to_finish()
{
  if (stage == 9)
  {
    current_time = millis();
    while (true)
    {
      if(IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1)
      {
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.print("Y junction");
        oled.display();
        stop();
        delay(50);
        turn_right_90(); // will turn until middle two sensors are on the line
      }
      else if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0) && (IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        //Serial.println("We are at a left Junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);  // will have to change the # steps
        read_ir();

        if (IR_Bin_val[3] == 0 || IR_Bin_val[4] == 0 || IR_Bin_val[5] == 0 || IR_Bin_val[6] == 1 || IR_Bin_val[7] == 0)
        {
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Turning right");
          //Serial.println("Turning left");
          stop();
          delay(50);
          turn_right_until_middle();
        }
        else
        {
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Turning left");
          oled.display();
          //Serial.println("Turning left");
          stop();
          delay(50);
          turn_left_90();
        }
      } 
      else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        //Serial.println("We are at a right junction");
        stop();
        delay(50);
        pid_forward(ir_to_wheels_dist);
        stop();
        delay(50); 
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        //Serial.println("Turning right");
        turn_right_90();
      }
      else if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)
      {
        stop();
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Task Completed");
        oled.display();
        stage+=1;
        break;
      }
      else{
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          oled.display();
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow(); 
      }
    }
  }
}


// void menu(){
//   while(update_btns()){
//     delay(2);
//   }


//   display_lcd("Menu", "for bac jun ult ");
//   int choice = get_pressed_button();
  
//   if(choice == 1){
//     state = "forward";
//     display_lcd("Line Following", "Forward");
//     delay(1000);
 
//   }else if(choice == 2){
//     state = "back";
//     display_lcd("Line Following", "backward");
//     delay(1000);
 
//   }else if(choice == 3){
    
//     display_lcd("Junction train", "get dis 90 180 ");
//     choice = get_pressed_button();
//     switch (choice){
//       case 1:
//         state = "detect_junc";
//         break;
//       case 2:
//         state = "get distance";
//         break;
//       case 3:
//         state = "turn 90";
//         break;
//       case 4:
//         state = "turn 180";
//         break;
//     }

//     display_lcd("State",state);
    
//   }else if(choice == 4){
//     state = "ultra";
//     display_lcd("gate training",state);
//   }
// }

// int get_pressed_button(){
//   while(!(update_btns())){//this is to wait until a button is pressed
//     delay(5);
    
//   }

//   while((update_btns())){//this is to wait until the button is released
//     delay(5);
//     if(ks1 == true && ks4 == true){
//       display_lcd("State 5", "Entered");
//       delay((1000));
//       return 5;
//     }
//   }

//   if(ks1 == true){
//     return 1;
//   }else if(ks2 == true ){
//     return 2;
//   }else if(ks3 == true ){
//     return 3;
//   }else{
//     return 4;
//   }

// }

// void change_forward_pid(){//ui to change forward pid values
//   stop();    
//   while(true){
//     int choice = get_pressed_button();
//     update_btns();
//     if(choice == 1){
//       Kp--;
//     }else if(choice==2){
//       Kp++;
//     }else if(choice == 3){
//       Kd--;
//     }else if(choice==4){
//       Kd++;
//     }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
//       display_lcd("Kp : "+(String)Kp, "Kd : "+(String)Kd);
//       update_btns();
//       break;
//     }
//     display_lcd("Kp : "+(String)Kp, "Kd : "+(String)Kd);
//   }
// }

// void change_turn90_right_time(){
//   stop();    
//   while(true){
//     int choice = get_pressed_button();
//     update_btns();
//     if(choice == 1){
//       dR90 -= 50;
//     }else if(choice == 2){
//       dR90 += 50;
//     }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
//       display_lcd("TR90 time :" ,(String)dR90);
//       update_btns();
//       break;
//     }
//     display_lcd("TR90 time :" ,(String)dR90);
//   }
// }

// void change_turn90_left_time(){
//   stop();    
//   while(true){
//     int choice = get_pressed_button();
//     update_btns();
//     if(choice == 1){
//       dL90 -= 50;
//     }else if(choice == 2){
//       dL90 += 50;
//     }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
//       display_lcd("TL90 time :" ,(String)dL90);
//       update_btns();
//       break;
//     }
//     display_lcd("TL90 time :" ,(String)dL90);
//   }
// }

// void change_turn180_left_time(){
//   stop();    
//   while(true){
//     int choice = get_pressed_button();
//     update_btns();
//     if(choice == 1){
//       dL180 -= 50;
//     }else if(choice == 2){
//       dL180 += 50;
//     }else if(choice == 5 ){//to get out of this you need to press 1 and 4 at the same time
//       display_lcd("TL180 time :" ,(String)dL180);
//       update_btns();
//       break;
//     }
//     display_lcd("TL180 time :" ,(String)dL180);
//   }
// }

// //keypad functions
// bool update_btns(){
//   // bool tp1 = false;
//   // bool tp2 = false;
//   // bool tp3 = false;
//   // bool tp4 = false;
//   bool pressed = false;
//   ks1 = tp1;
//   ks2 = tp2;
//   ks3 = tp3;
//   ks4 = tp4;
//   tp1 = false;
//   tp2 = false;
//   tp3 = false;
//   tp4 = false;
//   if(digitalRead(key1) == LOW){
//     tp1 = true;
//     pressed = true;
//   }
//   if(digitalRead(key2) == LOW){
//     tp2 = true;
//     pressed = true;
//   }
//   if(digitalRead(key3) == LOW){
//     tp3 = true;
//     pressed = true;
//   }
//   if(digitalRead(key4) == LOW){
//     tp4 = true;
//     pressed = true;
//   }
//   return pressed;
// }
