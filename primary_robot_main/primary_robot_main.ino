//#include <NewPing.h>  // import the NewPing library for ultrasonic sensor
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <LiquidCrystal_I2C.h>
#include <SPI.h>    // import the relevent library for SPI communication
#include <nRF24L01.h>  // import the library for radio module
#include <RF24.h>
#include <Servo.h>

int t=350;   // Change for tomorrow 350;        //300 for maze;

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
#define buzzer 4

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
#define key4 A10

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

//NewPing ultrasonic(trig, echo); //defines a new ultrasonic variable
Adafruit_SSD1306 oled = Adafruit_SSD1306(128,64,&Wire,-1);
//LiquidCrystal_I2C lcd(0x3F, 16, 2); // I2C address 0x27, 16 column and 2 rows
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

//const int buttonPin[] = {31, 32, 33, 34};     // the number of the pushbutton pins


int Threshold = 150;
int IR_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};        // IR_Bin_val[0] - left side IR sensor  // IR_Bin_val[10] - right side IR sensor
int IR_Bin_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double IR_weights[8] = {-15,-6,-2,-1,1,2,6,15};//{-16, -8, -4, -2, 2, 4, 8, 16}; //{-4, -3, -2, 0, 0, 2, 3, 4}  {-8, -4, -2, 0, 0, 2, 4, 8}

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speed_adjust = 0;
int Left_MotorBase_speed = 85;//set 130;   //100 for maze
int Right_MotorBase_speed = 105;//set 140; 
int max_speed = 255;
int min_speed = 30;

// color sensor readings
int redfrequency = 0;
int greenfrequency = 0;
int bluefrequency = 0;

float Kp = 60;// for competition           (early 22.04;)             //22.4//0.05 worked   //0.3 // 3.05, 15, 0.001 for 43 max    // 1.875*0.3=0.5625 // 1.875 is what you need to utilize the full range  //12*0.25
float Kd = 200;  //200; for competition   (early 85)              //18 worked   // 85
float Ki = 0;

float P, D;
float I = 0;   // I is again and again set to zero in pid_forward()

int ir_to_wheels_dist = 50;//50  // encoder value 

float error = 0;
float previousError = 0; //46

int counter = 0; 
int aState;
int aLastState; 

int stage = 0;
const byte address[][6] = {"00001","00002"};//the address to which the module should listen for incoming messages

int color = -1;

long current_time;


//turn delays
int dR90 = 500;
int dL90 = 500;
int dL180 = 1000;



void set_rgb_color();
void set_forward();
void forward();  // sets forward and go forward
void pid_forward(int steps);  // go forward with pid output for a given number of steps
void set_backword();
void backward();
void pid_backward(int steps);
void turn_left();
void turn_right(); // after turn left or turn right functions you need to specify the delay and then stop
void turn_left_90();
void turn_left_90_checkpoint();
void turn_left_until_middle();
void turn_right_90();
void turn_right_90_checkpoint();
void turn_right_until_middle();
void turn_right_90_using_delay();
void turn_left_180();
void turn_left_90_using_delay();
void stop();
void motor_speed();
void read_ir();
void display_ir();
void read_encoders();
void line_follow();
float measure_distance();
int read_color_sensor();
void radio_send();
int read_received_color_value();
void start_to_checkpoint1();
void send_start_signal_to_small_robot();
void checkpoint1();
void checkpoint_follow();
void checkpoint1_to_();
void _to_straight_path();
void left_to_right();
void right_to_left();
void straight_path_to_();
void _to_dotted_line();
void dotted_line_to_checkpoint2();
void read_received_box_color();
void checkpoint2_to_L_junction();
void L_junction_to_box_pickup();
void box_pickup_to_unload();
void unload_to_T_junction();
void waiting_for_go();
void T_junction_to_();
void _to_obstacle();
void obstacle_to_();
void _to_finish();
void checkpoint_follow();
void gate_down();
void gate_up();
void arm_down();
void arm_up();

void setup()
{
  Serial.begin(9600);

  // initialize the lcd
  // lcd.init();
  // lcd.backlight();

  //initialize servos
  gate_servo.attach(gate_servo_pin);
  arm_servo.attach(arm_servo_pin);
  arm_servo.write(180);   //85 down  //180 up
  gate_servo.write(50);   //120

  //initialize the oled
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(0,0);



	// Set all the motor control pins to outputs
	pinMode(EN_right, OUTPUT);
	pinMode(EN_left, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

  //buzzer
  pinMode(buzzer, OUTPUT);
//  horn(-1);

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

    // initialize the keypad
  pinMode(key1,INPUT_PULLUP);
  pinMode(key2,INPUT_PULLUP);
  pinMode(key3,INPUT_PULLUP);
  pinMode(key4,INPUT_PULLUP);

  // Setting its frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  //Setup the radio module
  radio.begin();
  radio.stopListening();
  radio.openWritingPipe(address[1]); //00002
  radio.openReadingPipe(1, address[0]); //00001
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);

	// Turn off motors - Initial state
    //stop();

  // Reads the initial state of the outputA
//  aLastState = digitalRead(ENCA_left);

  set_forward();
  current_time = millis();

   read_ir();
     stage=0;

}

void loop()
{
  if((millis()-current_time)>1000){
    read_ir();
    display_ir();
//    lcd.print(counter);
    current_time = millis();
  } 


 //     start_to_checkpoint1();
//      checkpoint1();
//      checkpoint1_to_();
//     _to_straight_path();
//    straight_path_to_();
//     _to_dotted_line();
//     dotted_line_to_checkpoint2();
//     read_received_box_color();
//     checkpoint2_to_L_junction();
//     L_junction_to_box_pickup();
//    box_pickup_to_unload();
// unload_to_T_junction();
// waiting_for_go();
//  T_junction_to_();
//  _to_obstacle();
// obstacle_to_();
//  _to_finish();



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

void backward()
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
  delay(500);  // will have to change the delay // 500 for Base 100
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
  } while (!(IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0));  // changed 3 to 5
      stop();
      delay(50);
}

void turn_left_until_middle()
{
  //-83, -51, -127, 13,-108, -94,   Encoder values
  //counter = 0;
  turn_left();
  //delay(100);  // will have to change the delay
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

    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (!(IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0));  //changed 4 to 2
    stop();
    delay(50);
}

void turn_right_until_middle()
{
    turn_right();
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
    digitalWrite(EN_left, HIGH);
    digitalWrite(EN_right, HIGH);
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
       for (int i = 0; i < 8; i++)
    {
        IR_val[i]=0;
    }
       for (int i = 0; i <5 ; i++)
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
        IR_val[i]=IR_val[i]/5;
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

    // for (int i=0;i<8;i++)
    // {
    //   serial.Println(IR_Bin_val[i]);
    // }
}

void line_follow()
{
  error=0;
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

float measure_distance(){
  digitalWrite(trig, HIGH);
  delayMicroseconds(12);
  digitalWrite(trig, LOW);
  float T=pulseIn(echo,HIGH);
  float x=T/58.00;
  return x;
}

int read_color_sensor()
{
  int i=0;
  int R_val=0,G_val=0,B_val=0,O_val =0,W_val=0;
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

    if ((redfrequency>40 && redfrequency <135) && (bluefrequency>90 && bluefrequency<160) && (greenfrequency>130 && greenfrequency<210)) R_val++;
    else if  ((redfrequency>130 && redfrequency <210) && (bluefrequency>95 && bluefrequency<155) && (greenfrequency>80 && greenfrequency<170)) G_val++;
    else if ((redfrequency>150 && redfrequency <220) && (bluefrequency>40 && bluefrequency<115) && (greenfrequency>100 && greenfrequency<185)) B_val++;
    else if (bluefrequency<100 && redfrequency<100 && greenfrequency<100) W_val++;
    else O_val++;
    i++;
  }
  while (i<51);

    if(R_val>40) return 1;
    else if (G_val>40) return 2;
    else if (B_val>40) return 3;
    else if (W_val>40) return 4;
    else return 5;
}

void radio_send()
{
  int text = 0;
  for (int i=0;i<5;i++)
  {
    radio.write(&text, sizeof(text));
    delay(200);
  }
}

int read_received_color_value()
{
  int R_val=0,G_val=0,B_val=0,W_val=0,O_val=0;
  for (int i=0;i<5;i++)
  {
     if (radio.available())
      {
        int text;
        radio.read(&text, sizeof(text));
        Serial.println(text);
        if (text==1) R_val++;
        else if (text==2) G_val++;
        else if (text==3) B_val++;
        else if (text==4) W_val++;
        else if (text==5) O_val++; 
      }
      delay(1000);
  }

  if (R_val>=1) return 1;
  else if (G_val>=1) return 2;
  else if (B_val>=1) return 3;
  else if (W_val>=1) return 4;
  else if (O_val>=1) return 5;
}

void start_to_checkpoint1()
{
  if (stage==0){
    pid_forward(ir_to_wheels_dist*2);
    current_time = millis();
    while (true) {
      read_ir();
      if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1) && (IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){  // && IR_Bin_val[2] == 1  IR_Bin_val[5] == 0 && 
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("We are at a right junction");
        stop();
        delay(100);
        //pid_forward(ir_to_wheels_dist);
        forward();
        delay(t);
        stop();
        delay(100);
        read_ir();
        if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0){
        for (int i=0;i<3;i++)
        {
          backward();
          delay(500);
          stop();
          delay(100);
          set_forward();
          read_ir();
          while (!(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
          {
            line_follow();
          }
          stop();
          delay(100);
          // while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          // {
          //   read_ir();
          //   backward();
          // }
        }
          oled.clearDisplay();
          oled.setCursor(0, 0);
          oled.print("Stage 1 completed!");
          oled.display();
          Serial.println("Stage 1 completed!");
          delay(100);
          stage+=1;
          break;
        }else{
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        Serial.println("Turning right");
        turn_right_90();
        delay(100);}
      }
      else if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0) && (IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {   //&& IR_Bin_val[2] == 0  IR_Bin_val[5] == 1 && 
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        Serial.println("We are at a left Junction");
        stop();
        delay(100);
        //pid_forward(ir_to_wheels_dist);  // will have to change the # steps
        forward();
        delay(t);
        stop();
        delay(100);
        read_ir();
        if (IR_Bin_val[0] == 0 || IR_Bin_val[1] == 0 || IR_Bin_val[2] == 0 || IR_Bin_val[3] == 0 || IR_Bin_val[4] == 0 || IR_Bin_val[5] == 0 || IR_Bin_val[6] == 0 || IR_Bin_val[7] == 0){
          line_follow();
        }
        else if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0){
        for (int i=0;i<3;i++)
        {
          backward();
          delay(500);
          stop();
          delay(100);
          set_forward();
          read_ir();
          while (!(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
          {
            line_follow();
          }
          stop();
          delay(100);
          // while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          // {
          //   read_ir();
          //   backward();
          // }
        }
          oled.clearDisplay();
          oled.setCursor(0, 0);
          oled.print("Stage 1 completed!");
          oled.display();
          Serial.println("Stage 1 completed!");
          delay(100);
          stage+=1;
          break;
        }
        else{        
        stop();
        delay(100);
        oled.setCursor(0, 16);
        oled.print("Turning left");
        oled.display();
        Serial.println("Turning left");
        turn_left_90();
        delay(100); 
        }
      }   
      else if(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0){
        stop();
        delay(100);
        forward();
        delay(t);
        stop();
        delay(100);
        read_ir();
        if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0){
        for (int i=0;i<3;i++)
        {
          backward();
          delay(500);
          stop();
          delay(100);
          set_forward();
          read_ir();
          while (!(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
          {
            line_follow();
          }
          stop();
          delay(100);
          // while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          // {
          //   read_ir();
          //   backward();
          // }
        }
          oled.clearDisplay();
          oled.setCursor(0, 0);
          oled.print("Stage 1 completed!");
          oled.display();
          Serial.println("Stage 1 completed!");
          delay(100);
          stage+=1;
          break;
        }
        else{
          oled.clearDisplay();
          oled.setCursor(0, 0);
          oled.print("Turning right");
          oled.display();
          Serial.println("Turning right");
          turn_right_90();
          delay(100);
        }
      }
      else if (IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1 && IR_Bin_val[3] == 1 && IR_Bin_val[4] == 1 && IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Dead end");
        oled.display();
        Serial.println("We are at a dead end");
        stop();
        delay(100);
        turn_left_180();
        delay(100);
      } 
      else {
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          oled.display();
          Serial.println("Moving Forward");
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

void send_start_signal_to_small_robot()
{
    radio.stopListening();
    int start=6;
    for (int i=0;i<=20;i++)
    {
       radio.write(&start, sizeof(start));
       delay(100);
    }
}

void checkpoint1()
{
  if (stage == 1)
  {
    while (true)
    {
      read_ir();
      if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1) && (IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){  // && IR_Bin_val[2] == 1  IR_Bin_val[5] == 0 && 
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("We are at a right junction");
        stop();
        delay(100);
        //pid_forward(ir_to_wheels_dist);
        for (int i=0;i<3;i++)
        {
          backward();
          delay(500);
          stop();
          delay(100);
          set_forward();
          read_ir();
          while (!(IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
          {
            line_follow();
          }
          stop();
          delay(100);
        }
        gate_down();
        send_start_signal_to_small_robot();
        delay(5000);
        gate_up();
        delay(1000);
        forward();
        delay(t);
        stop();
        delay(100);
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        Serial.println("Turning right");
        turn_right_90();
        delay(100);
        stage+=1;
        break;
      }
      else{
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          oled.display();
          Serial.println("Moving Forward");
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

void turn_right_90_checkpoint()
{
    turn_right();
    delay(500);
    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (IR_Bin_val[4] == 0);  //changed 4 to 2
    stop();
    delay(50);
}

void turn_left_90_checkpoint()
{
    turn_left();
    delay(500);

    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (IR_Bin_val[3]==1);  //changed 4 to 2
    stop();
    delay(50);
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
        Serial.println("We are at a left Junction");
        stop();
        delay(100);
        //pid_forward(ir_to_wheels_dist);  // will have to change the # steps
        forward();
        delay(t);
        oled.setCursor(0, 16);
        oled.print("Turning left");
        oled.display();
        Serial.println("Turning left");
        stop();
        delay(100);
        turn_left_90();
        delay(100); 
      } 
      else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("We are at a right junction");
        stop();
        delay(100);
        //pid_forward(ir_to_wheels_dist);
        forward();
        delay(t);
        stop();
        delay(100);
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        Serial.println("Turning right");
        turn_right_90();
        delay(100);
      }
      else if(IR_Bin_val[0] == 0 && IR_Bin_val[3] == 1 && IR_Bin_val[4] == 1 && IR_Bin_val[7] == 0)
      {
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.print("Y junction");
        oled.display();
        Serial.println("Y junction");
        stop();
        delay(100);
        turn_left_until_middle();
        delay(100);
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
          Serial.println("Moving Forward");
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow();
      }
    }
  }
}

void _to_straight_path()
{
  if (stage == 3)
  {
    t=t-50;
    current_time = millis();
    while (true)
    {
    if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("Right Junction");
        //Serial.println("We are at a right junction");
        stop();
        delay(100);
//        forward();
//        delay(t);
//        stop();
//        delay(100);
        Serial.println("Turning right");
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        turn_right_until_middle();
        stop();
        delay(100); 
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Stage completed");
        oled.display();
        stage += 1;
        break;
      }
    else if(IR_Bin_val[0] == 0 && IR_Bin_val[3] == 1 && IR_Bin_val[4] == 1 && IR_Bin_val[7] == 0)
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Y Junction");
        oled.display();
        Serial.println("Y junction");
//        forward();
//        delay(t);
        stop();
        delay(100);
        turn_left_until_middle(); // will turn until middle two sensors are on the line
        stop();
        delay(100);
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
void straight_path_to_()
{
  if (stage == 4)
  {
    current_time = millis();
    String side = "L";
    measure_distance();
    delay(20);
    while (true) {
      read_ir();
      float x = measure_distance();
      // Serial.println(x);
      
      if ((IR_Bin_val[2] == 0 && IR_Bin_val[1] == 0) && (IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        Serial.println("Left Junction");
        //Serial.println("We are at a left Junction");
        stop();
        delay(100);
        forward();
        delay(t);
        stop();
        delay(100);
        oled.setCursor(0, 16);
        oled.print("Turning Left");
        oled.display();
        Serial.println("Turning Left");
        turn_left_until_middle();
        stop();
        delay(100);
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Stage completed");
        oled.display();
        stage += 1;
        break;
      } 
      else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1) && (IR_Bin_val[6] == 0 && IR_Bin_val[5] == 0))   //IR_Bin_val[5] == 0 && 
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("Right Junction");
        //Serial.println("We are at a right junction");
        stop();
        delay(100);
        forward();
        delay(t);
        stop();
        delay(100);
        Serial.println("Turning Right");
        oled.setCursor(0, 16);
        oled.print("Turning Right");
        oled.display();
        turn_right_until_middle();
        stop();
        delay(100); 
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Stage completed");
        oled.display();
        stage += 1;
        break;
      }
      else if(x< 7 && side == "L")
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Obstacle found");
        oled.display();
        Serial.println("Obstacle Found");
        stop();
        delay(100);
        turn_right_90_using_delay();
        delay(100);
        left_to_right();
        
        side = "R";
      }
      else if(x < 7 && side == "R")
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
      delay(10);
    }

  }
}

void left_to_right()
{
  while (true){
    read_ir();
    if (IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0)
    {
      oled.clearDisplay(); 
      oled.setCursor(0, 0);
      oled.print("Line Detected");
      oled.display();
      Serial.println("Line Detected");
      stop();
      delay(100);
      forward();
      delay(t); 
      stop();
      delay(100);
      oled.setCursor(0, 16);
      oled.print("Turning Right");
      oled.display();
      Serial.println("Turning Right");
      turn_left_90();
      delay(100);
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
    read_ir();
    if (IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0)
    {
      oled.clearDisplay(); 
      oled.setCursor(0, 0);
      oled.print("Line Detected");
      oled.display();
      Serial.println("Line Detected");
      stop();
      delay(100);
      forward();
      delay(t);
      stop();
      delay(100); 
      oled.setCursor(0, 16);
      oled.print("Turning Left");
      oled.display();
      turn_right_90();
      delay(100);
      break;
    }
    else{
      forward();
    }
  }
}

void _to_dotted_line()
{
 if (stage==5)
 {
    while (true) {
      read_ir();
      
      if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0) && (IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        Serial.println("Left Junction");
        //Serial.println("We are at a left Junction");
        stop();
        delay(100);
        forward();
        delay(t);
        stop();
        delay(100);
        oled.setCursor(0, 16);
        oled.print("Turning Right");
        oled.display();
        Serial.println("Turning Right");
        turn_right_until_middle();
        stop();
        delay(100);
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Stage completed");
        oled.display();
        stage += 1;
        break;
      } 
      else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1) && (IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))   //IR_Bin_val[5] == 0 && 
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("Right Junction");
        //Serial.println("We are at a right junction");
        stop();
        delay(100);
        forward();
        delay(300);
        stop();
        delay(100);
        Serial.println("Turning Left");
        oled.setCursor(0, 16);
        oled.print("Turning Left");
        oled.display();
        turn_left_until_middle();
        stop();
        delay(100); 
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Stage completed");
        oled.display();
        stage += 1;
        break;
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

void dotted_line_to_checkpoint2()
{
  if (stage == 6)
  {
    int idx;
    current_time = millis();
    bool temp = false;
    while (true){
    if ((IR_Bin_val[0] == 0 || IR_Bin_val[1] == 0 || IR_Bin_val[2] == 0 || IR_Bin_val[3] == 0 || IR_Bin_val[4] == 0 || IR_Bin_val[5] == 0 || IR_Bin_val[6] == 0 || IR_Bin_val[7] == 0) && temp)
    {
      stop();
      delay(100);
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
        Serial.println("Line on right");
        turn_right_until_middle();
        stop();
        delay(100);
      }

      else if (idx < 4)
      {
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Line on left");
        oled.display();
        Serial.println("Line on left");
        turn_left_until_middle();
        stop();
        delay(100);
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
      stop();
      delay(100);
      forward();
      delay(700);
      stop();
      delay(100);
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
  if (stage == 7)
  {
    radio.startListening();
    current_time = millis();
    while ((color != 1 && color!=2 && color != 3 && color!=4 && color != 5) || ((millis()-current_time)>20000))
    {
      if (radio.available())
        {
          radio.read(&color, sizeof(color));
          Serial.println(color);
        }
        //delay(1000);
      }
    stage+=1;
    radio.stopListening();
    }
}


void checkpoint2_to_L_junction()
{
  if (stage == 8)
  {
    current_time = millis();
    while (true)
    {
      set_forward();
      read_ir();
      if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        //Serial.println("We are at a right junction");
        stop();
        delay(100);
        forward();
        delay(t);
        stop();
        delay(100);
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        //Serial.println("Turning right");
        turn_right_90();
        delay(100);
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
  if (stage == 9)
  {
    int count = 0;
    current_time = millis();
    measure_distance();
    delay(20);
    while(true)
    {
      read_ir();
      float x = measure_distance();
      //Serial.println(x);
      if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1) && (IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){   // IR_Bin_val[2]==1 IR_Bin_val[5]==0
        count+=1;
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("We are at a right junction");
        stop();
        delay(100);
        forward();
        delay(t);
        stop();
        delay(100); 
        oled.setCursor(0, 16);
        oled.print("Turning right");
        oled.display();
        Serial.println("Turning right");
        turn_right_90();
        stop();
        delay(100);
        for (int i=0;i<3;i++)
        {
          set_forward();
          while (measure_distance()>5)
          {
            line_follow();
            delay(10);
          }
          stop();
          delay(100);
          // while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          // {
          //   read_ir();
          //   backward();
          // }
          backward();
          delay(400);
          stop();
          delay(100);
        }
      }
      else if (x<7){
        stop();
        delay(100);
        Serial.println("Arm Down");
        arm_down();
        delay(100);
        if (count == 4)
        {
          stop();
          delay(100);
          while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          {
            read_ir();
            backward();
          }
          stop();
          delay(100);
          forward();
          delay(t);
          stop();
          delay(100);
          turn_left_90();
          stop();
          delay(100);
          oled.clearDisplay();
          oled.setCursor(0,0);
          oled.print("Box picked up");
          oled.display();
          stage+=1;
          break;
        }
        int a=read_color_sensor();
        delay(1000);
        oled.clearDisplay();
        oled.print(a);
        oled.display();
        if (color == a)
        {
          stop();
          delay(100);
          while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          {
            read_ir();
            backward();
          }
          stop();
          delay(100);
          forward();
          delay(t);
          stop();
          delay(100);
          turn_left_90();
          stop();
          delay(100);
          oled.clearDisplay();
          oled.setCursor(0,0);
          oled.print("Box picked up");
          oled.display();
          stage+=1;
          break;
        }
        else
        {
          arm_up();
          stop();
          delay(100);
          while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          {
            read_ir();
            backward();
          }
          stop();
          delay(100);
          forward();
          delay(t);
          stop();
          delay(100);
          turn_left_90();
          stop();
          delay(100);
        } 
      }    
      else
      {
        if ((millis() - current_time) > 1000){
          oled.clearDisplay(); 
          oled.setCursor(0, 0);
          oled.print("Moving Forward");
          oled.display();
          Serial.println("Moving Forward");
          current_time = millis();
        }
        //Serial.println("Moving Forward");
        set_forward();
        line_follow();
      }   
      delay(20);
    }
  }
}

void box_pickup_to_unload()
{
  if (stage == 10){
  current_time = millis();
  read_ir();
  set_forward();
  // Left_MotorBase_speed=50;
  // Right_MotorBase_speed=60;
  while (true)
  {
    read_ir();
      if ((IR_Bin_val[1] == 1 && IR_Bin_val[0] == 1) && (IR_Bin_val[7] == 0 && IR_Bin_val[6] == 0)){
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("Right Junction");
        stop();
        delay(100);
        arm_up();
        delay(200);
        float x=measure_distance();
        if (x>20){
          stop();
          delay(100);
          backward();
          delay(t+200);
          stop();
          delay(100);
          turn_left_180();
          delay(100);
          stage+=1;
          break;
        }
        else{
          arm_down();
          delay(200);
          forward();
          delay(200);
        }
        turn_right_until_middle();
        stop();
        delay(100);
       // forward();
        //delay(100); //enough delay to go past the junction
        stop();
        delay(100);
      }
      else if (IR_Bin_val[0] == 0 && IR_Bin_val[1]==0 && IR_Bin_val[2]==0 && IR_Bin_val[5]==1 && IR_Bin_val[6]==1 && IR_Bin_val[7]==1)
      {
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.print("Left Junction");
        oled.display();
        Serial.println("Left Junction");
        stop();
        delay(100);
        arm_up();
        delay(200);
        float x=measure_distance();
        if (x>20){
          stop();
          delay(100);
          backward();
          delay(t+200);
          stop();
          delay(100);
          turn_left_until_middle();
          delay(100);
          stage+=1;
          break;
        }
        else{
          arm_down();
          delay(200);
          forward();
          delay(200);
        }
        turn_left_until_middle();
        stop();
        delay(100);
        // forward();
        // delay(t);
        stop();
        delay(100);
        // stage+=1;
        // break;
      }
      else if(IR_Bin_val[1]==0 && IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0 && IR_Bin_val[6]==0)
      {
        stop();
        delay(100);
        arm_up();
        delay(200);
        float x=measure_distance();
        if (x>20){
          stop();
          delay(100);
          backward();
          delay(t+200);
          stop();
          delay(100);
          turn_left_until_middle();
          delay(100);
          stage+=1;
          break;
        }
        else{
          arm_down();
          delay(200);
          forward();
          delay(200);
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

// void box_pickup_to_()
// {
//   if (stage == 10){
//   current_time = millis();
//   read_ir();
//   set_forward();
//   // Left_MotorBase_speed=50;
//   // Right_MotorBase_speed=60;
//   while (true)
//   {
//     read_ir();
//       if ((IR_Bin_val[1] == 1 && IR_Bin_val[0] == 1) && (IR_Bin_val[7] == 0 && IR_Bin_val[6] == 0)){
//         oled.clearDisplay(); 
//         oled.setCursor(0, 0);
//         oled.print("Right Junction");
//         oled.display();
//         Serial.println("Right Junction");
//         forward();
//         delay(100); //enough delay to go past the junction
//         stop();
//         delay(100);
//       }
//       else if (IR_Bin_val[0] == 0 && IR_Bin_val[1]==0 && IR_Bin_val[2]==0 && IR_Bin_val[5]==1 && IR_Bin_val[6]==1 && IR_Bin_val[7]==1)
//       {
//         oled.clearDisplay();
//         oled.setCursor(0, 0);
//         oled.print("Stage completed");
//         oled.display();
//         Serial.println("Stage completed");
//         stop();
//         delay(100);
//         forward();
//         delay(t);
//         stop();
//         delay(100);
//         stage+=1;
//         break;
//       }
//       else
//       {
//         if ((millis() - current_time) > 1000){
//           oled.clearDisplay(); 
//           oled.setCursor(0, 0);
//           oled.print("Moving Forward");
//           oled.display();
//           current_time = millis();
//         }
//         //Serial.println("Moving Forward");
//         set_forward();
//         line_follow(); 
//       }
//   }
//   }
// }

// void _to_unload()
// {
//   if (stage == 11){
//   current_time = millis();
//   turn_right_until_middle();
//   stop();
//   delay(100);
//   read_ir();
//   while (true)
//   {
//     if (IR_Bin_val[1]==0 && IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0 && IR_Bin_val[6]==0)
//     {
//       Serial.println("Box Unloaded");
//       stop();
//       delay(100);
//       forward();
//       delay(200);
//       stop();
//       delay(200);
//       arm_up();
//       delay(300);
//       backward();
//       delay(t+200);
//       stop();
//       delay(100);
//       turn_left_180();
//       stop();
//       delay(100);
//       stage+=1;
//       break;
//     }
//       else
//       {
//         if ((millis() - current_time) > 1000){
//           oled.clearDisplay(); 
//           oled.setCursor(0, 0);
//           oled.print("Moving Forward");
//           oled.display();
//           current_time = millis();
//         }
//         //Serial.println("Moving Forward");
//         set_forward();
//         line_follow(); 
//       }
//   }
//   }
// }

void unload_to_T_junction()
{
  if (stage==11)
  {
    current_time = millis();
    while (true)
    {read_ir();
    if (IR_Bin_val[7]==0)
    {
      Serial.println("Right Junction");
      stop();
      delay(100);
      forward();
      delay(t);
      stop();
      delay(100);
      turn_right_90();
      stop();
      delay(100);
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

void waiting_for_go()
{
  if (stage==12){
    radio.startListening();
    int evans = 0;
    while ((evans != 100) || ((millis()-current_time)>30000))
    {
      if (radio.available())
        {
          radio.read(&evans, sizeof(evans));
          Serial.println(evans);
        }
        //delay(1000);
      }
    stage+=1;
    radio.stopListening();
  }
}

void T_junction_to_()
{
  if (stage==13)
  {
    current_time = millis();
    read_ir();
    while (true)
    {
      if(IR_Bin_val[0] == 0 && IR_Bin_val[3] == 1 && IR_Bin_val[4] == 1 && IR_Bin_val[7] == 0)
      {
        stop();
        delay(100);
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.print("Y junction");
        oled.display();
        stop();
        delay(100);
        turn_left_until_middle(); // will turn until middle two sensors are on the line
        delay(100);
        stop();
        delay(100);
        stage+=1;
        break;
      }
      else if(IR_Bin_val[0]==0 && IR_Bin_val[1]==0 && IR_Bin_val[2]==0 && IR_Bin_val[5]==1 && IR_Bin_val[6]==1 && IR_Bin_val[7]==1)
      {
        stop();
        delay(100);
        forward();
        delay(t);
        stop();
        delay(100);
        turn_left_90();
        delay(100);
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

void _to_obstacle()
{
  if (stage==14)
  {
    current_time = millis();
    read_ir();
    float x = measure_distance();
    delay(20);
    while (x>8)
    {
      if(IR_Bin_val[0]==1 && IR_Bin_val[1]==1 && IR_Bin_val[6]==0 && IR_Bin_val[7]==0)
      {
        stop();
        delay(100);
        turn_right_until_middle();
        forward();
        delay(100);
        stop();
        delay(100);
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
      delay(20);
      x=measure_distance();
  }
  stop();
  stage+=1;
  }
}

void obstacle_to_()
{
  if (stage==15)
  {
    current_time = millis();
    read_ir();
    float x = measure_distance();
    delay(20);
    while (x>15)
    {
      if(IR_Bin_val[0]==1 && IR_Bin_val[1]==1 && IR_Bin_val[6]==0 && IR_Bin_val[7]==0)
      {
        stop();
        delay(100);
        turn_right_until_middle();
        delay(100);
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
      delay(20);
      x=measure_distance();
    }
  }
}

void _to_finish()
{
 if (stage==16)
 {
   int count = 0;
    while (true) {
      read_ir();
      if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1) && (IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))   //IR_Bin_val[5] == 0 && 
      {
        count+=1;
        if (count==2)
        {
          for (int i=0;i<2;i++)
        {
          backward();
          delay(500);
          stop();
          delay(100);
          set_forward();
          read_ir();
          while (!(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
          {
            line_follow();
          }
          stop();
          delay(100);
          // while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          // {
          //   read_ir();
          //   backward();
          // }
        }
        oled.clearDisplay();
        oled.setCursor(0,0);
        oled.print("All Tasks Done!!!");
        oled.display();
        forward();
        delay(600);
        tone(buzzer,1000);
        delay(1000);
        noTone(buzzer);
        stage+=1;
        break;
        }
        oled.clearDisplay(); 
        oled.setCursor(0, 0);
        oled.print("Right Junction");
        oled.display();
        Serial.println("Right Junction");
        //Serial.println("We are at a right junction");
        stop();
        delay(100);
        forward();
        delay(300);
        stop();
        delay(100);
        Serial.println("Turning Left");
        oled.setCursor(0, 16);
        oled.print("Turning Left");
        oled.display();
        turn_left_until_middle();
        stop();
        delay(100); 
      }

      else if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)
      {
        count+=1;
        if (count==2)
        {
          for (int i=0;i<2;i++)
        {
          backward();
          delay(500);
          stop();
          delay(100);
          set_forward();
          read_ir();
          while (!(IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0))
          {
            line_follow();
          }
          stop();
          delay(100);
          // while (!(IR_Bin_val[2]==0 && IR_Bin_val[3]==0 && IR_Bin_val[4]==0 && IR_Bin_val[5]==0))
          // {
          //   read_ir();
          //   backward();
          // }
        }
        oled.clearDisplay();
        oled.setCursor(0,0);
        oled.print("All Tasks Done!!!");
        oled.display();
        tone(buzzer,1000);
        delay(1000);
        noTone(buzzer);
        forward();
        delay(600);
        stop();
        stage+=1;
        break;}
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

// void T_junction_to_finish()
// {
//   if (stage == 12)
//   {
//     current_time = millis();
//     read_ir();
//     while (true)
//     {
//       if(IR_Bin_val[0] == 0 && IR_Bin_val[3] == 1 && IR_Bin_val[4] == 1 && IR_Bin_val[7] == 0)
//       {
//         oled.clearDisplay();
//         oled.setCursor(0, 0);
//         oled.print("Y junction");
//         oled.display();
//         stop();
//         delay(100);
//         turn_right_until_middle(); // will turn until middle two sensors are on the line
//       }
//       else if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0) && (IR_Bin_val[5] == 1 && IR_Bin_val[6] == 1 && IR_Bin_val[7] == 1)) {
//         oled.clearDisplay(); 
//         oled.setCursor(0, 0);
//         oled.print("Left Junction");
//         oled.display();
//         //Serial.println("We are at a left Junction");
//         stop();
//         delay(100);
//         forward();  // will have to change the # steps
//         delay(t);
//         read_ir();

//         if (IR_Bin_val[3] == 0 || IR_Bin_val[4] == 0 || IR_Bin_val[5] == 0 || IR_Bin_val[6] == 1 || IR_Bin_val[7] == 0)
//         {
//           oled.clearDisplay(); 
//           oled.setCursor(0, 0);
//           oled.print("Turning right");
//           //Serial.println("Turning left");
//           stop();
//           delay(50);
//           turn_right_until_middle();
//         }
//         else
//         {
//           oled.clearDisplay(); 
//           oled.setCursor(0, 0);
//           oled.print("Turning left");
//           oled.display();
//           //Serial.println("Turning left");
//           stop();
//           delay(50);
//           turn_left_90();
//         }
//       } 
//       else if ((IR_Bin_val[0] == 1 && IR_Bin_val[1] == 1 && IR_Bin_val[2] == 1) && (IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)){
//         oled.clearDisplay(); 
//         oled.setCursor(0, 0);
//         oled.print("Right Junction");
//         oled.display();
//         //Serial.println("We are at a right junction");
//         stop();
//         delay(50);
//         pid_forward(ir_to_wheels_dist);
//         stop();
//         delay(50); 
//         oled.setCursor(0, 16);
//         oled.print("Turning right");
//         oled.display();
//         //Serial.println("Turning right");
//         turn_right_90();
//       }
//       else if (IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0)
//       {
//         stop();
//         oled.clearDisplay(); 
//         oled.setCursor(0, 0);
//         oled.print("Task Completed");
//         oled.display();
//         stage+=1;
//         break;
//       }
//       else{
//         if ((millis() - current_time) > 1000){
//           oled.clearDisplay(); 
//           oled.setCursor(0, 0);
//           oled.print("Moving Forward");
//           oled.display();
//           current_time = millis();
//         }
//         //Serial.println("Moving Forward");
//         set_forward();
//         line_follow(); 
//       }
//     }
//   }
// }

void gate_down()
{
for(int i=90;i>=0;i-=20){
  gate_servo.write(i);     
  delay(50);
  }
}

void gate_up()
{
for(int i=0;i<=170;i+=30){
  gate_servo.write(i);     
  delay(50);
  }
}

void arm_down()
{
for(int i=180;i>=82;i-=5){  //82 previous
  arm_servo.write(i);     
  delay(5);
  }
}

void arm_up()
{
 for(int i=78;i<=180;i+=1){
   arm_servo.write(i);     
   delay(10);
 }
}

