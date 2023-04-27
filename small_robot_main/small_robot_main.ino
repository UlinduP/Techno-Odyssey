#include <NewPing.h>  // imports the NewPing library for ultrasonic sensor
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
#define trig 41 //ash
#define echo 40 //white

// RGB sensor
#define S0 43  //yellow and orange
#define S1 42 //orage
#define S2 44 //purple
#define S3 45  // ash
#define sensorOut 46 // blue

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
RF24 radio(7, 8);  //defines a new radio variable CE, CSN respectively



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
double IR_weights[8] = {-4, -3, -2, 0, 0, 2, 3, 4};//{-5000, -2000, -1000, 1000, 2000, 5000}; //{-4000, -2000, -1000, 1000, 2000, 4000};  //{-32, -16, -8, -4, -2, 0, 2, 4, 8, 16, 32}

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
int steps = 0;

int current_time = millis();
int color = -1;

//functions legacy
void set_forward(); //checked
void set_backward();//checked
void forward(int Left = Left_MotorBase_speed, int Right = Right_MotorBase_speed);  // sets forward and go forward //checked
void pid_forward(int steps);  // go forward with pid output for a given number of steps //checked
void backward(int Left = Left_MotorBase_speed, int Right = Right_MotorBase_speed);//checked
void pid_backward(int steps);//not checked, not working( need to change back line following algo)
void turn_left_90(int dTime = dL90);//checked
void turn_right_90(int dTime = dR90);//checked
void turn_left_180();//not checked
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
//String read_Serial(); //get String sent through serial
//String read_bluetooth(); //get String sent through bluetooth.
//void horn(int dtime = 0); //to sound buzzer positive arg for delay 0 or no arg to keep the buzzer negative to turn off
//
//void display_lcd(String fline = "",String sline = "",int dtime = 0); //shows whats been passed to upper and lowerlines. dtime is used to keep the line for a while and erase it
//void display_line(String txt, int row = 0); // used to display a line in a row

void show_color(String color);// function to control led "red" "green" "blue" args work. check function to see what other colors can be show
//to get white the led is needed be attached to pwm and tune the values until all junctions work (explain this later)

//bool update_btns();//function to update what's pressed
//
//int get_pressed_button(); // returns what button is pressed - holds in a loop until a button is pressed and released returns 5 if 1 and 4 is pressed at the same time otherwise(1 2 3 4)
//String look_for_junc(int wait = waitTimeJunc);//function to identify junctions identifies right lift 4way and differentiates them from boxes - box box right box left
////(may need to update)
//
////ui functions
//void menu(); //main menu can be accesed by long pressing 4 when inside of the loop
//void change_forward_linefollow_time();//ui to change the pid forward time (this is used to measure distances)
//void change_det_jun();//ui to change junction and box differentiation time
//void change_backward_pid();//ui to change back pid values
//void change_forward_pid();//ui to change forward pid values

// turn lrft or right function you can specify the motor speed to turn the car depending on the scenario otherwise default = base speeds
void turn_left(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed); 
void turn_right(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed); 

////State manager funcs
//void line_back();//backward line following (current algo in it does not work change to a suitable one)
//void line_front();//forward line follow press 3 to update pid while running the func
//void jun_det_line_follow();//junction detection while line following press 3 to update time to  differentiate a junction from a box(this works but there may be bugs in it. test extensively)
////not tested for 4way junctions as well some times this may identify 4way as left or right depending on whichside crossed the line first
//
//void mes_dis_travel();//this function is made to measure time it takes to travel distances - not checked
//void det_jun_turn_90();//detect junctions and turn 
//void turn_180();//go forward and turn 180
//void horn(int dtime);
//
//void change_steps();

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


   //radio
   radio.begin();
   radio.openWritingPipe(address);
   radio.setPALevel(RF24_PA_MAX);
   radio.setDataRate(RF24_250KBPS);
   radio.stopListening();

  //buzzer
  pinMode(buzzerd,OUTPUT);
  
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

  
//  display_lcd("TechBots", "Beetle");
  delay(1000);
  
//  display_lcd("Kp : "+(String)Kp, "Kd : "+(String)Kd); //remove if you  can
  delay(1000);

  t1 = millis();

  pid_forward(100);
}


void loop() {


// things to execute in a given time  interval
  if((t1 - millis()) > 1000){
     // add code
     t1 = millis();
  }

}
void set_forward()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);   
}

void forward(int Left, int Right){ // left right speeds
    Left_MotorBase_speed = Left;
    Right_MotorBase_speed = Right;

    set_forward();

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
    for (int i = 0; i < count; i++) {
        read_ir();
        line_follow();
        delay(2);
    }
    stop();
}

void set_backword()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void backward(int Left, int Right){
    
    Left_MotorBase_speed = Left;
    Right_MotorBase_speed = Right;

    set_backward();

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

void turn_left(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed)
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
}

void turn_right(int left = Left_MotorBase_speed , int right = Right_MotorBase_speed)
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    LMotorSpeed = Left_MotorBase_speed;
    RMotorSpeed = Right_MotorBase_speed;
    motor_speed();
}

void turn_left_90(int dTime = dL90)
{
    turn_left();
    delay(500);  // will have to change the delay
    stop();
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (!(IR_val[3] == 0 & IR_val[4] == 0));
        stop();
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

void turn_right_90(int dTime = dR90)
{
    turn_right();
    delay(500);
    stop();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (!(IR_val[3] == 0 & IR_val[4] == 0));
    stop();
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

void turn_left_180()
{
    turn_left();
    delay(1000);  // will have to change the delay
    stop();
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    do {
        LMotorSpeed = Left_MotorBase_speed;
        RMotorSpeed = Right_MotorBase_speed;
        motor_speed();
        read_ir();
    } while (!(IR_val[3] == 0 & IR_val[4] == 0));
        stop();   
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
    
    Serial.print("Left Speed: ");
    Serial.println(LMotorSpeed);

    Serial.print("Right Speed: ");
    Serial.println(RMotorSpeed);
    
    analogWrite(ENA, LMotorSpeed);
    analogWrite(ENB, RMotorSpeed);
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

     for (int i = 0; i < 8; i++)
   {
     Serial.print(IR_val[i]);
     Serial.print(" ");
   }
   Serial.println(" ");

}

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

int measure_distance()
{
    int duration = ultrasonic.ping_median(); // sends out 5 pulses and takes the median duration
    int distance = ultrasonic.convert_in(duration)*2.54;  // distance in centimeters
    return distance;
}

int read_color_sensor()
{
  int i=0;
  int R_val=0,G_val=0,B_val=0,W_val=0,O_val =0;
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
    else if ((redfrequency>110 && redfrequency <190) && (bluefrequency>30 && bluefrequency<120) && (greenfrequency>70 && greenfrequency<190)) B_val++;
    else if (bluefrequency<100 && redfrequency<100 && greenfrequency<100) W_val++; 
    else O_val++;
    i++;
  }
  while (i<50);

    if(R_val>40) return 1;
    else if (G_val>40) return 2;
    else if (B_val>40) return 3;
    else if (W_val>40) return 4
    else return 5;
}

void send_color_value()
{
  int text=read_color_sensor(); //color value R-1; G-2; B-3; OTHER-4
  for (int i=0;i<5;i++)
  {
    radio.write(&text, sizeof(text));
    delay(200);
  }
}

void checkpoint_one_to_gate_A()
{
    if (stage==0)
    {
        while (measure_distance()>3)
        {
            line_follow();
        }
        stop();
        stage+=1;
    }
}

void gate_A_to_vault()
{
    if (stage==1 && measure_distance()>3)
    {
        int idx;
        current_time = millis();
        bool temp = false;
        while (measure_distance() > 3)
        {
        if ((IR_Bin_val[0] == 1 || IR_Bin_val[1] == 1 || IR_Bin_val[2] == 1 || IR_Bin_val[3] == 1 || IR_Bin_val[4] == 1 || IR_Bin_val[5] == 1 || IR_Bin_val[6] == 1 || IR_Bin_val[7] == 1) && temp)
        {
            for (int i=0;i<8;i++)
            {
                if (IR_Bin_val[i] == 1)
                {
                    idx = i;
                }
            }

            if (idx > 3)
            {
                lcd.clear(); 
                lcd.setCursor(0, 0);
                lcd.print("Line on right");
                stop();
                delay(50);
                turn_right_until_middle();
            }

            else if (idx < 3)
            {
                lcd.clear(); 
                lcd.setCursor(0, 0);
                lcd.print("Line on left");
                stop();
                delay(50);
                turn_left_until_middle();
            }
            temp = false;
        }
        else if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0 && IR_Bin_val[6] == 0 && IR_Bin_val[7] == 0) && !temp)
        {
            temp = true;
        }
        else
        {
            if ((millis() - current_time) > 1000){
            lcd.clear(); 
            lcd.setCursor(0, 0);
            lcd.print("Moving Forward");
            current_time = millis();
            }
            //Serial.println("Moving Forward");
            set_forward();
            line_follow();
        }
        }
            stop();
            delay(50);
            //might will have to call a forward function to get closer to the box
            color = read_color_sensor();
            send_color_value();
            turn_left_180();
            stage+=1;
    }
}

void vault_to_ramp_junction()
{
    if (stage==2)
    {
        forward();
        while (!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0))
        {
            line_follow();
        }
            stop();
            delay(50);
            turn_left_90();
            stage+=1;
    }
}

void ramp_start_to_gate_B()
{
    if (stage==3)
    {
        while (measure_distance()>3)
        {
            Left_MotorBase_speed = 80;  // higher speed for going up the hill
            Right_MotorBase_speed = 80;
            line_follow();
        }
            stop();
            delay(50);
            Left_MotorBase_speed = 70;
            Right_MotorBase_speed = 70;
            stage+=1;
    }
}

void gate_B_to_ramp_end()
{
    if (stage==4 && measure_distance()>20)
    {
        while (!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0))
        {
            Left_MotorBase_speed = 60;  // lower speed for coming down the hill
            Right_MotorBase_speed = 60;
            line_follow();
        }
            stop();
            delay(50);
            Left_MotorBase_speed = 60;
            Right_MotorBase_speed = 60;
            turn_left_180();
            stage+=1;
    }
}

void ramp_end_to_cross()
{
    if (stage==6)
    {
        while(!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0))
        {
            line_follow();
        }
            stop(); // not necessary
            delay(50);
            pid_forward(100);
            stop();
            delay(50);

            // turn_left();
            // delay(200); // turn the robot 180 degrees
            stage+=1;
    }
}

void cross_to_obstacle_box()
{
    if (stage==7)
    {
        while(!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0)){
            line_follow();
        }
            stop();
            delay(50);
            pid_forward(100);
            stop();
            delay(50);
            stage+=1;
    }
}

void obstacle_box_to_T()
{
    if (stage==8)
    {
        while(!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0)){
            line_follow();
        }
            stop();
            delay(50);
            turn_left_180();
            stage+=1;
    }
}

void T_to_ramp_end()
{
    if (stage==9)
    {
        int temp=0;
        while (true)
        {
            line_follow();
            if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0)
            {
                turn_left();
                delay(50); // delay to turn 90 degrees
                temp+=1;
            }
            if (temp==2)
            {
                stage+=1;
                break;
            }
        }
    }
}

void ramp_end_to_pole_junction()
{
    if (stage==10)
    {
        while (!(IR_val[5]==0 && IR_val[4]==0 && IR_val[3]==0 && IR_val[2]==0))
        {
            line_follow();
        }
            turn_right();
            delay(200);
            stage+=1;
    }
}

void pole_junction_to_pole()
{
    if (stage==11)
    {
        while (measure_distance()<12)
        {
            line_follow();
        }
            forward();
            delay(200); // give enough delay for the robot to go and touch the pole
            backward();
            delay(200); 
            turn_left();
            delay(500); // give enough delay for the robot to turn 180 degrees
            stage+=1;
    }
}

void pole_to_pole_junction()
{
    if (stage==12)
    {
        while (!(IR_val[5]==0 && IR_val[4]==0 && IR_val[3]==0 && IR_val[2]==0))
        {
            line_follow();
        }
            turn_right();
            delay(200); // give enough delay for the robot to turn 90 degrees
            stage+=1;
    }
}

void pole_junction_to_finish()
{
    if (stage==13)
    {
        while (!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 ))
        {
            line_follow();
        }
            forward();
            delay(100); // enough delay for the robot to go 10 cm
            stop();
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("All tasks done!");
    }
}
