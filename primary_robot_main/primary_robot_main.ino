#include <NewPing.h>  // imports the NewPing library for ultrasonic sensor

// sensors
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5
#define IR7 A6
#define IR8 A7

#define trig 10
#define echo 11
//#define RGB

NewPing ultrasonic(trig, echo); //defines a new ultrasonic variable

// Motor A connections
int ENA = 9;
int IN1 = 8;
int IN2 = 7;
// Motor B connections
int ENB = 3;
int IN3 = 5;
int IN4 = 4;


int Threshold = 100;
int IR_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};        // IR_Bin_val[0] - left side IR sensor  // IR_Bin_val[10] - right side IR sensor
int IR_Bin_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double IR_weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};//{-5000, -2000, -1000, 1000, 2000, 5000}; //{-4000, -2000, -1000, 1000, 2000, 4000};  //{-32, -16, -8, -4, -2, 0, 2, 4, 8, 16, 32}

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speed_adjust = 0;
int Left_MotorBase_speed = 70;  //60
int Right_MotorBase_speed = 70;
int max_speed = 70;
int min_speed = 40 ;

float Kp = 0;  // 3.05, 15, 0.001 for 43 max    // 1.875*0.3=0.5625 // 1.875 is what you need to utilize the full range  //12*0.25
float Kd = 0; // 8
float Ki = 0;

float P, I, D;
I = 0;   // I is again and again set to zero in pid_forward()

float error = 0;
float previousError = 0; //46

int stage = 0;

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
void turn_left_180();
void stop();
void motor_speed();
void read_ir();
void line_follow();
void measure_distance();
int read_color_sensor();
int read_bluetooth_value();
void send_bluetooth_value(int value);

void setup()
{
  Serial.begin(9600);

	// Set all the motor control pins to outputs
	pinMode(ENA, OUTPUT);
	pinMode(ENB, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
	
	// Turn off motors - Initial state
    stop();
}

void loop()
{

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
        read_IR();
    } while (!(IR_val[3] == 0 & IR_val[4] == 0));
        stop();
}

void turn_right_90()
{
    turn_right();
    delay(500);
    stop();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    do {
        LMotorSpeed = turnspeed;
        RMotorSpeed = turnspeed;
        motor_speed();
        read_ir();
    } while (!(IR_val[3] == 0 & IR_val[4] == 0));
    stop();
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
        read_IR();
    } while (!(IR_val[3] == 0 && IR_val[4] == 0));
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
    
    analogWrite(IN_Speed_Left, LMotorSpeed);
    analogWrite(IN_Speed_Right, RMotorSpeed);
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
       IR_Bin_val[i] = 0;       //change for white strips on black surface
     }
     else
     {
       IR_Bin_val[i] = 1;
     }
   }

     for (int i = 0; i < 8; i++)
   {
     Serial.print(IR_val[i]);
     Serial.print(" ");
   }
   Serial.println(" ");

}

void line_follow()
{
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
    
    motorSpeed();
}

int measure_distance()
{
    int duration = ultrasonic.ping_median(); // sends out 5 pulses and takes the median duration
    int distance = ultrasonic.convert_in(duration)*2.54  // distance in centimeters
    return distance;
}

void start_to_checkpoint1()
{
  bool check = false;
  while (stage == 0 && IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0 && check) {
    read_ir();
    if ((IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0) && (IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1)) {
      check = true;
      Serial.println("We are at a left Junction");
      pid_forward(100);  // will have to change the # steps
      Serial.println("Turning left");
      turn_left_90();
    } 
    else if ((IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1) && (IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0)){
      Serial.println("We are at a right junction");
      pid_forward(100);
      Serial.println("Turning right");
      turn_left_90();
    }
    else if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1){
      Serial.println("We are at a dead end");
      turn_left_180();
    } else {
      Serial.println("Moving Forward");
      line_follow();
      //pid_forward(15);
    }
  }
  else{
    stage += 1;
  }
}