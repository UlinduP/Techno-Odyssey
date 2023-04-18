#include <NewPing.h>  // imports the NewPing library for ultrasonic sensor
#include <SPI.h>    // import the relevent library for SPI communication
#include <nRF24L01.h>  // import the library for radio module
#include <RF24.h>

// sensors
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5

#define trig 10
#define echo 11

//#define RGB
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

NewPing ultrasonic(trig, echo); //defines a new ultrasonic variable
RF24 radio(7, 8);  //defines a new radio variable CE, CSN respectively

// Motor A connections
int ENA = 9;
int IN1 = 8;
int IN2 = 7;
// Motor B connections
int ENB = 3;
int IN3 = 5;
int IN4 = 4;


int Threshold = 100;
int IR_val[6] = {0, 0, 0, 0, 0, 0};        // IR_Bin_val[0] - left side IR sensor  // IR_Bin_val[10] - right side IR sensor
int IR_Bin_val[6] = {0, 0, 0, 0, 0, 0};
double IR_weights[6] = {-3, -2, -1, 1, 2, 3};//{-5000, -2000, -1000, 1000, 2000, 5000}; //{-4000, -2000, -1000, 1000, 2000, 4000};  //{-32, -16, -8, -4, -2, 0, 2, 4, 8, 16, 32}

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speed_adjust = 0;
int Left_MotorBase_speed = 70;  //60
int Right_MotorBase_speed = 70;
int max_speed = 70;
int min_speed = 40 ;

// for color sensor readings
int redfrequency = 0;
int greenfrequency = 0;
int bluefrequency = 0;

float Kp = 0;  // 3.05, 15, 0.001 for 43 max    // 1.875*0.3=0.5625 // 1.875 is what you need to utilize the full range  //12*0.25
float Kd = 0; // 8
float Ki = 0;

float P, I, D;
I = 0;   // I is again and again set to zero in pid_forward()

float error = 0;
float previousError = 0; //46

int stage = 0;
const byte address[6] = "00001"; //the address to which the module should listen for incoming messages

void set_forward();
void forward();  // sets forward and go forward
void pid_forward(int steps);  // go forward with pid output for a given number of steps
void set_backword();
void backword();
void pid_backward(int steps);
void turn_left();
void turn_right(); // after turn left or turn right functions you need to specify the delay and then stop
void turn_left_90();
void turn_left_until_middle();
void turn_right_90();
void turn_right_until_middle();
void turn_left_180();
void stop();
void motor_speed();
void read_ir();
void line_follow();
int measure_distance();
int read_color_sensor();
void send_color_value();
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
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.stopListening();
	
	// Turn off motors - Initial state
    stop();
}

void loop()
{
    checkpoint_one_to_gate_A();
    gate_A_to_vault();
    vault_to_ramp_junction();
    ramp_start_to_gate_B();
    gate_B_to_ramp_end();
    ramp_end_to_cross();
    cross_to_obstacle_box_to_cross();
    cross_to_ramp_end();
    ramp_end_to_pole_junction();
    pole_junction_to_pole();
    pole_to_pole_junction();
    pole_junction_to_finish();
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
    } while (!(IR_val[2] == 0 & IR_val[3] == 0));
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
    } while (!(IR_val[2] == 1 & IR_val[3] == 1));
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
        read_IR();
    } while (!(IR_val[2] == 0 & IR_val[3] == 0));
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

       for (int i = 0; i < 6; i++)
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
    }
        for (int i = 0; i < 6; i++)
    {
        IR_val[i]=IR_val[i]/3;
    }

    for (int i = 0; i < 6; i++)
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

     for (int i = 0; i < 6; i++)
   {
     Serial.print(IR_val[i]);
     Serial.print(" ");
   }
   Serial.println(" ");

}

void line_follow()
{
    for (int i = 0; i < 6; i++)
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

int read_color_sensor()
{
  int i=0;
  int R_val=0,G_val=0,B_val=0,O_val =0;
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
        while (measure_distance()>10)
        {
            line_follow();
        }
        else{
            stop();
            stage+=1;
        }
    }
}

void gate_A_to_vault()
{
    if (stage==1 && measure_distance()>20)
    {
        int idx;
        current_time = millis();
        bool temp = false;
        while (measure_distance() > 8)
        {
        if ((IR_Bin_val[0] == 1 || IR_Bin_val[1] == 1 || IR_Bin_val[2] == 1 || IR_Bin_val[3] == 1 || IR_Bin_val[4] == 1 || IR_Bin_val[5] == 1) && temp)
        {
            for (int i=0;i<6;i++)
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
        else if ((IR_Bin_val[0] == 0 && IR_Bin_val[1] == 0 && IR_Bin_val[2] == 0 && IR_Bin_val[3] == 0 && IR_Bin_val[4] == 0 && IR_Bin_val[5] == 0) && !temp)
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
         
        else
        {
            stop();
            //might will have to call a forward function to get closer to the box
            int color = read_color_sensor();
            send_bluetooth_value(color);
            turn_left();
            delay(100);  // add enough delay for the robot to turn 180 degrees
            stage+=1;
        }
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
        else{
            stop();
            turn_left();
            delay(20); // add enough delay to turn 90 degrees
            stage+=1;
        }
    }
}

void ramp_start_to_gate_B()
{
    if (stage==3)
    {
        while (measure_distance()>10)
        {
            line_follow();
        }
        else{
            stop();
            stage+=1;
        }
    }
}

void gate_B_to_ramp_end()
{
    if (stage==4 && measure_distance()>20)
    {
        while (!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0))
        {
            line_follow();
        }
        else{
            stop();
            turn_left();
            delay(200); //turn 180 degrees
            stage+=1;
        }
    }
}

void ramp_end_to_cross()
{
    if (stage==6)
    {
        while(!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0))
        {
            line_follow();
        }
        else{
            stop(); // not necessary
            turn_left();
            delay(200); // turn the robot 180 degrees
            stage+=1;
        }
    }
}

void cross_to_obstacle_box_to_cross()
{
    if (stage==7)
    {
        backword();
        delay(2000); // add delay such that the vehicle would travel 30 cm 
        forward();
        delay(1500); // add delay such that the vehicle would travel 30 cm without the obstacle
        stage+=1;
    }
}

void cross_to_ramp_end()
{
    if (stage==8)
    {
        int temp=0;
        while (true)
        {
            line_follow();
            if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0)
            {
                turn_left();
                delay(50); // delay to turn 90 degrees
                temp+=1
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
    if (stage==9)
    {
        while (!(IR_val[5]==0 && IR_val[4]==0 && IR_val[3]==0 && IR_val[2]==0))
        {
            line_follow();
        }
        else
        {
            turn_right();
            delay(200);
            stage+=1;
        }
    }
}

void pole_junction_to_pole()
{
    if (stage==10)
    {
        while (measure_distance()<12)
        {
            line_follow();
        }
        else{
            forward();
            delay(200); // give enough delay for the robot to go and touch the pole
            backword();
            delay(200); 
            turn_left();
            delay(500); // give enough delay for the robot to turn 180 degrees
            stage+=1;
        }
    }
}

void pole_to_pole_junction()
{
    if (stage==11)
    {
        while (!(IR_val[5]==0 && IR_val[4]==0 && IR_val[3]==0 && IR_val[2]==0))
        {
            line_follow();
        }
        else{
            turn_right();
            delay(200); // give enough delay for the robot to turn 90 degrees
            stage+=1;
        }
    }
}

void pole_junction_to_finish()
{
    if (stage==12)
    {
        while (!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 ))
        {
            line_follow();
        }
        else{
            forward();
            delay(100); // enough delay for the robot to go 10 cm
            stop();
            serial.Println("All tasks done!");
        }
    }
}