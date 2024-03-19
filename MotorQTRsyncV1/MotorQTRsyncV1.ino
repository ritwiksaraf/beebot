/*
Version 0
Code for Testing PID control of movement
Components:
2 x DC geared motors with encoders
2 x L289N motor drivers
1 x QTR8RC IR array

Pinouts:
DC motor encoder:
Sr. | Device    | Pins | Arduino Pins
1)  | L298N-i   | enA  |
                  1
                  2
                  enB
                  3
                  4
2)  | L298N-ii  | enA  |
                  1
                  2
                  enB
                  3
                  4
3)  | QTR8RC    |      |

*/

/*
Behaviors to remember-

1) Wheel behaviors-
   --> wheel 2- O|__|O wheel 1
                 |  |
       wheel 4- O|__|O wheel 3

 The wheels as per the above picture are labeled as w1,w2,w3,w4.
 
*/

//QTR vars
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
#define num_sensors 8
const int w1 = 5;
const int w2 = 3;
const int w3= 9;
const int w4 = 6;
const int w11 = 8; 
const int w12 = 7	;
const int w13 = 2 ;
const int w14= 4;
const int w21 = 12; 
const int w22 = 13;
const int w23 = 11 ;
const int w24 = 10;

//vars for the PID
float kd, kp, ki;
int error, lasterror, motorspeed;

//vars for the timer
bool check = 1;


void setup() {
  Serial.begin(9600);

  //QTR calibration
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){3, 4, 5, 6, 7, 8, 9, 10}, SensorCount);
  qtr.setEmitterPin(2);
  Serial.println("Starting QTR Calibration");
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
    Serial.print(i);
  }
  delay(1000);
  Serial.println("Calibration Complete");


}

void loop() {


}




// Sample code for the bot we built using 4 motors, ultrasonic sensor and bluetooth controlled. You can utilize parts from this to make the final code
#include <NewPing.h>

// Libraries
#include <Servo.h>          
// Initialising the motor driver connections
const int LeftMotorForward = 6;
const int LeftMotorBackward = 7;
const int RightMotorForward = 8;
const int RightMotorBackward = 9;

// Defining Trigger and Echo Pin of Ultrasonic Sensor for New Ping Library
#define trig_pin 4 
#define echo_pin 5 

// Defining maximum distance 
#define maximum_distance 200

boolean goesForward = false;
int distance = 100;

// Defining connections for New Ping Library
NewPing sonar(trig_pin, echo_pin, maximum_distance); 

// Initialize Servo Motor
Servo servo_motor; 

// Initialize Bluetooth Command
char command; 

// Initialize Toggle Switch Pin
const int TSwitch = 2;

// Initialize Switch State
int SwitchState;

void setup() 
{
  // Set Output pins for Motor Driver
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  // Set Input pin for Switch Input
  pinMode(TSwitch, INPUT);

  // Setting Output pin for Servo Motor
  servo_motor.attach(3); 

  // Initializing Start Position
  servo_motor.write(90);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);

  // Initializing Serial Monitor
  Serial.begin(9600); 
}

void loop() 
{ 
  // Taking Switch state Input
  SwitchState = digitalRead(TSwitch);
  //Printing Switch State
  Serial.println(SwitchState);
  // Setting 2 modes of operation LOW = Bluetooth Mode, HIGH = Obstacle Avoider 
  if(SwitchState == LOW)
  { 
    // Bluetooth Code Begins
    if(Serial.available() > 0)
    { 
      command = Serial.read(); 
      Stop(); 
      switch(command)
      {
       case 'F':  
         forward1();
         break;
       case 'B':  
         back();
         break;
       case 'L':  
         left();
         break;
       case 'R':
         right();
         break;
       }
     }
   }

  if(SwitchState == HIGH) 
  {
    // Algorithm for Obstacle avoider 
    int distanceRight = 0;
    int distanceLeft = 0;
    delay(50);

    if (distance <= 30)
    {
      Stop();
      delay(300);
      back();
      delay(400);
      Stop();
      delay(300);
      distanceRight = lookRight();
      delay(300);
      distanceLeft = lookLeft();
      delay(300);

      if (distance >= distanceLeft)
      {
        right();
        delay(300);
        Stop();
      }
      else
      {
        left();
        delay(300);
        Stop();
      }
    }
    else
    {
      forward(); 
    }
      distance = readPing();
  }
}

int lookRight(){  
  servo_motor.write(10);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(90);
  return distance;
}

int lookLeft(){
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(90);
  return distance;
  delay(100);
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void Stop(){

  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void forward1()
{
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
}

void forward(){

  if(!goesForward){

    goesForward=true;

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
  }
}

void back(){

  goesForward=false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);

}

void right(){

  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(LeftMotorForward, LOW);

}

void left(){

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, HIGH);

  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
} 
