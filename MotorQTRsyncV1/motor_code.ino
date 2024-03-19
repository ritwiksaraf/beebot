

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
