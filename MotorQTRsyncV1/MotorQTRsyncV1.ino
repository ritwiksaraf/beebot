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

error = 
