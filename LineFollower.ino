/* 
  Line Follower
  12/06/2020
  Aaron Oliver-Taylor


  Pin Mapping
  Microcontroller    Description
  D00         UART TX
  D01         UART RX
  D02         
  D03~        HBRIDGE_IN4 (RB)
  D04         
  D05~        HBRIDGE_IN3 (RA)
  D06~        HBRIDGE_IN2 (LB)
  D07         NEOPIXEL_DATA_IN
  D08         
  D09~        HBRIDGE_IN1 (LA)
  D10~        LINE_DETECTOR_L 
  D11~        LINE_DETECTOR_C
  D12         LINE_DETECTOR_R
  D13         
  
  A0          
  A1          
  A2          
  A3          
  A5    
  
 */

#include <Adafruit_NeoPixel.h>
#include "PidController.h"

#define MOTOR_FORWARD 0
#define MOTOR_REVERSE 1
#define MODE_NORMAL 0
#define MODE_NOSIGNAL 1
#define HEADING_UNKNOWN 999
#define MOTOR_MAXSPEED 255
#define MOTOR_MINSPEED 32

uint8_t pinMotorRA = 5;
uint8_t pinMotorRB = 3;
uint8_t pinMotorLA = 9;
uint8_t pinMotorLB = 6;
uint8_t pinLineDetectorL = 10;
uint8_t pinLineDetectorC = 11;
uint8_t pinLineDetectorR = 8;
uint8_t pinNeoPixelDataOut = 7;

uint8_t motorSpeedR = MOTOR_MAXSPEED;
uint8_t motorSpeedL = MOTOR_MAXSPEED;
uint8_t motorDirectionR = MOTOR_FORWARD;
uint8_t motorDirectionL = MOTOR_FORWARD;



uint8_t lineSensorL = 0;
uint8_t lineSensorC = 0;
uint8_t lineSensorR = 0;

uint8_t numNeoPixels = 12;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numNeoPixels, pinNeoPixelDataOut, NEO_GRB + NEO_KHZ800);

uint32_t tSerial = 0;
uint16_t serialUpdatePeriod = 1000;

uint32_t tMotorControl = 0;
uint16_t motorControlUpdatePeriod = 50;

uint32_t tLineSensorSample = 0;
uint16_t lineSensorSamplePeriod = 50;

uint32_t tNoSignal = 0;
bool doNoSignalTimer = false;
uint16_t noSignalTimeout = 1000;

uint32_t tSpiral = 0;
uint16_t spiralIncrementPeriod = 500;

uint32_t tPid = 0;
uint16_t pidUpdatePeriod = 50;

float kp = 0.1;
float ki = 0.0001;
float kd = 0.001;
float setpoint = 0;
float heading = 0;
float pidOutput = 0;

PidController headingPid(kp, ki, kd, setpoint, lineSensorSamplePeriod);

uint8_t mode = MODE_NORMAL;

uint16_t hue = 0;
uint8_t saturation = 255;
uint8_t value = 10; 

float speedFraction; 



void setup() {
  // put your setup code here, to run once:
  pinMode(pinMotorRA, OUTPUT);
  pinMode(pinMotorRB, OUTPUT);
  pinMode(pinMotorLA, OUTPUT);
  pinMode(pinMotorLB, OUTPUT);
  pinMode(pinNeoPixelDataOut, OUTPUT);
  pinMode(pinLineDetectorL, INPUT_PULLUP);
  pinMode(pinLineDetectorC, INPUT_PULLUP);
  pinMode(pinLineDetectorR, INPUT_PULLUP);
  digitalWrite(pinMotorRA, LOW);
  digitalWrite(pinMotorRB, LOW);
  digitalWrite(pinMotorLA, LOW);
  digitalWrite(pinMotorLB, LOW);

  headingPid.SetOutputLimits(-1.0, 1.0);
  Serial.begin(115200);
  tSerial = millis();
  tMotorControl = millis();
  tLineSensorSample = millis();
  speedFraction = (MOTOR_MAXSPEED - MOTOR_MINSPEED)/MOTOR_MAXSPEED;
}

void loop() {
  /* control the motors */
  if (millis() > tMotorControl + motorControlUpdatePeriod){
    tMotorControl += motorControlUpdatePeriod;
    driveMotor(pinMotorRA, pinMotorRB, motorDirectionR,  motorSpeedR);
    driveMotor(pinMotorLA, pinMotorLB, motorDirectionL,  motorSpeedL);

    uint32_t rgbcolor = pixels.ColorHSV(hue, saturation, value);

    for (uint8_t i = 0; i < numNeoPixels; i++){
      pixels.setPixelColor(i, rgbcolor);
    }
    pixels.show();
    hue = hue += 64;
    

  }

  /* sample the line detector array */
  if (millis() > tLineSensorSample + lineSensorSamplePeriod){
    tLineSensorSample += lineSensorSamplePeriod;
    heading = doHeading();
  }

    
  switch(mode){
    /* a line is detected, use PID to control heading */
    case MODE_NORMAL: 
      if (millis() > tPid + pidUpdatePeriod){
        if (heading != HEADING_UNKNOWN){
          doNoSignalTimer = false;
          pidOutput = headingPid.AddSample(heading);
          float speedMult = abs(pidOutput);
          if (pidOutput < 0){
            motorSpeedR = (1- speedMult)*MOTOR_MAXSPEED;
            motorSpeedL = (speedMult)*MOTOR_MAXSPEED;
          }else if (pidOutput > 0){
            motorSpeedL = (1- speedMult)*MOTOR_MAXSPEED;
            motorSpeedR = (speedMult)*MOTOR_MAXSPEED;
          }
        }
      }

      if(heading == HEADING_UNKNOWN){
        if(doNoSignalTimer == false){
          tNoSignal = millis();
          //doNoSignalTimer = true;
        }
      }

      if (doNoSignalTimer == true){
        if (millis() > tNoSignal + noSignalTimeout){
          mode = MODE_NOSIGNAL;
          motorSpeedL = 0;
          motorSpeedR = 255;
          tSpiral = millis();
        }
      }

      break;
    /* no line detected  move robot in anti-clockwise spiral */
    case MODE_NOSIGNAL:
      if (millis() > tSpiral + spiralIncrementPeriod){
        tSpiral += spiralIncrementPeriod;
        motorSpeedL += 5;
      }

      if (heading != HEADING_UNKNOWN){
        mode = MODE_NORMAL;
      }

      break;
    
    default:
      break;
  }




  if (millis() > tSerial + serialUpdatePeriod){
    tSerial += serialUpdatePeriod;
    Serial.print(digitalRead(pinLineDetectorL));
    Serial.print('\t');
    Serial.print(digitalRead(pinLineDetectorC));
    Serial.print('\t');
    Serial.print(digitalRead(pinLineDetectorR));
    Serial.print('\t');
    Serial.print(heading);
    Serial.print('\t');
    Serial.print(mode);
    Serial.print('\t');
    Serial.print(pidOutput);
    Serial.print('\t');
    Serial.print(motorSpeedR);
    Serial.print('\t');
    Serial.print(motorSpeedL);
    Serial.println();
  }

}

void driveMotor(uint8_t pinA, uint8_t pinB, uint8_t direction, uint8_t speed){
  if (direction == MOTOR_FORWARD){
      digitalWrite(pinB, LOW);
      analogWrite(pinA, speed);
  }else if (direction == MOTOR_REVERSE){
     digitalWrite(pinA, LOW);
    analogWrite(pinB, speed);
  }
}

float doHeading(void){
  lineSensorL = digitalRead(pinLineDetectorL);
  lineSensorC = digitalRead(pinLineDetectorC);
  lineSensorR = digitalRead(pinLineDetectorR);
  if(lineSensorL == 1 && lineSensorC == 0 && lineSensorR == 0){
    heading = -1;

  }else if (lineSensorL == 1 && lineSensorC == 1 && lineSensorR == 0){
    heading = -0.5;
    
  }else if (lineSensorL == 0 && lineSensorC == 1 && lineSensorR == 0){
    heading = 0;
    
  }else if (lineSensorL == 0 && lineSensorC == 1 && lineSensorR == 1){
    heading = 0.5;
    
  }else if (lineSensorL == 0 && lineSensorC == 0 && lineSensorR == 1){
    heading = 1;

  }else if (lineSensorL == 0 && lineSensorC == 0 && lineSensorR == 0){
    if (doNoSignalTimer == false){
      heading = HEADING_UNKNOWN;
    }
  }
  return heading;
}
