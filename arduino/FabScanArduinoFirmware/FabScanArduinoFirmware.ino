// FabScan - http://hci.rwth-aachen.de/fabscan
//
//  Created by Francis Engelmann on 7/1/11.
//  Copyright 2011 Media Computing Group, RWTH Aachen University. All rights reserved.

#define LED_PIN 17
#define LASER_PIN 18
#define MS_PIN 19 //microstepping
#define LIGHT_PIN 10 //still needed?

//turntable, socket 3
#define ENABLE_PIN_0  11
#define DIR_PIN_0     13
#define STEP_PIN_0    12
#define RESET_PIN_0   7
#define SLEEP_PIN_0   8

//other one (unused)
#define ENABLE_PIN_1  2
#define DIR_PIN_1     3
#define STEP_PIN_1    4
#define RESET_PIN_1   6
#define SLEEP_PIN_1   9

  
#define TURN_LASER_OFF      200
#define TURN_LASER_ON       201
#define PERFORM_STEP        202
#define SET_DIRECTION_CW    203
#define SET_DIRECTION_CCW   204
#define TURN_STEPPER_ON     205
#define TURN_STEPPER_OFF    206
#define TURN_LIGHT_ON       207
#define TURN_LIGHT_OFF      208
#define ROTATE_LASER        209
#define FABSCAN_PING        210
#define FABSCAN_PONG        211
#define SELECT_STEPPER      212

//the protocol: we send one byte to define the action what to do.
//If the action is unary (like turnung off the light) we only need one byte so we are fine.
//If we want to tell the stepper to turn, a second byte is used to specify the number of steps.
//These second bytes are defined here below.

#define ACTION_BYTE         1    //normal byte, first of new action
#define LIGHT_INTENSITY     2
#define TURN_TABLE_STEPS    3
#define LASER1_STEPS        4
#define LASER2_STEPS        5
#define LASER_ROTATION      6
#define STEPPER_ID          7

int incomingByte = 0;
int byteType = 1;
int currStepper;

void step()
{
 digitalWrite(LED_PIN, 0);
 
 if(currStepper == 0){
   digitalWrite(STEP_PIN_0, 0);
 }else if(currStepper == 1){
   digitalWrite(STEP_PIN_1, 0);
 }

 delay(3);
 if(currStepper == 0){
   digitalWrite(STEP_PIN_0, 1);
 }else if(currStepper == 1){
   digitalWrite(STEP_PIN_1, 1);
 }
 digitalWrite(LED_PIN, 1);
 delay(3);
}

void step(int count)
{
  for(int i=0; i<count; i++){
    step();
  }
}

void setup() 
{ 
  // initialize the serial port
  Serial.begin(9600);
 pinMode(LASER_PIN, OUTPUT);
 pinMode(LIGHT_PIN, OUTPUT);
 pinMode(LED_PIN, OUTPUT);
 
 pinMode(ENABLE_PIN_1, OUTPUT);
 pinMode(RESET_PIN_1, OUTPUT);
 pinMode(SLEEP_PIN_1, OUTPUT);
 pinMode(DIR_PIN_1, OUTPUT);
 pinMode(STEP_PIN_1, OUTPUT);
 
 pinMode(ENABLE_PIN_0, OUTPUT);
 pinMode(RESET_PIN_0, OUTPUT);
 pinMode(SLEEP_PIN_0, OUTPUT);
 pinMode(DIR_PIN_0, OUTPUT);
 pinMode(STEP_PIN_0, OUTPUT);
  
 digitalWrite(RESET_PIN_0, HIGH);  //HIGH to turn
 digitalWrite(SLEEP_PIN_0, HIGH);  //HIGH to turn
 digitalWrite(ENABLE_PIN_0, HIGH);  //LOW to turn
  
 //digitalWrite(RESET_PIN_1, HIGH);  //HIGH to turn
 //digitalWrite(SLEEP_PIN_1, HIGH);  //HIGH to turn
 //digitalWrite(ENABLE_PIN_1, HIGH);  //LOW to turn
 
 digitalWrite(LIGHT_PIN, 1); //turn light on

 digitalWrite(LASER_PIN, 1); //turn laser on
 Serial.write(FABSCAN_PONG); //send a pong back to the computer so we know setup is done and that we are actually dealing with a FabScan
 
 currStepper = 0;  //0==turntable is default stepper, change here if turntable and laser are incorrectly connected and not possible to easily change it

  digitalWrite(MS_PIN, 0);
  
} 

void loop() 
{
  digitalWrite(LED_PIN, 0);
  if(Serial.available() > 0){
    
    digitalWrite(LED_PIN, 1);
    incomingByte = Serial.read();
    //Serial.write("a");

    switch(byteType){
      case ACTION_BYTE:
          switch(incomingByte){    //this switch always handles the first byte
            //Laser
            case TURN_LASER_OFF:
              digitalWrite(LASER_PIN, LOW);    // turn the LASER off
              break;
            case TURN_LASER_ON:
              digitalWrite(LASER_PIN, HIGH);   // turn the LASER on
              break;
            case ROTATE_LASER: //unused
              byteType = LASER_ROTATION;
              break;
            //TurnTable
            case PERFORM_STEP:
              byteType = TURN_TABLE_STEPS;
              break;
            case SET_DIRECTION_CW:
              if(currStepper == 0){
                digitalWrite(DIR_PIN_0, 1);
              }else if(currStepper == 1){
                digitalWrite(DIR_PIN_1, 1);
              }
              break;
            case SET_DIRECTION_CCW:
              if(currStepper == 0){
                digitalWrite(DIR_PIN_0, 0);
              }else if(currStepper == 1){
                digitalWrite(DIR_PIN_1, 0);
              }
              break;
            case TURN_STEPPER_ON:
              if(currStepper == 0){
                digitalWrite(ENABLE_PIN_0, 0);
              }else if(currStepper == 1){
                digitalWrite(ENABLE_PIN_1, 0);
              }
              break;
            case TURN_STEPPER_OFF:
              if(currStepper == 0){
                digitalWrite(ENABLE_PIN_0, 1);
              }else if(currStepper == 1){
                digitalWrite(ENABLE_PIN_1, 1);
              }
              break;
            case TURN_LIGHT_ON:
              byteType = LIGHT_INTENSITY;
              break;
            case TURN_LIGHT_OFF:
              digitalWrite(LIGHT_PIN, 0);
              break;
            case FABSCAN_PING:
              delay(1);
              Serial.write(FABSCAN_PONG);
              break;
            case SELECT_STEPPER:
              byteType = STEPPER_ID;
              break;
            }
      
          break;
       case LIGHT_INTENSITY:       //after this point we take care of the second byte if one is sent
          analogWrite(LIGHT_PIN, incomingByte);
          byteType = ACTION_BYTE;  //reset byteType
          break;
        case TURN_TABLE_STEPS:
          step(incomingByte);
          byteType = ACTION_BYTE;
          break;
        case STEPPER_ID:
          currStepper = incomingByte;
          byteType = ACTION_BYTE;
          break;
    }
  }
} 
