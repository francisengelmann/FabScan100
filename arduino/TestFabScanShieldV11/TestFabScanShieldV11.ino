// FabScan - http://hci.rwth-aachen.de/fabscan
//
// R. Bohne 30.12.2013
// This sketch tests all four stepper drivers on the FabScan-Shield V1.1
// It also turns on the light and the laser
// This sketch is not the real FabScan firmware, but just a test script for people who want to test their hardware!

#define LIGHT_PIN 17
#define LASER_PIN 18
#define MS_PIN    19

//Stepper 1 as labeled on Shield, Turntable
#define ENABLE_PIN_0  2
#define STEP_PIN_0    3
#define DIR_PIN_0     4

//Stepper 2, Laser Stepper
#define ENABLE_PIN_1  5
#define STEP_PIN_1    6
#define DIR_PIN_1     7

//Stepper 3, currently unused
#define ENABLE_PIN_2  11
#define STEP_PIN_2    12
#define DIR_PIN_2     13

//Stepper 4, currently unused
#define ENABLE_PIN_3  14
#define STEP_PIN_3    15
#define DIR_PIN_3     16

void setup() 
{ 
  pinMode(LASER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);

  pinMode(MS_PIN, OUTPUT);
  digitalWrite(MS_PIN, HIGH);  //HIGH for 16microstepping, LOW for no microstepping

  pinMode(ENABLE_PIN_0, OUTPUT);
  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(STEP_PIN_0, OUTPUT);

  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);

  pinMode(ENABLE_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);

  pinMode(ENABLE_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(STEP_PIN_3, OUTPUT);

  //enable turntable and laser steppers
  digitalWrite(ENABLE_PIN_0, LOW);  //HIGH to turn off
  digitalWrite(ENABLE_PIN_1, LOW);  //HIGH to turn off
  digitalWrite(ENABLE_PIN_2, LOW);  //LOW to turn on
  digitalWrite(ENABLE_PIN_3, LOW);  //LOW to turn on 

  digitalWrite(LIGHT_PIN, 1); //turn light on
  digitalWrite(LASER_PIN, 1); //turn laser on
} 

//loop just steps all four steppers. Attached motors should turn!
void loop() 
{
  digitalWrite(STEP_PIN_0, HIGH);
  digitalWrite(STEP_PIN_1, HIGH);
  digitalWrite(STEP_PIN_2, HIGH);
  digitalWrite(STEP_PIN_3, HIGH);
  delay(1);
  digitalWrite(STEP_PIN_0, LOW);
  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(STEP_PIN_2, LOW);
  digitalWrite(STEP_PIN_3, LOW);
  delay(1);
  
} 

