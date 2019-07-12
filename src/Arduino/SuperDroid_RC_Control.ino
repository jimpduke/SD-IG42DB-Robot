#include <Wire.h> 
#include "PWM.hpp"

// Set one for serial debug statements 0 for no serial debug statements
const int rc_debug = 0; 
const int drive_debug = 0;

//RadioLink AT10 II RC Controler input PIN connections
const int channel_1 = 18;  // Right Stick Horizontal
const int channel_2 = 19;  // Right Stick Vertical
const int channel_3 = 20;  // Left Stick Vertical
const int channel_4 = 21;  // Left Stick Horizontal

// Wait Time
const int PollWaitTime = 5;

// Deadzone +- 20 pulses
const int deadzone = 20;

// Movement:
//      Motor Stop: 
//           INA = LOW
//           INB = LOW
const int Stop = LOW;

//      Right Side Forward:
//           INA = HIGH
//           INB = LOW
const int RightForwardA = LOW;
const int RightForwardB = HIGH;

//      Right Side Backward:
//           INA = LOW
//           INB = HIGH
const int RightBackwardA = HIGH;
const int RightBackwardB = LOW;

//      Left Side Forward:
//           INA = LOW
//           INB = HIGH
const int LeftForwardA = HIGH;
const int LeftForwardB = LOW;

//      Left Side Backward:
//           INA = HIGH
//           INB = LOW
const int LeftBackwardA = LOW;
const int LeftBackwardB = HIGH;

// Front Left Wheel Pins
const int FrontLeftinAPin = A1;
const int FrontLeftinBPin = A0;
// PWMPin controls speed
const int FrontLeftPWMPin = 5;

// Front Right Wheel Pins
const int FrontRightinAPin = A3;
const int FrontRightinBPin = A2;
// PWMPin controls speed
const int FrontRightPWMPin = 9;

// Rear Left Wheel Pins
const int RearLeftinAPin = 4;
const int RearLeftinBPin = 2;
// PWMPin controls speed
const int RearLeftPWMPin = 3;

// Rear Right Wheel Pins
const int RearRightinAPin = 8;
const int RearRightinBPin = 7;
// PWMPin controls speed
const int RearRightPWMPin = 6;

// RC channel Input
int channel_1_in;
int channel_2_in;
int channel_3_in;
int channel_4_in;

// PWM objects for each RC input channel
PWM ch1(18);
PWM ch2(19);
PWM ch3(20);
PWM ch4(21);

void setup() {
  Serial.begin (9600);
  
  pinMode(FrontLeftinAPin, OUTPUT);
  pinMode(FrontLeftinBPin, OUTPUT);
  pinMode(FrontLeftPWMPin, OUTPUT);

  pinMode(FrontRightinAPin, OUTPUT);
  pinMode(FrontRightinBPin, OUTPUT);
  pinMode(FrontRightPWMPin, OUTPUT);
  
  pinMode(RearRightinAPin, OUTPUT);
  pinMode(RearRightinBPin, OUTPUT);
  pinMode(RearRightPWMPin, OUTPUT);
  
  pinMode(RearLeftinAPin, OUTPUT);
  pinMode(RearLeftinBPin, OUTPUT);
  pinMode(RearLeftPWMPin, OUTPUT);

  // On Startup ensure all motors are stopped
  digitalWrite(FrontLeftinAPin, Stop); 
  digitalWrite(FrontLeftinBPin, Stop);
  digitalWrite(FrontRightinAPin, Stop); 
  digitalWrite(FrontRightinBPin, Stop);
  digitalWrite(RearRightinAPin, Stop); 
  digitalWrite(RearRightinBPin, Stop);
  digitalWrite(RearLeftinAPin, Stop); 
  digitalWrite(RearLeftinBPin, Stop); 

  //Setup RC receiver Interrupt based input
  ch1.begin(true); // ch1 on pin 18 reading PWM HIGH duration
  ch2.begin(true); // ch2 on pin 19 reading PWM HIGH duration
  ch3.begin(true); // ch3 on pin 20 reading PWM HIGH duration
  ch4.begin(true); // ch4 on pin 21 reading PWM HIGH duration
}

void loop() {
  //Get RC PWM signals and convert to Motor PWM ranges
  channel_1_in = 0;
  channel_1_in = ch1.getValue();
  channel_1_in = pulseToPWM(channel_1_in);
  
  channel_2_in = 0;
  channel_2_in = ch2.getValue();
  channel_2_in = pulseToPWM(channel_2_in);

  channel_3_in = 0;
  channel_3_in = ch3.getValue();
  channel_3_in = pulseToPWM(channel_3_in);

  channel_4_in = 0;
  channel_4_in = ch4.getValue();
  channel_4_in = pulseToPWM(channel_4_in);

  if (rc_debug)
  {
    Serial.print("RC Channel 1 in: ");
    Serial.print(channel_1_in);
    Serial.print(",  RC Channel 2 in: ");
    Serial.print(channel_2_in);
    Serial.print(", RC Channel 3 in: ");
    Serial.print(channel_3_in);
    Serial.print(", RC Channel 4 in: ");
    Serial.println(channel_4_in);
  }

  // Move Robot
  DriveMecanum (channel_2_in, channel_4_in, channel_1_in);
      
  delay(PollWaitTime);
}

//Stop 
void StopAll (){
  digitalWrite(FrontLeftinAPin, Stop); 
  digitalWrite(FrontLeftinBPin, Stop);
  digitalWrite(FrontRightinAPin, Stop); 
  digitalWrite(FrontRightinBPin, Stop);
  digitalWrite(RearRightinAPin, Stop); 
  digitalWrite(RearRightinBPin, Stop);
  digitalWrite(RearLeftinAPin, Stop); 
  digitalWrite(RearLeftinBPin, Stop);
}

// TODO Better scaling of mixed values
// TODO replace mixing with Vector movement based on Mecanum kinematics mathematics
//
void DriveMecanum (int driveVal, int turnVal, int strafeVal) {

  if ((driveVal == 0) && (turnVal == 0) && (strafeVal == 0)){
    StopAll();
    return;
  }

  // Mix RC axis for all wheels
  int FL = (driveVal + turnVal + strafeVal);
  int RL = (driveVal + turnVal - strafeVal);
  int FR = (driveVal - turnVal - strafeVal);
  int RR = (driveVal - turnVal + strafeVal);

  FL = constrain(FL, -255, 255);
  RL = constrain(RL, -255, 255);
  FR = constrain(FR, -255, 255);  
  RR = constrain(RR, -255, 255);

  // Set motor directions
  if (FR > 0){
    digitalWrite(FrontRightinAPin, RightForwardA); 
    digitalWrite(FrontRightinBPin, RightForwardB);
  }
  else if (FR < 0){
    digitalWrite(FrontRightinAPin, RightBackwardA); 
    digitalWrite(FrontRightinBPin, RightBackwardB);
  }
  else {
    digitalWrite(FrontRightinAPin, Stop); 
    digitalWrite(FrontRightinBPin, Stop);
  }

  if (RR > 0){
    digitalWrite(RearRightinAPin, RightForwardA); 
    digitalWrite(RearRightinBPin, RightForwardB);
  }
  else if (RR < 0){
    digitalWrite(RearRightinAPin, RightBackwardA); 
    digitalWrite(RearRightinBPin, RightBackwardB);
  }
  else {
    digitalWrite(RearRightinAPin, Stop); 
    digitalWrite(RearRightinBPin, Stop);
  }

  if (FL > 0){
    digitalWrite(FrontLeftinAPin, LeftForwardA); 
    digitalWrite(FrontLeftinBPin, LeftForwardB);
  }
  else if (FL < 0){
    digitalWrite(FrontLeftinAPin, LeftBackwardA); 
    digitalWrite(FrontLeftinBPin, LeftBackwardB);
  }
  else {
    digitalWrite(FrontLeftinAPin, Stop); 
    digitalWrite(FrontLeftinBPin, Stop);
  }

  if (RL > 0){
    digitalWrite(RearLeftinAPin, LeftForwardA); 
    digitalWrite(RearLeftinBPin, LeftForwardB);
  }
  else if (RL < 0){
    digitalWrite(RearLeftinAPin, LeftBackwardA); 
    digitalWrite(RearLeftinBPin, LeftBackwardB);
  }
  else {
    digitalWrite(RearLeftinAPin, Stop); 
    digitalWrite(RearLeftinBPin, Stop);
  }
    
  // Set Motor Speeds
  analogWrite(FrontRightPWMPin, abs(FR));
  analogWrite(RearRightPWMPin, abs(RR));
  analogWrite(FrontLeftPWMPin, abs(FL));
  analogWrite(RearLeftPWMPin, abs(RL));
 
  if(drive_debug){
      Serial.print("FR: "); 
      Serial.print (FR);
      Serial.print(", RR: ");
      Serial.print(RR);
      Serial.print(", FL: ");
      Serial.print(FL);
      Serial.print(", RL: ");
      Serial.println(RL);
  }
}

// Convert RC pulse value to motor PWM value
int pulseToPWM(int pulse) {
  
  // If we're receiving numbers, convert them to motor PWM
  if ( pulse > 1000 ) {
    pulse = map(pulse, 1000, 2000, -350, 350);
    pulse = constrain(pulse, -255, 255);
  } else {
    pulse = 0;
  }

  // Anything in deadzone should stop the motor
  if ( abs(pulse) <= deadzone ) {
    pulse = 0;
  }

  return pulse;
}
