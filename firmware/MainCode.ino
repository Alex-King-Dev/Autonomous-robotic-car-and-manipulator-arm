// Author: Alex King
// University of Sheffield
// Date: February 2026

#include <Servo.h> // import the servo library

//servo declaration
Servo Servo1; // joint 1 (shoulder)
Servo Servo2; // joint 2 (elbow)

//Pins
const int trigPin = 26;
const int echoPin = 24;

const int S0 = 28;
const int S1 = 30;
const int S2 = 32;
const int S3 = 34;
const int OUT_PIN = 36;

//encoder / motor values
const int ENC_K = 5450;
const int EncAPinA = 19;
const int EncAPinB = 18;
const int EncBPinA = 20;
const int EncBPinB = 21;

const int pinAI1 = 4;
const int pinAI2 = 3;
const int pinPWMA = 2;

const int pinBI1 = 6;
const int pinBI2 = 5;
const int pinPWMB = 7;

volatile long encCount1 = 0;
volatile long encCount2 = 0;
volatile float wheelAngle1 = 0;
volatile float wheelAngle2 = 0;

//servo pins
const int Servo1pin = 8;  
const int Servo2pin = 9;

//Kill switch pin
const int killSwitchPin = 47;

//Timings]
const int stopTime = 1000;
const int forwardTime = 200;    // small forward step while checking wall
const int reverseTime = 200;
const int forwardSegment1 = 1500;
const int forwardSegment2 = 1550;
const int reverseSegment1 = 50;

//Speeds
const int baseSpeed = 210;
float rightWheelSpeed;
float leftWheelSpeed;
const float Kp = 0.002;
float error = 0;
float correction = 0;

//Ultrasonic thresholds
const float firstWallThreshold  = 6.5;
const float secondWallThresholdHalt = 12;
const float secondWallThreshold = 11;
const float thirdWallThreshold = 8;
const float fourthWallThreshold = 10;

int state = 1;

//control Rate (servo update rate)
unsigned long timestamp;
unsigned long elapsed;
unsigned long controlRate = 20; // ms
const int servoStopTime = 500;

//time between arm trajectory points (WAYPOINTS)
const unsigned long waypointInterval = 125; // ms
unsigned long lastWaypointChange = 0;

//number of arm trajectory points
const int numPoints = 110;

//shoulder coordinate values
float thetaS1[numPoints] = {
    0.760, 0.739,
    0.718, 0.697,
    0.676, 0.655,
    0.634, 0.613,
    0.592, 0.571,
    0.550, 0.529,
    0.508, 0.487,
    0.466, 0.445,
    0.424, 0.403,
    0.382, 0.361,
    0.350, 0.340,

    0.345, 0.361,
    0.382, 0.403,
    0.424, 0.445,
    0.466, 0.487,
    0.508, 0.529,
    0.550, 0.571,
    0.592, 0.613,
    0.634, 0.655,
    0.676, 0.697,
    0.718, 0.739,
    0.750, 0.760
};

float thetaS2[numPoints] = {
    1.450, 1.390,
    1.340, 1.290,
    1.240, 1.190,
    1.140, 1.090,
    1.040, 0.990,
    0.940, 0.890,
    0.840, 0.790,
    0.740, 0.690,
    0.640, 0.580,
    0.500, 0.400,
    0.350, 0.330,

    0.350, 0.400,
    0.500, 0.580,
    0.640, 0.690,
    0.740, 0.790,
    0.840, 0.890,
    0.940, 0.990,
    1.040, 1.090,
    1.140, 1.190,
    1.240, 1.290,
    1.340, 1.380,
    1.420, 1.450
};

float thetaDropS2[numPoints] = {
    0.760, 0.8,
    0.9, 1.0,
    1.1, 1.2,
    1.3, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400,
    1.400, 1.400
};

float thetaDropS3[numPoints] = {
    1.45, 1.4,
    1.3, 1.3,
    1.3, 1.3,
    1.3, 1.3,
    1.3, 1.3,
    1.29, 1.29,
    1.29, 1.29,
    1.28, 1.28,
    1.28, 1.26,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
    1.25, 1.25,
};



//current commanded angles
float S1commanded = 0.0;
float S2commanded = 0.0;

//servo pulse width variables 
int S1pulsewidth;
int S2pulsewidth;

//radians per control interval
const float maxServoRate = 0.02;

//index of current trajectory point (0->10)
int trajIndex = 0;

//Function declarations
bool killSwitchOFF();
void Forward(int forwardTime);
void Reverse(int reverseTime);
void Stop(int stopTime);
void turnLeft(float targetAngle);
void turnRight(float targetAngle);
float getDistance();
void channelA1();
void channelA2();
void channelB1();
void channelB2();
void resetEncoders();
float getAngle1();
float getAngle2();
void penDraw();
void penDrop();


//Setup
void setup(){
  pinMode(pinBI1, OUTPUT);
  pinMode(pinBI2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

  pinMode(pinAI1, OUTPUT);
  pinMode(pinAI2, OUTPUT);
  pinMode(pinPWMA, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT_PIN, INPUT);

  pinMode(EncAPinA, INPUT_PULLUP);
  pinMode(EncAPinB, INPUT_PULLUP);
  pinMode(EncBPinA, INPUT_PULLUP);
  pinMode(EncBPinB, INPUT_PULLUP);

  encCount1 = 0;
  wheelAngle1 = 0;
  encCount2 = 0;
  wheelAngle2 = 0;

  attachInterrupt(digitalPinToInterrupt(EncAPinA), channelA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncAPinB), channelB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncBPinA), channelA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncBPinB), channelB2, CHANGE);

  pinMode(killSwitchPin, INPUT_PULLUP);


  timestamp = millis();
  lastWaypointChange = millis();

  Serial.begin(9600);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

//Main loop
void loop(){
  // If kill switch is pressed, stop immediately and do nothing else
  if (!killSwitchOFF()) {
    Stop(0);
    return;
  }

  encCount1 = 0;
  wheelAngle1 = 0;
  encCount2 = 0;
  wheelAngle2 = 0;

  float distance = getDistance();

  switch (state) {
    case 1: //spin 360
      Stop(2000);
      turnLeft(370);
      Stop(200);
      state ++;
      break;

    case 2: //slight left
      turnLeft(25);
      Stop(stopTime);
      state ++;
      break;

    case 3: //forward after left
      Forward(forwardSegment1);
      Stop(stopTime);
      state ++;
      break;

    case 4: //straight right
      turnRight(60);
      Stop(stopTime);
      state ++;
      break;

    case 5: //forward after right
      Forward(forwardSegment2);
      Stop(stopTime);
      state ++;
      break;

    case 6: //slight left back onto track
      turnLeft(17);
      Stop(stopTime);
      state ++;
      break;

    case 7: //drive to first wall
      Serial.print(distance);
      if (distance > firstWallThreshold) {
        Forward(forwardTime);
      } 
      else {
        Stop(stopTime);
        state ++;
      }
      break;

    case 8: //turn left 90
      turnLeft(90);
      Stop(stopTime);
      state ++;
      break;

    case 9: //drive to second wall
      if (distance > secondWallThresholdHalt) {
        Forward(forwardTime);
      } else {
        Stop(stopTime);

        Servo1.attach(Servo1pin);
        Servo2.attach(Servo2pin);
        S1commanded = limitServoRate(thetaS1[10],thetaS1[6], maxServoRate);
        S2commanded = limitServoRate(thetaS2[10],thetaS2[6], maxServoRate);
        moveServo(Servo1, 1300, S1commanded);
        delay(200);
        moveServo(Servo2, 1350, S2commanded);
        delay(200);
        Servo1.detach();
        Servo2.detach();

        state ++;
      }
      break;

    case 10: //drive to second wall
      if (distance > secondWallThreshold) {
        Forward(forwardTime);
      } else {
        Stop(stopTime);
        Servo1.attach(Servo1pin);
        Servo2.attach(Servo2pin);
        penDraw();
        Servo1.detach();
        Servo2.detach();
        state ++;
      }
      break;

    case 11: //stop at whiteboard
      Stop(2000);
      state ++;
      break;

    case 12: //reverse at whiteboard
      Reverse(reverseSegment1);

      Servo1.attach(Servo1pin);
      Servo2.attach(Servo2pin);
      S1commanded = limitServoRate(thetaS1[44],thetaS1[32], maxServoRate);
      S2commanded = limitServoRate(thetaS2[44],thetaS2[32], maxServoRate);
      moveServo(Servo1, 1300, S1commanded);
      delay(200);
      moveServo(Servo2, 1350, S2commanded);
      delay(200);
      Servo1.detach();
      Servo2.detach();

      Stop(stopTime);
      state ++;
      break;

    case 13: //left turn
      turnLeft(90);
      Stop(stopTime);
      state ++;
      break;

    case 14: // forward
      if (distance > thirdWallThreshold) {
        Forward(forwardTime);
      } else {
        Stop(stopTime);
        state ++;
      }
      break;
    
    case 15: //left turn
      turnLeft(80);
      Stop(stopTime);
      state ++;
      break;
    
    case 16: // forward
       if (distance > fourthWallThreshold) {
        Forward(forwardTime);
      } else {
        Stop(stopTime);
        state ++;
      }
      break;
 
    case 17: //left turn in yellow zone
      turnLeft(80);
      Stop(stopTime);
      state ++;
      break;

    case 18: //finished
      Stop(0);
      break;
  }
}


//Motor functions
void Forward(int forwardTime){
  Serial.println("FORWARD");

  resetEncoders();

  // Right wheel forward
  digitalWrite(pinBI1, HIGH);
  digitalWrite(pinBI2, LOW);
  analogWrite(pinPWMB, baseSpeed);

  // Left wheel forward
  digitalWrite(pinAI1, HIGH);
  digitalWrite(pinAI2, LOW);
  analogWrite(pinPWMA, baseSpeed);

  delay(forwardTime);
}

void Reverse(int reverseTime){
  Serial.println("REVERSE");

  resetEncoders();

  // Right wheel reverse
  digitalWrite(pinBI1, LOW);
  digitalWrite(pinBI2, HIGH);
  analogWrite(pinPWMB, baseSpeed);

  // Left wheel reverse
  digitalWrite(pinAI1, LOW);
  digitalWrite(pinAI2, HIGH);
  analogWrite(pinPWMA, baseSpeed);

  delay(reverseTime);
}

void Stop(int stopTime){
  Serial.println("STOP");

  resetEncoders();

  // Right wheel stopped
  digitalWrite(pinBI1, LOW);
  digitalWrite(pinBI2, LOW);
  analogWrite(pinPWMB, 0);

  // Left wheel stopped
  digitalWrite(pinAI1, LOW);
  digitalWrite(pinAI2, LOW);
  analogWrite(pinPWMA, 0);

  delay(stopTime);
}

void turnLeft(float targetAngle){
  Serial.println("TURN LEFT");

  resetEncoders();
  error = 0;
  correction = 0;

  while(abs((getAngle1() + getAngle2()) / 2.0) < targetAngle){
    //PID wheel speed control
    error = encCount1 - encCount2;
    correction = Kp * error;

    rightWheelSpeed = baseSpeed - correction;
    leftWheelSpeed  = baseSpeed + correction;

    Serial.println(leftWheelSpeed);
    Serial.println(rightWheelSpeed);
    Serial.println(correction);
    Serial.println("-----");

    // Right wheel forward
    digitalWrite(pinBI1, HIGH);
    digitalWrite(pinBI2, LOW);
    analogWrite(pinPWMB, rightWheelSpeed);

    // Left wheel backward
    digitalWrite(pinAI1, LOW);
    digitalWrite(pinAI2, HIGH);
    analogWrite(pinPWMA, leftWheelSpeed);
    //turning
  }

  Stop(0);
}

void turnRight(float targetAngle){
  Serial.println("TURN RIGHT");

  resetEncoders();
  error = 0;
  correction = 0;

  while(abs((getAngle1() + getAngle2()) / 2.0) < targetAngle){
    //PID wheel speed control
    error = encCount1 - encCount2;
    correction = Kp * error;

    rightWheelSpeed = baseSpeed - correction;
    leftWheelSpeed  = baseSpeed + correction;
    
    // Right wheel backward
    digitalWrite(pinBI1, LOW);
    digitalWrite(pinBI2, HIGH);
    analogWrite(pinPWMB, rightWheelSpeed);

    // Left wheel forward
    digitalWrite(pinAI1, HIGH);
    digitalWrite(pinAI2, LOW);
    analogWrite(pinPWMA, leftWheelSpeed);
    //turning
  }

  Stop(0);
}

//Returns true when kill switch is on
bool killSwitchOFF(){
  return digitalRead(killSwitchPin) == HIGH;
}

//Ultrasonic function
float getDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return 999.0;
  }

  float distance = duration * 0.034 / 2.0;
  return distance;
}


//inverse kinematic functions
float limitServoRate(float currentAngle, float targetAngle, float maxDelta){
  if (targetAngle > currentAngle + maxDelta) {
    return currentAngle + maxDelta;
  }
  if (targetAngle < currentAngle - maxDelta) {
    return currentAngle - maxDelta;
  }
  return targetAngle;
}

//servo movement function
int moveServo(Servo servo, int refPulseWidth, float servoRadAngle){
  const float Pi = 3.142;
  float minPulseWidth = float(refPulseWidth) - 1000.0;
  int cmdSignal = (servoRadAngle + (Pi / 2.0)) * (2000.0 / Pi) + minPulseWidth;
  servo.writeMicroseconds(cmdSignal);
  return cmdSignal;
}

//encoder interupts
void channelA1(){
  if (digitalRead(EncAPinA) == digitalRead(EncAPinB)) {
    encCount1++;
  } else {
    encCount1--;
  }

  wheelAngle1 = (360.0 * encCount1) / ENC_K;
}

void channelA2(){
  if (digitalRead(EncAPinA) != digitalRead(EncAPinB)) {
    encCount1++;
  } else {
    encCount1--;
  }

  wheelAngle1 = (360.0 * encCount1) / ENC_K;
}

void channelB1(){
  if (digitalRead(EncBPinA) == digitalRead(EncBPinB)) {
    encCount2++;
  } else {
    encCount2--;
  }

  wheelAngle2 = (360.0 * encCount2) / ENC_K;
}

void channelB2(){
  if (digitalRead(EncBPinA) != digitalRead(EncBPinB)) {
    encCount2++;
  } else {
    encCount2--;
  }

  wheelAngle2 = (360.0 * encCount2) / ENC_K;
}

//reset encoders
void resetEncoders(){
  encCount1 = 0;
  encCount2 = 0;
}

void penDraw(){
  while (trajIndex < 44){
    unsigned long now = millis();

    //1) Change waypoint every waypointInterval ms
    if ((now - lastWaypointChange) >= waypointInterval){
      lastWaypointChange = now;

      if (trajIndex < numPoints - 1) {
        //still going DOWN the path
        trajIndex++;
        Serial.print(trajIndex);
      }
    }

    float S1radangle = thetaS1[trajIndex];   // from table
    float S2radangle = thetaS2[trajIndex];   // from table

    elapsed = now - timestamp;
    if ( elapsed > controlRate ) {

      S1commanded = limitServoRate(S1commanded, S1radangle, maxServoRate);
      S1pulsewidth = moveServo(Servo1, 1300, S1commanded);

      S2commanded = limitServoRate(S2commanded, S2radangle, maxServoRate);
      S2pulsewidth = moveServo(Servo2, 1350, S2commanded);

      timestamp = now;
    }
  }
}

void penDrop(){
  while (trajIndex < 44){
    unsigned long now = millis();

    //1) Change waypoint every waypointInterval ms
    if ((now - lastWaypointChange) >= waypointInterval){
      lastWaypointChange = now;

      if (trajIndex < numPoints - 1) {
        //still going DOWN the path
        trajIndex++;
        Serial.print(trajIndex);
      }
    }

    float S1radangle = thetaDropS1[trajIndex];   // from table
    float S2radangle = thetaDropS2[trajIndex];   // from table

    elapsed = now - timestamp;
    if ( elapsed > controlRate ) {

      S1commanded = limitServoRate(S1commanded, S1radangle, maxServoRate);
      S1pulsewidth = moveServo(Servo1, 1300, S1commanded);

      S2commanded = limitServoRate(S2commanded, S2radangle, maxServoRate);
      S2pulsewidth = moveServo(Servo2, 1350, S2commanded);

      timestamp = now;
    }
  }
}

//encoder angles
float getAngle1(){
  return (360.0 * encCount1) / ENC_K;
}

float getAngle2(){
  return (360.0 * encCount2) / ENC_K;
}