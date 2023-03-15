/********************************************************** 
-- Author: Mayowa Oladimeji
-- Date: 27/12/2022
-- Purpose: Fire + Gas Detection Robot Obstacle Avoidance Robot
-- How it works: The robot moves around and runs its obstacle avoidance routine. When fire is detected, it stops the car, keeps buzzing until fire 
is gone and resumes its routine. When gas leaks, ring buzzer until gas levels are back to normal. Both routines also send an SMS alert via the Wi-Fi module (simulated by Serial print statements.
1) The IR sensor simulates flame sensor and when any button is pressed 
on the remote, this simulates presence of fire,
2) DC motors represent the robot wheels,
3) Gas sensor simulated by potentiometer (for the analog readings). The potentiometer turned completely to the left is 0, and 1024 when turned completely to the right (clockwise rotation).
***********************************************************/

int enableRightWheels = 5;
int enableLeftWheels = 6;

int driverPin1a = 7;
int driverPin1b = 8;
int driverPin2a = 12;
int driverPin2b = 13;

const int trigPin = A4;
const int echoPin = A5;

const int rightTrigPin = A1;
const int rightEchoPin = A0;

int flamePin = 2;
int buzzer = 11;
int buzzerFreq = 512;
int gasAnalogPin = A3;
unsigned long lastGasMessageTime = 0;
unsigned long lastFireMessageTime = 0;

int leftWheelsSpeed;
int rightWheelsSpeed;
float frontDistanceInCM;
float sideDistanceInCM;


void setup() {
  pinMode(enableLeftWheels, OUTPUT);
  pinMode(enableRightWheels, OUTPUT);

  pinMode(driverPin1a, OUTPUT);
  pinMode(driverPin1b, OUTPUT);
  pinMode(driverPin2a, OUTPUT);
  pinMode(driverPin2b, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(rightTrigPin, OUTPUT);
  
 attachInterrupt(digitalPinToInterrupt(flamePin), flameDetection, RISING);

  leftWheelsSpeed = 120;
  rightWheelsSpeed = 120;
  
  Serial.begin(9600);
}


void loop() {
  frontDistanceInCM = checkDistance(trigPin, echoPin);
  sideDistanceInCM = checkDistance(rightTrigPin, rightEchoPin);
    
  noTone(buzzer);
  startRobot();
  checkForGasLeak();
  
  if(frontDistanceInCM < 30) {
    stopRobot();
    delay(100);
    moveBackwards();
    delay(200);
    if(sideDistanceInCM < 20) {
      turnLeftWithFullSpeed();
      delay(1000);
    }  
    else {
      turnRightWithFullSpeed();
      delay(1000);
    }
  }
  else {
    moveForward();  
  }
}

float checkDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float durationInMicro = pulseIn(echo, HIGH);
  return (durationInMicro*0.034)/2;
}

void checkForGasLeak() {
  if(analogRead(gasAnalogPin) > 512) {
  	tone(buzzer, buzzerFreq); 
    sendSMSNotification("Gas leak detected!!", "gas");
  }
}

void flameDetection() {
  stopRobot();
  tone(buzzer, buzzerFreq); 
  sendSMSNotification("Fire detected!", "fire");
}

void sendSMSNotification(String message, String type) {
  unsigned long currTime = millis()/1000; 
  if(type == "gas") {
    if(currTime - lastGasMessageTime > 10) {
      Serial.println(message);
      lastGasMessageTime = currTime;
    }
  }
  if(type == "fire") {
    if(currTime - lastFireMessageTime > 10) {
      Serial.println(message);
      lastFireMessageTime = currTime;
    }
  }
}

void startRobot() {
  analogWrite(enableLeftWheels, leftWheelsSpeed);
  analogWrite(enableRightWheels, rightWheelsSpeed);
}

void stopRobot() {
    digitalWrite(driverPin1a, LOW);
    digitalWrite(driverPin1b, LOW);

    digitalWrite(driverPin2a, LOW);
    digitalWrite(driverPin2b, LOW); 
}

void stopWheels() {
  analogWrite(enableLeftWheels, 0);
  analogWrite(enableRightWheels, 0);
}

void moveForward() {
    digitalWrite(driverPin1a, LOW);
    digitalWrite(driverPin1b, HIGH);
    
    digitalWrite(driverPin2a, LOW);
    digitalWrite(driverPin2b, HIGH);     
}

void moveBackwards() {
    digitalWrite(driverPin1a, HIGH);
    digitalWrite(driverPin1b, LOW);
    
    digitalWrite(driverPin2a, HIGH);
    digitalWrite(driverPin2b, LOW);  
}

void turnRight() {
    analogWrite(enableRightWheels, 0);
    analogWrite(enableLeftWheels, 250);
    
    digitalWrite(driverPin2a, LOW);
    digitalWrite(driverPin2b, HIGH); 
}

void turnRightWithFullSpeed() {
 digitalWrite(enableLeftWheels, HIGH);  
 digitalWrite(enableRightWheels, HIGH);
  
 digitalWrite(driverPin1a, LOW);  
 digitalWrite(driverPin1b, LOW);
  
 digitalWrite(driverPin2a, LOW); 
 digitalWrite(driverPin2b, HIGH);
}

void turnLeft() {
    analogWrite(enableRightWheels, 250);
    analogWrite(enableLeftWheels, 0); 

    digitalWrite(driverPin1a, LOW);
    digitalWrite(driverPin1b, HIGH);
}

void turnLeftWithFullSpeed(){
  digitalWrite(enableLeftWheels, HIGH);  
  digitalWrite(enableRightWheels, HIGH);
  
  digitalWrite(driverPin1a, LOW);  
  digitalWrite(driverPin1b, HIGH);
  
  digitalWrite(driverPin2a, LOW); 
  digitalWrite(driverPin2b, LOW); 
}
