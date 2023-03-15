/********************************************************** 
-- Author: Mayowa Oladimeji
-- Date: 27/12/2022
-- Purpose: Fire + Gas Detection Obstacle Avoiding Robot
-- Register Manipulation version
-- How it works: The robot moves around and runs its obstacle avoidance routine. When fire is detected, it stops the car, keeps buzzing until fire is gone and resumes its routine(using interrupt pin). When gas leaks, ring buzzer until gas levels are back to normal. Both routines also send an SMS alert via the Wi-Fi module (simulated by Serial print statements.
**********************************************************/

byte enableRightWheels = 5;
byte enableLeftWheels = 6;

//right wheels - 1a/1b, left wheels - 2a/2b
byte driverPin1a = 7;
byte driverPin1b = 8;
byte driverPin2a = 12;
byte driverPin2b = 13;
byte trigPin = A4;
byte echoPin = A5;
byte rightTrigPin = A1;
byte rightEchoPin = A0;

byte flamePin = 2;
byte buzzer = 11;
volatile int buzzerFreq = 512;
int gasAnalogPin = A3;

//time count to only send message in intervals
unsigned long lastGasMessageTime = 0;
unsigned long lastFireMessageTime = 0;

//changed to bytes since the max. speed being set in the program is 250, no need for int, also saves space
byte leftWheelsSpeed;
byte rightWheelsSpeed;

void setup() {
  //pin2 - input, pins 5,6,7 - output
  DDRD = B11100000;

  //pins 8, 12, 13 - output, buzzer status doesn't matter so it can remain as input
  DDRB = B00110001;

  //analog pins - A1, A4 - output, A0, A3, A5 - input
  DDRC = B00010010;

  // set initial wheels speed;
  leftWheelsSpeed = 120;
  rightWheelsSpeed = 120;

  /***** flamePin i.e., pin 2 is the interrupt pin INT0
    - to do reg. manipulation for interrupts, EICRA and EIMSK registers are used
    - bit at position 0 and 1 of the EICRA register controls INT0 and when both are set to 1, means RISING EDGE
    - bit 0 of EIMSK register controls INT0, and setting to 1 tells the Arduino to treat the pin as interrupt *****/
  EICRA |= ((1 << 1) | (1 << 0));
  EIMSK |= (1 << 0);

  //initialises the interrupt
  sei();

  /** reg. version of Serial begin - bit 3 of this register controls the transmitter pin (TXEN0); Baud rate can be increased **/
  UCSR0B = 1 << 3; 
  UBRR0L = 2;
}

int main() {
  init();
  setup();
  while(1) {
    /** if only needed at runtime for calc, declaring it at runtime is better since it also saves memory **/
    float frontDistanceInCM = checkDistance(trigPin, echoPin);
    float sideDistanceInCM = checkDistance(rightTrigPin, rightEchoPin);
      
    noTone(buzzer);
    startRobot();
    checkForGasLeak();
    
    /**** - using the F string wrapper around Serial print helps to save SRAM by using flash memory instead for the strings ****/
    if(frontDistanceInCM < 30) {
      stopRobot();
      delay(100);
      moveBackwards();
      delay(200);
      if(sideDistanceInCM < 40) {
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
}

//register manipulation wasn't working for ultrasonic sensors
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

//inbuilt ISR routine for pin 2 (INT0)
ISR(INT0_vect) {
  stopRobot();
  tone(buzzer, buzzerFreq); 
  sendSMSNotification("Fire detected!", "fire");
}

/** simulate sending SMS via Wi-Fi module + online service
    using a char array instead of String data type also optimises memory **/
void sendSMSNotification(char *message, char *type) {
  unsigned long currTime = millis()/1000;
  
  //cannot use flash memory for Serial.println here since the message is gotten from parameter, in a variable
  if(type == "gas") {
    //10 seconds interval
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
    PORTD &= ~(1 << driverPin1a);

    /*** driverPin1b is a PORT B pin and the modulo part ensures the value is written to the correct bit corresponding to the pin in the Port B data output register ***/
    PORTB &= ~(1 << (driverPin1b % 8));
    PORTB &= ~(1 << (driverPin2a % 8));
    PORTB &= ~(1 << (driverPin2b % 8));
}

void stopWheels() {
  analogWrite(enableLeftWheels, 0);
  analogWrite(enableRightWheels, 0);
}


void moveForward() {
    PORTD &= ~(1 << driverPin1a);
    PORTB |= (1 << (driverPin1b % 8));
    PORTB &= ~(1 << (driverPin2a % 8));
    PORTB |= (1 << (driverPin2b % 8));    
}

void moveBackwards() {
    PORTD |= (1 << driverPin1a);
    PORTB &= ~(1 << (driverPin1b % 8));

    PORTB |= (1 << (driverPin2a % 8));
    PORTB &= ~(1 << (driverPin2b % 8)); 
}

void turnRight() {
    analogWrite(enableRightWheels, 0);
    analogWrite(enableLeftWheels, 250);
    
    PORTB &= ~(1 << (driverPin2a % 8)); 
    PORTB |= (1 << (driverPin2b % 8)); 
}

void turnRightWithFullSpeed() {
 PORTD |= (1 << enableLeftWheels);
 PORTD |= (1 << enableRightWheels);

  PORTD &= ~(1 << driverPin1a); 
  PORTB &= ~(1 << (driverPin1b % 8));
  PORTB &= ~(1 << (driverPin2a % 8));
  PORTB |= (1 << (driverPin2b % 8)); 
}

//stop left wheels, set forward direction for right wheels
void turnLeft() {
    analogWrite(enableRightWheels, 250);
    analogWrite(enableLeftWheels, 0); 

    PORTD &= ~(1 << driverPin1a); 
    PORTB |= (1 << driverPin1b % 8);
}

void turnLeftWithFullSpeed(){
  PORTD |= (1 << enableLeftWheels);
  PORTD |= (1 << enableRightWheels);
  
  PORTD &= ~(1 << driverPin1a); 
  PORTB |= (1 << (driverPin1b % 8)); 
  PORTB &= ~(1 << (driverPin2a % 8)); 
  PORTB &= ~(1 << (driverPin2b % 8)); 
}

