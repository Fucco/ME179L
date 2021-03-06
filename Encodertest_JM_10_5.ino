/* Encoder Test
 *  
*** Description ***
Demonstrates how to count turns of an encoder (Lego pulley) wheel. In addition to the wheel you will need:
--A DC Motor.
--A micro switch.
--A break-beam sensor.
--An LCD screen (optional).
The wheel should be attached to a rotating axle, directly or indirectly attached to the motor, and placed
near the break-beam sensor so that as it turns it alternately blocks and allows the beam through.
*** History ***
--10/4/12:  Modified by Blane for readability and simpler logic.
*/

#include <AFMotor.h>
#include <SoftwareSerial.h>

// Define constants:
#define switchPin 3
#define changeSpeedPin 14
#define changeDistancePin 15   
#define txPin 13   // LCD tx pin.
#define rxPin 13   // LCD rx pin (not really used).
#define encoderPin 2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt).
#define motorTerminalLeft 4   // (1-4 only).
#define motorTerminalRight 3   // (1-4 only).

// Define (and initialize) global variables:
volatile int encoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
int encoderCountGoal[]= {100,200,300};
int timeStart=0;
int timeStop=0;
int timeTaken=0;
int motorSpeed=1;
int speedSetting[] = {130,190,250};
int distanceSetting=1;

// Define serial display and motor objects:
SoftwareSerial mySerial =  SoftwareSerial( rxPin, txPin);  
AF_DCMotor leftMotor( motorTerminalLeft, MOTOR34_1KHZ); 
AF_DCMotor rightMotor( motorTerminalRight, MOTOR34_1KHZ);


void setup() {
  
  // Setup hardware interrupt:
  int interruptPin = encoderPin - 2;   // Hardware interrupt pin (0 or 1 only, to refer to digital pin 2 or 3, respectively).
  attachInterrupt( interruptPin, IncrementAndDisplay, FALLING);   // Attach interrupt pin, name of function to be called 
        // during interrupt, and whether to run interrupt upon voltage FALLING from high to low or ...
  
  // Setup encoder i.e. break-beam:
  pinMode( encoderPin, INPUT); 
  digitalWrite( encoderPin, HIGH);  
  
  // Setup switch:
  pinMode( switchPin, INPUT); 
  digitalWrite( switchPin, HIGH);  

  pinMode(changeSpeedPin, INPUT);      // Make switch pin an input
  digitalWrite(changeSpeedPin, HIGH);  // Enable the pull-up resistor on left switch pin

  pinMode(changeDistancePin, INPUT);      // Make switch pin an input
  digitalWrite(changeDistancePin, HIGH);  // Enable the pull-up resistor on left switch pin
  
  
  // Setup serial display:
  pinMode( txPin, OUTPUT);
  mySerial.begin(9600); 
        
  // Set motor speed:
  rightMotor.setSpeed(speedSetting[motorSpeed]);
  leftMotor.setSpeed(speedSetting[motorSpeed]);
  
  mySerial.print("?f");          //Clears LCD screen
  mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
  mySerial.print("Press to Begin...");    //Displays "Press to Begin..."
}


void loop() {

  while (digitalRead(switchPin)) {      // Wait until switch is pressed.
    
    if (!digitalRead(changeSpeedPin)) {
      increaseSpeed();
      delay(500);
    }
    
    if (!digitalRead(changeDistancePin)) {
      changeDistance();
      delay(500);
    }
  }         
  
  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("Counts:");
  
  encoderCount = 0; 
  driveForward();
  
  timeStart=millis();   //Start timing
  
  while (encoderCount < encoderCountGoal[distanceSetting]) {
      mySerial.print("?x00?y1");
      mySerial.print(encoderCount);
    }   // Wait until encoder count goal is reached 
        // (allowing interrupt to update value of encoderCount during this time):

 
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE); 

  timeStop=millis();    //Stop timing
  
  delay(150);
  timeTaken = timeStop - timeStart;   //Calcutale how much time passed during the run

 
  //Print the time and the encoderc count
  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("Time: ");
  mySerial.print(timeTaken);
  mySerial.print("?x00?y1");
  mySerial.print("Count: ");
  mySerial.print(encoderCount);
}


void IncrementAndDisplay() {
  ++encoderCount;
}

void increaseSpeed() {

  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("Speed set to:");

  motorSpeed++;
  motorSpeed= motorSpeed % 3;


  //Set the speed to the next setting
  switch (motorSpeed) {
    case 0: 
    leftMotor.setSpeed(speedSetting[motorSpeed]);
    rightMotor.setSpeed(speedSetting[motorSpeed]);
    mySerial.print("?x00?y1");
    mySerial.print("Low");
    break;
    
    case 1: 
    leftMotor.setSpeed(speedSetting[motorSpeed]);
    rightMotor.setSpeed(speedSetting[motorSpeed]);
    mySerial.print("?x00?y1");
    mySerial.print("Medium");
    break;
    
    case 2: 
    leftMotor.setSpeed(speedSetting[motorSpeed]);
    rightMotor.setSpeed(speedSetting[motorSpeed]);
    mySerial.print("?x00?y1");
    mySerial.print("High");
    break;
  }

  
}

void changeDistance() {

  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("Distance set to:");

  distanceSetting++;
  distanceSetting= distanceSetting % 3;

  //Change the distance to the next setting
  switch (distanceSetting) {
    case 0: 
    mySerial.print("?x00?y1");
    mySerial.print("Short");
    break;
    
    case 1: 
    mySerial.print("?x00?y1");
    mySerial.print("Medium");
    break;
    
    case 2: 
    mySerial.print("?x00?y1");
    mySerial.print("Long");
    break;
  }

  
}


void driveForward() {

  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
}




