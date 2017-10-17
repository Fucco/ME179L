

#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

// Define constants:   
#define txPin 13   // LCD tx pin.
#define rxPin 13   // LCD rx pin (not really used).
#define encoderPin 2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt).
#define motorTerminalLeft 3   // (1-4 only).
#define motorTerminalRight 4   // (1-4 only).

// Define (and initialize) global variables:
volatile int encoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
const int analogInPin = A2;
const int analogInPinFront = A1;
const int analogInPinLight = A0;
int encoderCountGoal= 300;
double PID=0, start=0;
double Setpoint=400;
double frontDistance=200;
double Kp=1.2, Ki=0, Kd=0;
double Input,FrontInput,LightInput=1000;

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
  
  // Setup serial display:
  pinMode( txPin, OUTPUT);
  mySerial.begin(9600); 

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
        
  // Set motor speed:
  rightMotor.setSpeed(255);
  leftMotor.setSpeed(255);

  mySerial.print("?f");          //Clears LCD screen
  mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
  mySerial.print("Press to Begin...");    //Displays "Press to Begin..."

  
}


void loop() {
  
  Input = analogRead(analogInPin);
  FrontInput=analogRead(analogInPinFront);
  while (LightInput>650 && start<1) {      // Wait until switch is pressed.
    LightInput=analogRead(analogInPinLight);
    Serial.print("Light = " );                       
    Serial.println(LightInput);

    mySerial.print("?f");          //Clears LCD screen
    mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
    mySerial.print(LightInput);
    delay(300);
  }         
  start++;
  driveForward();
  
  Serial.print("sensor = " );                       
  Serial.println(Input);
  
  PID=Kp*(Setpoint-Input);
  Serial.print("PID = " );                       
  Serial.println(PID);
  
  Serial.print("Front = " );                       
  Serial.println(FrontInput);


if (PID<0){
  rightMotor.setSpeed(255+PID);
}
else {
  leftMotor.setSpeed(255-PID);
}

if (FrontInput>frontDistance){
  turnRight();
}

  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(10);                     

  
}

void IncrementAndDisplay() {
  ++encoderCount;
}

void driveForward() {

  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
}

void turnRight() {

  leftMotor.run(FORWARD);
  rightMotor.run(BACKWARD);
  delay(500);
}




