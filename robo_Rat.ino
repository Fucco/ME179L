#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <Servo.h> 


#define txPin 13   // LCD tx pin.
#define rxPin 12// LCD rx pin (not really used).
#define motor_Left 1   // Left Motor
#define motor_Right 2   // Right Motor
#define motor_Sweep 4   // Sweep Motor
#define encoderPin 2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt).
#define ReflectPinLeft A0 //Reflective Sensor
#define ReflectPinRight A1 //Reflective Sensor
#define LightSensor A2 //Light Sensor




volatile int EncoderCount = 0; // Use "volatile" for faster updating of value during hardware interrupts.


int ReflectValueRight, ReflectValueLeft,RangeSensorValue,LightSensorValue; //Stores Reflect Sensor output
int DefaultSpeed = 160;
int EncoderGoal=300;
int SweepSpeed = 255;
int start=0;
double Setpoint, Input, Output;
int Kp=0.8, Ki=0.1, Kd=0;
int ServoPosBack=0,ServoPosForward=200;


SoftwareSerial mySerial =  SoftwareSerial( rxPin, txPin); 
AF_DCMotor MotorLeft( motor_Left, MOTOR34_1KHZ); 
AF_DCMotor MotorRight( motor_Right, MOTOR34_1KHZ);
AF_DCMotor MotorSweep( motor_Sweep, MOTOR34_1KHZ);


PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);

Servo servo1;  // create servo object to control a servo

void setup() {
  // put your setup code here, to run once:

    int interruptPin = encoderPin - 2;
    attachInterrupt(interruptPin, IncrementAndDisplay, FALLING);

    pinMode(encoderPin, INPUT); 
    
    digitalWrite(encoderPin, HIGH);
    digitalWrite(LightSensor, HIGH);

    digitalWrite(ReflectPinLeft, HIGH);
    digitalWrite(ReflectPinRight, HIGH);

    MotorLeft.setSpeed(DefaultSpeed);
    MotorRight.setSpeed(DefaultSpeed);
    MotorSweep.setSpeed(SweepSpeed);

    Serial.begin(9600); 
    mySerial.begin(9600);

    //initialize the variables we're linked to
  Input = 0;
  Setpoint = 100;
  myPID.SetOutputLimits(-255, 255);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);


  
  
  servo1.attach(9);  // attaches the servo on pin 9 to the servo object
  

}

void loop() {
  // put your main code here, to run repeatedly:
  servo1.write(5); 
  delay(1000);
  ReflectValueRight = analogRead(ReflectPinRight);
  ReflectValueLeft = analogRead(ReflectPinLeft);
  RangeSensorValue = analogRead(ReflectPinLeft);
  
  while (start<1) { //Wait for light 
    LightSensorValue = analogRead(LightSensor); //Check light Sensor
    
    mySerial.print("?f");     //Print some things to the LCD
    mySerial.print("?x00?y0");  
    mySerial.print("Light = ");
    mySerial.print(LightSensorValue);
    
    Serial.print("Value= ");    //Print some things to montor
    Serial.println(LightSensorValue);
    
    if (LightSensorValue<800) {   //Check if light is on
      StartMotors(); //Start and drive forward to the line
      delay(2000);
      start++;                    //Prevents arduino to enter the startup loop again
      break;                      //Breaks the while loop if light is on
    }
    delay(300);                   //Some delay for easier readings from LCD
  } //End of Light check 
  
  
  while (1) { //(RangeSensorValue<800)

    //Reads values from sensors
    ReflectValueRight = analogRead(ReflectPinRight);
    ReflectValueLeft = analogRead(ReflectPinLeft);
    RangeSensorValue = analogRead(ReflectPinLeft);

    mySerial.print("?f");
    mySerial.print("?x00?y0");  
    mySerial.print("Encoder = ");
    mySerial.println(EncoderCount);
    delay(10);
    FollowLine();

    if (EncoderCount>EncoderGoal && start==1) {
      Stop();
      servo1.write(180); 
      servo1.write(150);
      start++;
    }

  }





  

} //End of loop()


void Forward(){
  MotorLeft.run(FORWARD);
  MotorRight.run(FORWARD);
}

void Stop(){
  MotorLeft.run(RELEASE);
  MotorRight.run(RELEASE);
}

void BackWard(){
  MotorLeft.run(BACKWARD);
  MotorRight.run(BACKWARD);
}

void TurnLeft(){
  MotorLeft.run(RELEASE);
  MotorRight.run(FORWARD);
}

void TurnRight(){
  MotorLeft.run(FORWARD);
  MotorRight.run(RELEASE);
}

void StopMotors(){
  MotorLeft.run(RELEASE);
  MotorRight.run(RELEASE);
  MotorSweep.run(RELEASE);
}

void StartMotors(){
  MotorLeft.run(FORWARD);
  MotorRight.run(FORWARD);
  MotorSweep.run(BACKWARD);
}

void IncrementAndDisplay(){
  EncoderCount++;
}

 
void FollowLine() {


   if (ReflectValueRight>200){
      MotorLeft.setSpeed(DefaultSpeed+35);
      MotorRight.setSpeed(DefaultSpeed-160);
  }
  else if (ReflectValueLeft>200){
      MotorLeft.setSpeed(DefaultSpeed-160);
      MotorRight.setSpeed(DefaultSpeed+35);
  }
  else {
      MotorLeft.setSpeed(DefaultSpeed);
      MotorRight.setSpeed(DefaultSpeed);
  }

  
}

void FollowLinePID() {

   Input=ReflectPinLeft-ReflectPinRight;
   myPID.Compute();

   MotorLeft.setSpeed(DefaultSpeed+Output);
   MotorRight.setSpeed(DefaultSpeed+Output);


}





