#include <Servo.h>
#define MOTOR_PIN 8
#define TRANSMITTER_PIN 2

unsigned long MOTOR_VALUE;
Servo ESC_MOTOR;  


// This code is to test the ESC on the rover.

void LEDSetup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on light to ensure board is working
}

void setup() 
{
  Serial.begin(9600);
  ESC_MOTOR.attach(MOTOR_PIN);  // attaches the servo on pin 10 to the servo object
  pinMode(MOTOR_PIN, INPUT); 
  pinMode(TRANSMITTER_PIN, INPUT);

  LEDSetup();
}

void loop() 
{
  MOTOR_VALUE = pulseIn(TRANSMITTER_PIN, HIGH); //this will have to be changed it its own pin sometime since this occupies either the esc or servo
  delay(15);
  //Serial.println(MOTOR_VALUE);
  if (MOTOR_VALUE > 1020 && MOTOR_VALUE < 1860)
  {
    MOTOR_VALUE = map(MOTOR_VALUE, 1020, 1860, 0, 180);
    Serial.println(MOTOR_VALUE);
    ESC_MOTOR.write(MOTOR_VALUE);
  }
}
