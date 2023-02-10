#include <Servo.h>

#define THROTTLE_PIN 3 // initialize throttle pin
volatile long TH_START_TIME = 0; // initialize starting timer
volatile long TH_CURRENT_TIME = 0; // initialize current time
volatile long THROTTLE_PULSE = 0; // inititalize throttle 
int THROTTLE_PW = 0; // initialize throttle 

#define STEERING_PIN 2 // initialize steering pin
volatile long ST_START_TIME = 0; // initialize starting timer
volatile long ST_CURRENT_TIME = 0; // initialize current time
volatile long STEERING_PULSE = 0; // initialize steering
int STEERING_PW = 0; // initialize steering

int VELOCITY = 90; // declare VELOCITY as global to keep track of speed between function calls
Servo ESC_MOTOR; // initialize ESC_MOTOR as a Servo object
Servo TURN_SERVO; // initialize TURN_SERVO as a Servo object

void RCReadSetup(){
  pinMode(THROTTLE_PIN, INPUT_PULLUP); // set the pin to accept inputs
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN),ThrottleTimer,CHANGE); // attach interrupt to THROTTLE_PIN
  pinMode(STEERING_PIN, INPUT_PULLUP); // set the pin to accept inputs
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN),SteeringTimer,CHANGE); // attach interrupt to STEERING_PIN
}

void RCRead() {
  // return PWM values for RC throttle and steering
  if(THROTTLE_PULSE < 2200){
    THROTTLE_PW = THROTTLE_PULSE;
  }
  Serial.print("Throttle: ");
  Serial.println(THROTTLE_PW); // print throttle value
  delay(250);

  if(STEERING_PULSE < 2200){
    STEERING_PW = STEERING_PULSE;
  }
  Serial.print("Steering: ");
  Serial.println(STEERING_PW); // print steering value
  delay(250);
}

void ThrottleTimer(){
  // checking THROTTLE_PIN interrupt
  TH_CURRENT_TIME = micros();
  if (TH_CURRENT_TIME > TH_START_TIME){
    THROTTLE_PULSE = TH_CURRENT_TIME - TH_START_TIME;
    TH_START_TIME = TH_CURRENT_TIME;
  }
}

void SteeringTimer(){
  // checking STEERING_PIN interrupt
  ST_CURRENT_TIME = micros();
  if (ST_CURRENT_TIME > ST_START_TIME){
    STEERING_PULSE = ST_CURRENT_TIME - ST_START_TIME;
    ST_START_TIME = ST_CURRENT_TIME;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  RCReadSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  RCRead();
  
  ESC_MOTOR.attach(9); // set ESC_MOTOR to pin 9
  TURN_SERVO.attach(10); // set TURN_SERVO to pin 10 --> NEED TO CHANGE PIN ASSIGNMENT

}
