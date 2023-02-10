/*
 * Dead Man's Switch test file
 */

#include <Servo.h>

#define LEFT_LIMIT_SWITCH 6 // initialize limit switch & pin number
#define RIGHT_LIMIT_SWITCH 7 // initialize limit switch & pin number

#define THROTTLE_PIN 2 // initialize RC throttle pin
volatile long TH_START_TIME = 0; // initialize starting timer using volatile to let program know the variable can change
volatile long TH_CURRENT_TIME = 0; // initialize current time
volatile long THROTTLE_PULSE = 0; // inititalize throttle pulse width
volatile long THROTTLE_PW = 0; // initialize throttle pulse width updater
volatile long THROTTLE_VALUE = 0; // initialize throttle value

#define STEERING_PIN 3 // initialize RC steering pin
volatile long ST_START_TIME = 0; // initialize starting timer
volatile long ST_CURRENT_TIME = 0; // initialize current time
volatile long STEERING_PULSE = 0; // initialize steering pulse width
volatile long STEERING_PW = 0; // initialize steering pulse width updater
volatile long STEERING_VALUE = 0; // initialize steering servo value

#define DEAD_MAN_PIN 4 // initialize RC dead man switch pin
volatile long DEAD_MAN_VALUE; // initialize value read from RC dead man buttons

int VELOCITY = 90; // declare VELOCITY as global to keep track of speed between function calls
Servo ESC_MOTOR; // initialize ESC_MOTOR as a Servo object
Servo TURN_SERVO; // initialize TURN_SERVO as a Servo object

/* MOTOR SPEEDS AND DIRECTIONS BASED OFF PULSE WIDTH MODULATION VALUES
* 0 <= speed <= 90 == reverse direction for motors
* 90 <= speed <= 180 == forward direction for motors
* 90 == stop motors
* 0 == full speed in reverse
* 180 == full speed forwards
*/

void DriveForward(){
  if(VELOCITY <= 110){ // while rover is not yet at the desired speed
    delay(250);
    VELOCITY++; // gradually increase speed until target speed is reached
  }
  ESC_MOTOR.write(VELOCITY); // pass VELOCITY reading to the motor
}


void DeadManSwitch() { // Auton|RC Control toggle function
  DEAD_MAN_VALUE = pulseIn(DEAD_MAN_PIN, HIGH); // read value from RC buttons between ~2000 & ~1000
  if(DEAD_MAN_VALUE < 2100 && DEAD_MAN_VALUE > 1900) { // if top button is pressed use RC control
    RCDrive(); // return PWM values for RC throttle and steering  
    Serial.println("RC Control");
  }
  else{ // if bottom button is pressed, use autonomous routine
    DriveForward(); // drive forward until interrupted (250ms blocking delay)
    LimitSwitchCheck(); // if limit switch is triggered, interrupt drive forward and avoid obstacle
    Serial.println("Autonomous");
  }
}

void setup() {
  Serial.begin(9600);
  
  ESC_MOTOR.attach(8); // set ESC_MOTOR to pin 8
  TURN_SERVO.attach(9); // set TURN_SERVO to pin 9
  
  LimitSwitchSetup(); // setup limit switches
  
  LEDSetup(); // setup LED light

  RCSetup(); // setup RC reading pins

}

void loop() {
  DeadManSwitch(); // use dead man switch toggle
}
