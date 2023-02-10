/*
 * Demonstration file
 */
#include <Servo.h>
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit Switch variables
#define REAR_LIMIT_SWITCH 5 // initialize rear limit switch to pin 5
#define LEFT_LIMIT_SWITCH 6 // initialize front left limit switch to pin 6
#define RIGHT_LIMIT_SWITCH 7 // initialize front right limit switch to pin 7
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic variables
#define PING_PIN 22 // Trigger Pin of Ultrasonic Sensor to pin 22
#define ECHO_PIN 23 // Echo Pin of Ultrasonic Sensor to pin 23
IntervalTimer MY_TIMER_1; // uses pin 10
IntervalTimer MY_TIMER_2; // uses pin 11
/*----------------------------------------------------------------------------------------------------------------------*/
// RC controller variables
#define THROTTLE_PIN 4 // initialize throttle pin
volatile long TH_START_TIME = 0; // initialize starting timer
volatile long TH_CURRENT_TIME = 0; // initialize current time
volatile long THROTTLE_PULSE = 0; // inititalize throttle pulse width
volatile long THROTTLE_PW = 0; // initialize throttle pulse width updater
volatile long THROTTLE_VALUE = 0; // initialize throttle value

#define STEERING_PIN 3 // initialize steering pin
volatile long ST_START_TIME = 0; // initialize starting timer
volatile long ST_CURRENT_TIME = 0; // initialize current time
volatile long STEERING_PULSE = 0; // initialize steering pulse width
volatile long STEERING_PW = 0; // initialize steering pulse width updater
volatile long STEERING_VALUE = 0; // initialize steering servo value

#define DEAD_MAN_PIN 4 // initialize RC dead man switch pin
volatile long DEAD_MAN_VALUE; // initialize value read from RC dead man buttons
/*
 * VOLATILE LONG --> let program know that this variable will change and to not try to skip over it
 */
/*----------------------------------------------------------------------------------------------------------------------*/
// Drive motor and steering servo variables
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
/*----------------------------------------------------------------------------------------------------------------------*/

void DriveForward(){
  if(VELOCITY <= 110){ // while rover is not yet at the desired speed
    delay(250);
    VELOCITY++; // gradually increase speed until target speed is reached
  }
  ESC_MOTOR.write(VELOCITY); // pass VELOCITY reading to the motor
}

// NOTE: UNSURE HOW TO SET RC BUTTONS TO MOMENTARY, CURRENTLY SET AS TOGGLE BUTTONS AND THEREFORE NOT TRULY A DEAD MAN'S SWITCH
void DeadManSwitch() { // Auton|RC Control toggle function
  DEAD_MAN_VALUE = pulseIn(DEAD_MAN_PIN, HIGH); // read value from RC buttons between ~2000 & ~1000
  if(DEAD_MAN_VALUE < 2100 && DEAD_MAN_VALUE > 1900) { // if top button is pressed use RC control
    RCDrive(); // return PWM values for RC throttle and steering  
    Serial.println("RC Control");
  }
  else{ // if bottom button is pressed, use autonomous routine
    DriveForward(); // drive forward until interrupted (250ms blocking delay)
    LimitSwitchCollision(); // if limit switch is triggered, interrupt drive forward and do collision response
    UltrasonicCollision(); // if sonar is triggered, interrupt drive forward and do collision response
    Serial.println("Autonomous");
  }
}

void setup() {
  Serial.begin(9600);
  
  ESC_MOTOR.attach(8); // set ESC_MOTOR to pin 8
  TURN_SERVO.attach(9); // set TURN_SERVO to pin 9
  
  LimitSwitchSetup(); // setup limit switches

  UltrasonicSetup(); // setup ultrasonics
  
  LEDSetup(); // turn on on board LED to ensure program is running

  RCSetup(); // setup RC controller
  // NOTE: IF MAPPING IS MESSED UP FOR RC CONTROLLER VALUES... 
  // TRY TUNING THE TRIM BUTTONS ON CONTROLLER FIRST BEFORE TRYING TO MAKE ADJUSTMENTS TO THE CODE 

}

void loop() {

  DeadManSwitch(); // allow for toggling between auton and RC control

}
