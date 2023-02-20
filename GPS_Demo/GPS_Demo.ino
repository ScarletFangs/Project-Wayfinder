/*
 * GPS Navigation Test File
 */
#include <LSM303.h>
#include <Wire.h>
#include <Servo.h>
#include <millisDelay.h>

LSM303 COMPASS; // initialize an LSM303 compass object

volatile double CURRENT_LAT = 0;
volatile double CURRENT_LONG = 0;
volatile float CURRENT_HEADING = 0;
volatile double LAT_DIFF = 0; // initialize latitude difference variable
volatile double LONG_DIFF = 0; // initialize longitude difference variable
volatile double TARGET_LAT = 34.114405; // initialize target latitude
volatile double TARGET_LONG = -117.795699; // initialize target longitude
volatile double A = 0; // initialize argument of the square root
volatile double C = 0; // initialize arctangent term
volatile double DISTANCE = 21; // initialize final distance variable
volatile double TARGET_HEADING = 0; // initialize target heading
volatile float ANGLE_PROVISIONAL = 0; // initialize angle provisional
volatile float ANGLE_TURN = 0; // initialize angle provisional
/*
 * VOLATILE DOUBLE --> changing number of up to 15 decimal precision
 * VOLATILE FLOAT --> changing number of up to 7 decimal precision
 */
/*----------------------------------------------------------------------------------------------------------------------*/
// RC controller variables
#define THROTTLE_PIN 2 // initialize throttle pin
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

volatile int VELOCITY = 90; // declare VELOCITY as global to keep track of speed between function calls
Servo ESC_MOTOR; // initialize ESC_MOTOR as a Servo object --> on pin 8
Servo TURN_SERVO; // initialize TURN_SERVO as a Servo object --> on pin 9

millisDelay PROGRAM_STARTUP;
millisDelay VELOCITY_TIMER;
volatile long REMAINING_VELOCITY_TIME = 0;
volatile int CHECK = 0;
/*
 * VOLATILE FLOAT --> changing number with a lot of decimal points
 * CONST DOUBLE --> unchanging number of double variable type
 */
 
void setup() {
  ESC_MOTOR.attach(9); // set ESC_MOTOR to pin 8
  TURN_SERVO.attach(8); // set TURN_SERVO to pin 9

  LEDSetup(); // setup LED

  CompassSetup(); // Setup compass

  GPSSetup(); // setup GPS serial communication
}

void loop() {
  //Serial.println(DEAD_MAN_VALUE);
  //DeadManSwitch();
  //TurnToHeading();
  //TURN_SERVO.write(90);
  //ESC_MOTOR.write(90);
  TURN_SERVO.write(180);
  ESC_MOTOR.write(105);
}
