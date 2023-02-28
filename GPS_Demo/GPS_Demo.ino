/*
 * GPS Navigation Test File
 */
#include <LSM303.h> // include compass library
#include <Wire.h> // include I2C library
#include <Servo.h> // include servo/motor library
#include <millisDelay.h> // include delay timer library (from SafeString library)
#include <SoftwareSerial.h> // include software serial library
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit Switch variables
#define REAR_LIMIT_SWITCH 5 // initialize rear limit switch to pin 5
#define LEFT_LIMIT_SWITCH 6 // initialize front left limit switch to pin 6
#define RIGHT_LIMIT_SWITCH 7 // initialize front right limit switch to pin 7
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic variables
#define TRIG_PIN 22 // Trigger Pin of Ultrasonic Sensor to pin 22
#define ECHO_PIN 23 // Echo Pin of Ultrasonic Sensor to pin 23
IntervalTimer MY_TIMER_1; // timer that uses pin 10
IntervalTimer MY_TIMER_2; // timer that uses pin 11
/*----------------------------------------------------------------------------------------------------------------------*/
// GPS and Compass variables
LSM303 COMPASS; // initialize an LSM303 compass object

volatile double CURRENT_LAT = 0; // initialize current latitude
volatile double CURRENT_LONG = 0; // initialize current longitude
volatile double CURRENT_HEADING = 0;
volatile double LAT_DIFF = 0; // initialize latitude difference variable
volatile double LONG_DIFF = 0; // initialize longitude difference variable
volatile double TARGET_LAT = 34.0277013; // initialize target latitude
volatile double TARGET_LONG = -117.504342; // initialize target longitude
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
int THROTTLE_PW = 0; // initialize throttle pulse width updater
int THROTTLE_VALUE = 0; // initialize throttle value

#define STEERING_PIN 3 // initialize steering pin
volatile long ST_START_TIME = 0; // initialize starting timer
volatile long ST_CURRENT_TIME = 0; // initialize current time
volatile long STEERING_PULSE = 0; // initialize steering pulse width
int STEERING_PW = 0; // initialize steering pulse width updater
int STEERING_VALUE = 0; // initialize steering servo value

#define DEAD_MAN_PIN 4 // initialize RC dead man switch pin
volatile long DEAD_MAN_VALUE; // initialize value read from RC dead man buttons

/*
 * VOLATILE LONG --> let program know that this variable will change and to not try to skip over it
 */
/*----------------------------------------------------------------------------------------------------------------------*/
// Motor variables
volatile int VELOCITY = 90; // declare VELOCITY as global to keep track of speed between function calls
Servo ESC_MOTOR; // initialize ESC_MOTOR as a Servo object --> on pin 9
Servo TURN_SERVO; // initialize TURN_SERVO as a Servo object --> on pin 8

millisDelay PROGRAM_STARTUP;
millisDelay VELOCITY_TIMER;
volatile long REMAINING_VELOCITY_TIME = 0;
volatile int CHECK = 0;
/*
 * VOLATILE FLOAT --> changing number with a lot of decimal points
 * CONST DOUBLE --> unchanging number of double variable type
 */
/*----------------------------------------------------------------------------------------------------------------------*/
// Bluetooth variables
//#define BLUETOOTH_TX 34 // initialize bluetooth TX pin on 34
//#define BLUETOOTH_RX 35 // initialize bluetooth RX pin on 35
//SoftwareSerial BTserial(BLUETOOTH_RX, BLUETOOTH_TX); // create software serial port on pins 34 & 35
/*----------------------------------------------------------------------------------------------------------------------*/
void setup() {

  Serial.begin(9600);
  
  ESC_MOTOR.attach(9); // Set ESC_MOTOR to pin 9
  TURN_SERVO.attach(8); // Set TURN_SERVO to pin 8

  ESC_MOTOR.write(90); // Reset drive motors
  TURN_SERVO.write(90); // reset turn servo

  LEDSetup(); // Setup LED

  CompassSetup(); // Setup compass

  GPSSetup(); // Setup GPS serial communication

  RCSetup(); // Setup RC control

  BluetoothSetup(); // Setup bluetooth telemetry
}

void loop() {
  
  //DeadManSwitch(); // toggle between autonomous control and RC control

  GPSNavigation(30); // GPSNavigation(int error boundary to decide when to switch between HeadingHold() and TurnToHeading())
}
