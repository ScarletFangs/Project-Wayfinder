/*
 * GPS Navigation test file designed to navigate the rover to a waypoint using the GPS module & compass as the sole 
 * means of navigation. Collision response functions have been included for use with RCDrive() only. ObstaceleAvoidance() 
 * is not currently used. Also includes telemetry through the means of a bluetooth module. 
 * Gabriel Gardner - 03/17/2023
 */
#include <LSM303.h> // Include compass library 
#include <Wire.h> // Include I2C library for communication with compass
#include <Servo.h> // Include servo/motor library for driving esc and turn servo
#include <millisDelay.h> // Include delay timer library (from SafeString library) for SensorTimers()
#include <NewPing.h> // Include NewPing library for ultrasonics
/*----------------------------------------------------------------------------------------------------------------------*/
// VARIABLE TYPES AND THEIR MEANINGS

/*
 * VOLATILE --> let program know that this variable will change and to not try to skip over it
 * STATIC --> maintain variable value between function calls
 * CONST --> read-only variable that cannot be changed after intialization
 * DOUBLE --> floating-point type of up to 15 decimal precision
 * FLOAT --> foating-point type of up to 7 decimal precision
 * LONG --> integer type of up to 32 bits
 * SHORT --> integer type of up to 16 bits
 */

/*----------------------------------------------------------------------------------------------------------------------*/
// Debugging Variables (#define = turn on prints, #undef = turn off prints)
#undef SERIAL_DEBUG // Initialize serial monitor debugging condition
#define BLUETOOTH_DEBUG // Initialize Bluetooth debugging condition
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit Switch variables
#define REAR_LIMIT_SWITCH 23 // Initialize rear limit switch
#define LEFT_LIMIT_SWITCH 21 // Initialize front left limit switch
#define RIGHT_LIMIT_SWITCH 22 // Initialize front right limit switch
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic variables
#define LEFT_TRIG 16    // Trigger Pin of Left Ultrasonic Sensor to pin 16
#define LEFT_ECHO 17    // Echo Pin of Left Ultrasonic Sensor to pin 17
#define CENTER_TRIG 13  // Trigger Pin of Center Ultrasonic Sensor to pin 13 --> CURRENTLY CONFLICTING WITH PROGRAM_LED
#define CENTER_ECHO 20  // Echo Pin of Center Ultrasonic Sensor to pin 20
#define RIGHT_TRIG 41   // Trigger Pin of Right Ultrasonic Sensor to pin 41
#define RIGHT_ECHO 40   // Echo Pin of Right Ultrasonic Sensor to pin 40
#define REAR_TRIG 39    // Trigger Pin of Rear Ultrasonic Sensor to pin 39
#define REAR_ECHO 38    // Echo Pin of Rear Ultrasonic Sensor to pin 38

volatile long LEFT_SONAR_DISTANCE = 0;
volatile long CENTER_SONAR_DISTANCE = 0;
volatile long RIGHT_SONAR_DISTANCE = 0;

#define SONAR_NUM 3 // Number of ultrasonic sensors
#define MAX_DISTANCE 500 // Maximum distance (in cm) to ping
#define PING_INTERVAL 60 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo)

unsigned long PING_TIMER_ARRAY[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor
unsigned int DISTANCE_ARRAY[SONAR_NUM];    // Where the ping distances are stored
uint8_t CURRENT_SENSOR = 0;         // Keeps track of which sensor is active
unsigned int GLOBAL_MIN_SENSOR = 0; // Sensor with the smallest distance 0 = Right Sensor; 1 = Middle Sensor; 2 = Left Sensor 
unsigned int OBS_DISTANCE = 50; // Minimum distance for ultra sonics to a avoid obstacle

NewPing SONAR_TIME[SONAR_NUM] = // Sensor object array using NewPing library 
   {NewPing(LEFT_TRIG, LEFT_ECHO, MAX_DISTANCE),      // Left ultrasonic setup
    NewPing(CENTER_TRIG, CENTER_ECHO, MAX_DISTANCE),  // Right ultrasonic setup
    NewPing(RIGHT_TRIG, RIGHT_ECHO, MAX_DISTANCE)};   // Right ultrasonic setup
/*----------------------------------------------------------------------------------------------------------------------*/
// Compass variables
LSM303 COMPASS; // Declare an LSM303 compass object
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
short X_MIN, Y_MIN, Z_MIN, X_MAX, Y_MAX, Z_MAX; // Declare compass calibration values
char report[80]; // Declare report size for printing calibration values
/*----------------------------------------------------------------------------------------------------------------------*/
// GPS Variables
static double CURRENT_LAT = 0; // Initialize current latitude to 0
static double CURRENT_LONG = 0; // Initialize current longitude to 0
volatile double CURRENT_HEADING = 0; // Initialize current heading of rover to 0 (Direct North)
volatile double TARGET_LAT = 0; // Initialize target latitude to 0
volatile double TARGET_LONG = 0; // Initialize target longitude to 0
volatile double DISTANCE = 0; // Initialize final distance variable to 21
volatile double TARGET_HEADING = 0; // Initialize target heading to 0
volatile float ANGLE_TURN = 0; // Initialize angle provisional to 0
const float HEADING_ERROR = 5; // Initialize margin of error for TurnToHeading()
const float HOLDING_ERROR = 5; // Initialize margin of error for HeadingHold()

const short ROWS = 4; // Number of rows in WAYPOINT_ARRAY
const short COLS = 3; // Number of columns in WAYPOINT_ARRAY
// Initialize the array of target coordinates with 0 = intermediate waypoint and 1 = cone location
//const float WAYPOINT_ARRAY[ROWS][COLS] = 
// {{34.0291156, -117.5027155, 0}, 
//  {34.0288818, -117.5026586, 0}, 
//  {34.0291156, -117.5027155, 1}};
const double WAYPOINT_ARRAY[ROWS][COLS] = 
 {{34.0289898, -117.5029493, 0}, 
  {34.0288814, -117.5026420, 0}, 
  {34.0286257, -117.5027913, 0},
  {34.0290408, -117.5027879, 1}};
int CHECKPOINT; // Declare the checkpoint tracker
int WAYPOINT = 0; // Initialize the waypoint tracker
/*----------------------------------------------------------------------------------------------------------------------*/
// RC controller variables
#define THROTTLE_PIN 2 // Initialize throttle pin
volatile long TH_START_TIME = 0; // Initialize starting timer
volatile long TH_CURRENT_TIME = 0; // Initialize current time
volatile long THROTTLE_PULSE = 0; // Inititalize throttle pulse width
int THROTTLE_PW = 0; // Initialize throttle pulse width updater
int THROTTLE_VALUE = 0; // Initialize throttle value
int THROTTLE_CENTER; // Declare the zero signal for throttle

#define STEERING_PIN 3 // Initialize steering pin
volatile long ST_START_TIME = 0; // Initialize starting timer
volatile long ST_CURRENT_TIME = 0; // Initialize current time
volatile long STEERING_PULSE = 0; // Initialize steering puls width
int STEERING_PW = 0; // Initialize steering pulse width updater
int STEERING_VALUE = 0; // Initialize steering servo value
int STEERING_CENTER; // Declare the zero signal for steering

#define DEAD_MAN_PIN 4 // Initialize RC dead man switch pin
volatile short DEAD_MAN_VALUE; // Declare value read from RC dead man buttons
bool RC_CONTROL = true; // Initialize RC_CONTROL boolean variable to TRUE
bool AUTON_CONTROL = false; // Initialize AUTON_CONTROL boolean variable to FALSE
/*----------------------------------------------------------------------------------------------------------------------*/
// Motor variables
static short VELOCITY = 90; // Declare VELOCITY as global to keep track of speed between function calls
Servo ESC_MOTOR; // Initialize ESC_MOTOR as a Servo object --> on pin 9
Servo TURN_SERVO; // Initialize TURN_SERVO as a Servo object --> on pin 8
/*----------------------------------------------------------------------------------------------------------------------*/
// Structural Timer Variables
millisDelay TIMER_50MS; // Initialize millisDelay timer object
millisDelay TIMER_110MS; // Initialize millisDelay timer object
millisDelay TIMER_1S; // Initialize millisDelay timer object
/*----------------------------------------------------------------------------------------------------------------------*/
/*
 * END VARIABLE INITIALIZATIONS AND DECLARATIONS
 */
 
void setup() {

  Serial.begin(115200);
  
  ESC_MOTOR.attach(9); // Set ESC_MOTOR to pin 9
  TURN_SERVO.attach(8); // Set TURN_SERVO to pin 8

  ESC_MOTOR.write(90);
  TURN_SERVO.write(90);

  //LEDSetup(); // Setup LED --> CANNOT USE UNTIL CENTER_TRIG IS REASSIGNED

  CompassSetup(); // Setup compass

  GPSSetup(); // Setup GPS serial communication

  RCSetup(); // Setup RC control
  RCRead(); // Read current signal from remote control
  THROTTLE_CENTER = THROTTLE_PW; // Set throttle zero point
  STEERING_CENTER = STEERING_PW; // Set steering zero point

  BluetoothSetup(); // Setup bluetooth telemetry

  LimitSwitchSetup(); // Setup limit switches

  UltrasonicSetup(); // Setup ultrasonic sensors

  SensorTimerSetup(); // Start timers to be used in loop()
    
  Serial.println("Warming Up, Please Turn On ESC.");
  Serial8.println("Warming Up, Please Turn On ESC.");

  delay(5000); // Send ESC and Servo 0 signal until program starts

  Serial.println("Begin Compass Calibration. Activate Right Limit Switch to Finish Calibration.");
  Serial8.println("Begin Compass Calibration. Activate Right Limit Switch to Finish Calibration.");

  delay(2000);
  
  while(true){
    CalibrateCompass(); // Calibrate the compass until rear and front right limit switches are activated
    
    // If limit switch is pressed, stop compass calibration
    if(digitalRead(RIGHT_LIMIT_SWITCH) == HIGH){
      // Establish compass bearing using calibration values from CalibrateCompass()
      COMPASS.m_min = (LSM303::vector<int16_t>){X_MIN, Y_MIN, Z_MIN};
      COMPASS.m_max = (LSM303::vector<int16_t>){X_MAX, Y_MAX, Z_MAX};
      break; // Exit calibration
    }
  }

  Serial.println("Waiting for GPS Lock...");
  Serial8.println("Waiting for GPS Lock...");

  // Wait until GPS lock is obtained
  while(CURRENT_LAT == 0){
    CurrentCoordinates(); // Get initial GPS coordinates of the rover
    CurrentHeading(); // Get initial heading of the rover
    UpdateTargetWaypoint(WAYPOINT); // Update target coordinates
    GPSUpdate(); // Calculate initial DISTANCE and TARGET_HEADING
    TurningAngle(); // Calculate initial turning angle ANGLE_TURN
  }

  delay(1000);

  Serial.println("GPS Lock Confirmed.");
  Serial8.println("GPS Lock Confirmed.");

//  delay(1000);
//
//  Serial.println("Please select rover mode");
//  Serial8.println("Please select rover mode");
//  DeadManSwitch();
//
//  delay(5000);
//  
//  if(RC_CONTROL == true){
//    Serial.print("Starting Program in RC Control in 5 ");
//    delay(1000);
//    Serial.print("4 ");
//    delay(1000);
//    Serial.print("3 ");
//    delay(1000);
//    Serial.print("2 ");
//    delay(1000);
//    Serial.print("1 ");
//
//    Serial8.print("Starting Program in RC Control in 5 ");
//    delay(1000);
//    Serial8.print("4 ");
//    delay(1000);
//    Serial8.print("3 ");
//    delay(1000);
//    Serial8.print("2 ");
//    delay(1000);
//    Serial8.print("1 ");
//  }
//  else if(AUTON_CONTROL = true){
//    Serial.print("Starting Program in Auton Control in 5 ");
//    delay(1000);
//    Serial.print("4 ");
//    delay(1000);
//    Serial.print("3 ");
//    delay(1000);
//    Serial.print("2 ");
//    delay(1000);
//    Serial.print("1 ");
//
//    Serial8.print("Starting Program in Auton Control in 5 ");
//    delay(1000);
//    Serial8.print("4 ");
//    delay(1000);
//    Serial8.print("3 ");
//    delay(1000);
//    Serial8.print("2 ");
//    delay(1000);
//    Serial8.print("1 ");
//  }
}

void loop() {

  GPSNavigation();
  
  // Use RC toggle to enter program

///*----------------------------------------------------------------------------------------------------------------------*/
//  DeadManSwitch(); // Check for toggle state
//  if(RC_CONTROL == true){ // Enable driver control if top button is pressed
//
//    SensorTimers(); // Check sensors on regular timing intervals
//    
//    RCDrive(); // Drive through remote control
//  }
///*----------------------------------------------------------------------------------------------------------------------*/
//  else if(AUTON_CONTROL == true){ // Enter autonomous routine if bottom button is pressed
//
//    // CANNOT RETURN TO RC CONTROL ONCE IN GPS NAVIGATION
//    GPSNavigation(); // Enter GPS Navigation (without collision detection)
//    
//  }
///*----------------------------------------------------------------------------------------------------------------------*/
//  else{ // If toggle switch has not been properly detected, do nothing
//    Serial.println("ERROR: TOGGLE NOT DETECTED");
//    Serial8.println("ERROR: TOGGLE NOT DETECTED");
//  }

}
