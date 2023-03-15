/*
 * GPS Navigation test file designed to navigate the rover to a waypoint using the GPS module & compass as the sole 
 * means of navigation. Collision response functions have been included for use with RCDrive() only. ObstaceleAvoidance() 
 * is not currently used. Also includes telemetry through the means of a bluetooth module communicating with a cellphone. 
 * Gabriel Gardner - 03/15/2023
 */
#include <LSM303.h> // include compass library
#include <Wire.h> // include I2C library
#include <Servo.h> // include servo/motor library
#include <millisDelay.h> // include delay timer library (from SafeString library)
#include <NewPing.h> // include NewPing library for ultrasonics
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
#define SERIAL_DEBUG // Initialize serial monitor debugging condition
#define BLUETOOTH_DEBUG // Initialize Bluetooth debugging condition
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit Switch variables
#define REAR_LIMIT_SWITCH 21 // initialize rear limit switch
#define LEFT_LIMIT_SWITCH 22 // initialize front left limit switch
#define RIGHT_LIMIT_SWITCH 23 // initialize front right limit switch
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

#define SONAR_NUM 3 // Number of ultrasonic sensors
#define MAX_DISTANCE 500 // Maximum distance (in cm) to ping
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo)

unsigned long PING_TIMER_ARRAY[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor
unsigned int DISTANCE_ARRAY[SONAR_NUM];         // Where the ping distances are stored
uint8_t CURRENT_SENSOR = 0;          // Keeps track of which sensor is active
unsigned int GLOBAL_MIN_SENSOR = 0;         // Sensor with the smallest distance 0 = Right Sensor; 1 = Middle Sensor; 2 = Left Sensor 
unsigned int OBS_DISTANCE = 50; // Minimum distance for ultra sonics to a avoid obstacle

NewPing SONAR_TIME[SONAR_NUM] = // Sensor object array using NewPing library 
   {NewPing(LEFT_TRIG, LEFT_ECHO, MAX_DISTANCE),      // Left ultrasonic setup
    NewPing(CENTER_TRIG, CENTER_ECHO, MAX_DISTANCE),  // Right ultrasonic setup
    NewPing(RIGHT_TRIG, RIGHT_ECHO, MAX_DISTANCE)};   // Right ultrasonic setup
/*----------------------------------------------------------------------------------------------------------------------*/
// Compass variables
LSM303 COMPASS; // declare an LSM303 compass object
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
short X_MIN, Y_MIN, Z_MIN, X_MAX, Y_MAX, Z_MAX; // Declare compass calibration values
char report[80]; // Declare report size for printing calibration values
/*----------------------------------------------------------------------------------------------------------------------*/
// GPS Variables
static double CURRENT_LAT = 0; // initialize current latitude to 0
static double CURRENT_LONG = 0; // initialize current longitude to 0
volatile double CURRENT_HEADING = 0; // initialize current heading of rover to 0 (Direct North)
volatile double TARGET_LAT = 0; // initialize target latitude to 0
volatile double TARGET_LONG = 0; // initialize target longitude to 0
volatile double DISTANCE = 0; // initialize final distance variable to 21
volatile double TARGET_HEADING = 0; // initialize target heading to 0
volatile float ANGLE_TURN = 0; // initialize angle provisional to 0
const float HEADING_ERROR = 5; // Initialize margin of error for TurnToHeading()
const float HOLDING_ERROR = 5; // Initialize margin of error for HeadingHold()

const short ROWS = 3; // number of rows in WAYPOINT_ARRAY
const short COLS = 3; // number of columns in WAYPOINT_ARRAY
// Initialize the array of target coordinates with 0 = intermediate waypoint and 1 = cone location
float WAYPOINT_ARRAY[ROWS][COLS] = {{34.029115599999997, -117.502715499999993, 0}, {34.028881800000000, -117.502658599999989, 0}, {34.029115599999997, -117.502715499999993, 1}};
int CHECKPOINT; // Declare the checkpoint tracker
int WAYPOINT = 0; // Initialize the waypoint tracker
/*----------------------------------------------------------------------------------------------------------------------*/
// RC controller variables
#define THROTTLE_PIN 2 // initialize throttle pin
volatile long TH_START_TIME = 0; // initialize starting timer
volatile long TH_CURRENT_TIME = 0; // initialize current time
volatile long THROTTLE_PULSE = 0; // inititalize throttle pulse width
int THROTTLE_PW = 0; // initialize throttle pulse width updater
int THROTTLE_VALUE = 0; // initialize throttle value
int THROTTLE_CENTER; // Declare the zero signal for throttle

#define STEERING_PIN 3 // initialize steering pin
volatile long ST_START_TIME = 0; // initialize starting timer
volatile long ST_CURRENT_TIME = 0; // initialize current time
volatile long STEERING_PULSE = 0; // initialize steering pulse width
int STEERING_PW = 0; // initialize steering pulse width updater
int STEERING_VALUE = 0; // initialize steering servo value
int STEERING_CENTER; // Declare the zero signal for steering

#define DEAD_MAN_PIN 4 // initialize RC dead man switch pin
volatile short DEAD_MAN_VALUE; // declare value read from RC dead man buttons
bool RC_CONTROL = true; // initialize RC_CONTROL boolean variable to TRUE
bool AUTON_CONTROL = false; // initialize AUTON_CONTROL boolean variable to FALSE
/*----------------------------------------------------------------------------------------------------------------------*/
// Motor variables
static short VELOCITY = 90; // declare VELOCITY as global to keep track of speed between function calls --> NOT CURRENTLY USED
Servo ESC_MOTOR; // initialize ESC_MOTOR as a Servo object --> on pin 9
Servo TURN_SERVO; // initialize TURN_SERVO as a Servo object --> on pin 8
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

  Serial.begin(9600);
  
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
  STEERING_CENTER = STEERING_PW; // set steering zero point

  BluetoothSetup(); // Setup bluetooth telemetry

  StructuralTimers(); // Start timers to be used in loop()
    
  Serial.println("Warming Up, Please Turn On ESC.");

  delay(5000); // Send ESC and Servo 0 signal until program starts

  Serial.println("Begin Compass Calibration. Activate Rear Limit Switch to Finish Calibration.");
  
  while(true){
    CalibrateCompass(); // Calibrate the compass until rear and front right limit switches are activated
        
    if(digitalRead(REAR_LIMIT_SWITCH) == HIGH && digitalRead(RIGHT_LIMIT_SWITCH) == HIGH){
      // If limit switch is pressed, continue with setup
        // Establish compass bearing using calibration values from CalibrateCompass()
      COMPASS.m_min = (LSM303::vector<int16_t>){X_MIN, Y_MIN, Z_MIN};
      COMPASS.m_max = (LSM303::vector<int16_t>){X_MAX, Y_MAX, Z_MAX};
      break;
    }
  }

  Serial.println("Collecting Starting Positional Data...");

  CurrentCoordinates(); // Get initial GPS coordinates of the rover
  CurrentHeading(); // Get initial heading of the rover
  UpdateTargetWaypoint(WAYPOINT); // Update target coordinates
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // Calculate initial DISTANCE and TARGET_HEADING
  TurningAngle(TARGET_HEADING, CURRENT_HEADING); // Calculate initial turning angle ANGLE_TURN

  Serial.println("Starting Program...");

  delay(1000); // Wait while peripherals set up
}

void loop() {
  // Use RC toggle to enter program

/*----------------------------------------------------------------------------------------------------------------------*/
  DeadManSwitch(); // Check for toggle state
  if(RC_CONTROL == true){ // Enable driver control   
    
    // Every 50ms, check compass and collision sensors
    if(TIMER_50MS.justFinished()){
      CurrentHeading(); // Update current heading of rover
      LimitSwitchCollision(); // Check if limit switches detected a collision
      TIMER_50MS.repeat(); // Restart TIMER_50MS
    }
  
    // Every 110ms,  check ultrasonics --> sonar sweep takes 99ms to complete
    if(TIMER_110MS.justFinished()){
      UltrasonicCollision(); // Check if ultrasonics detected a collision
      TIMER_110MS.repeat(); // Restart TIMER_110MS
    }
    
    CollisionDetection(); // Check for collisions
    RCDrive(); // Drive through remote control
  }
/*----------------------------------------------------------------------------------------------------------------------*/
  else if(AUTON_CONTROL == true){ // Enter autonomous routine    
    
    // Every 50ms, check compass
    if(TIMER_50MS.justFinished()){
      CurrentHeading(); // Update current heading of the rover
      TurningAngle(TARGET_HEADING, CURRENT_HEADING); // Calculate the difference in heading
      TIMER_50MS.repeat(); // Restart TIMER_50MS
    }
  
    // Every second, check current GPS coordinates
    if(TIMER_1S.justFinished()){
      CurrentCoordinates(); // Update current coordinates of the rover
      GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // Update DISTANCE and TARGET_HEADING
      TIMER_1S.repeat(); // Restart TIMER_1S
    }
    
    GPSNavigation(); // Enter GPS Navigation (without collision detection)
  }
/*----------------------------------------------------------------------------------------------------------------------*/
  else{ // If toggle switch has not been properly detected, do nothing
    Serial.println("ERROR: TOGGLE NOT DETECTED");
  }

}
