/*
 * GPS Navigation test file designed to navigate the rover to a waypoint using the GPS module & compass as the sole 
 * means of navigation. Obstacle avoidance functions have been included for use with RCDrive() only. Also includes 
 * telemetry through the means of a bluetooth module communicating with a cellphone. 
 */
#include <LSM303.h> // include compass library
#include <Wire.h> // include I2C library
#include <Servo.h> // include servo/motor library
#include <millisDelay.h> // include delay timer library (from SafeString library)
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
// Limit Switch variables
#define REAR_LIMIT_SWITCH 5 // initialize rear limit switch to pin 5
#define LEFT_LIMIT_SWITCH 6 // initialize front left limit switch to pin 6
#define RIGHT_LIMIT_SWITCH 23 // initialize front right limit switch to pin 7
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic variables
#define TRIG_PIN 22 // Trigger Pin of Ultrasonic Sensor to pin 22
#define ECHO_PIN 23 // Echo Pin of Ultrasonic Sensor to pin 23
IntervalTimer MY_TIMER_1; // declare timer that uses pin 10
IntervalTimer MY_TIMER_2; // declare timer that uses pin 11
/*----------------------------------------------------------------------------------------------------------------------*/
// GPS and Compass variables
LSM303 COMPASS; // declare an LSM303 compass object
millisDelay COMPASS_TIMER; // Declare COMPASS delay timer
short COMPASS_DELAY; // Declare COMPASS_TIMER delay interval

millisDelay GPS_TIMER; // Declare GPS delay timer
short GPS_DELAY; //Declare GPS_TIMER delay interval

static double CURRENT_LAT = 0; // initialize current latitude to 0
static double CURRENT_LONG = 0; // initialize current longitude to 0
volatile double CURRENT_HEADING = 0; // initialize current heading of rover to 0 (Direct North)
volatile double TARGET_LAT = 0; // initialize target latitude to 0
volatile double TARGET_LONG = 0; // initialize target longitude to 0
volatile double DISTANCE = 21; // initialize final distance variable to 21
volatile double TARGET_HEADING = 0; // initialize target heading to 0
volatile float ANGLE_TURN = 0; // initialize angle provisional to 0

const short ROWS = 3; // number of rows in WAYPOINT_ARRAY
const short COLS = 3; // number of columns in WAYPOINT_ARRAY
// Initialize the array of target coordinates with 0 = intermediate waypoint and 1 = cone location
float WAYPOINT_ARRAY[ROWS][COLS] = {{34.029115599999997, -117.502715499999993, 0}, {34.028881800000000, -117.502658599999989, 0}, {34.029115599999997, -117.502715499999993, 1}};

double TARGET_HEADING_ARRAY[4] = {90, 0, 270, 0};
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
volatile short DEAD_MAN_VALUE; // declare value read from RC dead man buttons
bool RC_CONTROL = true; // initialize RC_CONTROL boolean variable to TRUE
bool AUTON_CONTROL = false; // initialize AUTON_CONTROL boolean variable to FALSE
/*----------------------------------------------------------------------------------------------------------------------*/
// Motor variables
static short VELOCITY = 90; // declare VELOCITY as global to keep track of speed between function calls --> NOT CURRENTLY USED
Servo ESC_MOTOR; // initialize ESC_MOTOR as a Servo object --> on pin 9
Servo TURN_SERVO; // initialize TURN_SERVO as a Servo object --> on pin 8
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

  delay(5000); // Send ESC and Servo 0 signal until program starts

  LEDSetup(); // Setup LED

  CompassSetup(); // Setup compass

  GPSSetup(); // Setup GPS serial communication

  RCSetup(); // Setup RC control

  BluetoothSetup(); // Setup bluetooth telemetry

  delay(1000); // Wait while peripherals set up
}

void GPSNavigation(){
  // Use GPS navigation functions to run through a course autonomously
  
  for(int i = 0; i < ROWS; i++){ // Loop through each waypoint until the course has been completed
    
    TARGET_LAT = WAYPOINT_ARRAY[i][0]; // update TARGET_LAT
    TARGET_LONG = WAYPOINT_ARRAY[i][1]; // update TARGET_LONG
    int checkpoint = WAYPOINT_ARRAY[i][2]; // check to see if target coordinates is a cone or not
    
    if(checkpoint == 0){ // if this is an intermediate checkpoint
      // Drive fast through target
      Serial.println("Start");
      TurnToHeading(80, 10); // TurnToHeading(int ESC_MOTOR speed, int error margin of difference between CURRENT and TARGET headings)
      ESC_MOTOR.write(90); // stop drive motors before entering HeadingHold()
      delay(1000); // wait for 1 second before entering HeadingHold()
      while(DISTANCE >= 15){ // drive fast to target until within 10 meters
        HeadingHold(120); // HeadingHold(int ESC_MOTOR speed)
      }
      while(DISTANCE >= 5){ // drive to target until within 5 meters
        HeadingHold(110); // HeadingHold(int ESC_MOTOR speed)
      }
    }
    else if(checkpoint == 1){ // if this is a cone location
      // Drive to target then enter vision program
      TurnToHeading(80, 10); // TurnToHeading(int ESC_MOTOR speed, int error margin of difference between CURRENT and TARGET headings)
      ESC_MOTOR.write(90); // stop drive motors before entering HeadingHold()
      delay(1000); // wait for 1 second before entering HeadingHold()
      while(DISTANCE >= 10){ // drive fast to target until within 10 meters
        HeadingHold(120); // HeadingHold(int ESC_MOTOR speed)
      }
      while(DISTANCE >= 5){ // drive to target until within 5 meters
        HeadingHold(110); // HeadingHold(int ESC_MOTOR speed)
      }
      while(DISTANCE >=1){ // drive slow to target until within 1 meters
        HeadingHold(100); // HeadingHold(int ESC_MOTOR speed)
      }
      
      // ENTER VISION PROGRAM HERE

      // Use fake vision program until real program is completed
      while(true){
        // Drive in a circle backwards forever to mimic vision sensor search function
        Serial8.println("Searching For Cone");
        Serial.println("Searching For Cone");
        TURN_SERVO.write(0);
        ESC_MOTOR.write(80);
      }
      
    }

  } // END for(int i = 0; i < ROWS; i++)
}

void loop() {

//  CurrentCoordinates(); // Get current rover coordinates
//  if(CURRENT_LAT != 0){ // Do nothing until GPS updates
//    GPSNavigation();
//  }
  
  //DeadManSwitch(); // Update state of RC_CONTROL and AUTON_CONTROL
  //CurrentHeading();
  //Serial.println(CURRENT_HEADING);
  for(int i = 0; i < 3; i++){
    TARGET_HEADING = TARGET_HEADING_ARRAY[i];
    TurnToHeading(80, 5);
    ESC_MOTOR.write(90);
    delay(5000);
  }

//  if(RC_CONTROL){
//    Serial.println("RC");
//    RCDrive(); // return PWM values for RC throttle and steering  
//    LimitSwitchCollision(); // if limit switch is triggered, interrupt RC control and do collision response
//    UltrasonicCollision(); // if sonar is triggered, interrupt RC control and do collision response
//  }
//  else if(AUTON_CONTROL){
//    Serial.println("Auton");
//    GPSNavigation(); // Run through course autonomously
//    // NOTE: ONCE ENTERED YOU CANNOT RETURN TO RC CONTROL
//  }
//  else{ // In case something happens with the toggle switch
//    Serial8.println("ERROR: TOGGLE NOT DETECTED");
//    Serial.println("ERROR: TOGGLE NOT DETECTED");
//  }
}
