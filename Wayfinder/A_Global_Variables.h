/*
 * All initializations and declarations of variables
 */
bool BEGIN_PROGRAM = false; // Initialize start check
/*----------------------------------------------------------------------------------------------------------------------*/
// Debugging Variables (1 = turn on prints, 0 = turn off prints)
#define DEBUG_MODE 1 // Initialize "debug mode" condition for setup (removes all blocking checks in setup)
#define SERIAL_DEBUG 0 // Initialize serial monitor debugging condition
#define BLUETOOTH_DEBUG 0 // Initialize Bluetooth debugging condition
/*----------------------------------------------------------------------------------------------------------------------*/
// Collision Response Variables
elapsedMillis COLLISION_TIMER; // Declare elapsedMillis timer object
const int DELAY500MS = 500; // Delay interval
const int DELAY1500MS = 1500; // Delay interval
volatile uint16_t TOTALDELAY = 0; // Total delay variable to track changes in collision response routine
bool COLLISION_FINISHED = true; // Check if collision response has finished
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit Switch variables
#define LEFT_LIMIT_SWITCH 21 // Initialize front left limit switch
#define RIGHT_LIMIT_SWITCH 22 // Initialize front right limit switch
#define REAR_LIMIT_SWITCH 23 // Initialize rear limit switch

bool LIMIT_COLLISION_FRONT = false; // Initialize tracker for front limit switch collision detection
bool LIMIT_COLLISION_REAR = false; // Initialize tracker for rear limit switch collision detection
const int FRONT = 5; // Initialize directionality call
const int REAR = 6; // Initialize directionality call
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic variables
#define LEFT_TRIG 38    // Trigger Pin of Left Ultrasonic Sensor to pin 16
#define LEFT_ECHO 36    // Echo Pin of Left Ultrasonic Sensor to pin 17
#define LEFT_US 0       // Left US position

#define CENTER_TRIG 41  // Trigger Pin of Center Ultrasonic Sensor to pin 36
#define CENTER_ECHO 40  // Echo Pin of Center Ultrasonic Sensor to pin 38
#define CENTER_US 1     // Center US position

#define RIGHT_TRIG 16   // Trigger Pin of Right Ultrasonic Sensor to pin 41
#define RIGHT_ECHO 17   // Echo Pin of Right Ultrasonic Sensor to pin 40
#define RIGHT_US 2      // Right US position

#define SONAR_NUM 3 // Number of ultrasonic sensors

unsigned int DISTANCE_ARRAY[SONAR_NUM] = {0, 0, 0}; // Where the ping distances are updated and check in UltrasonicCollision()
uint8_t CURRENT_SENSOR = 0;         // Keeps track of which sensor is active
unsigned int GLOBAL_MIN_SENSOR = 0; // Sensor with the smallest distance 0 = Right Sensor; 1 = Middle Sensor; 2 = Left Sensor 
unsigned int OBS_DISTANCE = 30; // Minimum distance for ultra sonics to a avoid obstacle
unsigned int EOBS_DISTANCE = 50; // Minimum distance for early obstacle detection

bool ULTRASONIC_COLLISION = false; // Initialize tracker for ultrasonic collision detection
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
volatile double CURRENT_HEADING = 0; // Initialize current heading of rover to 0 degrees
volatile double TARGET_LAT = 0; // Initialize target latitude to 0
volatile double TARGET_LONG = 0; // Initialize target longitude to 0
volatile double DISTANCE = 0; // Initialize distance variable to 0
volatile double TARGET_HEADING = 0; // Initialize target heading to 0
volatile double ANGLE_TURN = 0; // Initialize angle provisional to 0
const double HEADING_ERROR = 10; // Initialize margin of error for TurnToHeading()

bool COMPASS_PID = true;
volatile double COMPASS_ERROR = 0;
volatile double COMPASS_DERIVATIVE = 0;
const double CINTEGRAL_BOUND = 7.5;
volatile double COMPASS_prevERROR = 0;
volatile double COMPASS_TOTAL_ERROR = 0;
volatile double COMPASS_LOCK = 0;
// THE GOLDEN RATIO DO NOT TOUCH
const double cp = 0.75;
const double ci = 0.3;
const double cd = 0.35;

// Initialize check to go to TurnToHeading() --> 0 = Start in HeadingHold(), 1 = Start in TurnToHeading()
volatile int TURN_TO = 0; 
/*----------------------------------------------------------------------------------------------------------------------*/
/* Initialize the array of target coordinates with third column denoting waypoint type
 *  0 = Intermediate waypoint --> Enters TurnToHeading() for next point
 *  1 = Quick transition point --> Skips TurnToHeading() for next point
 *  9 = Cone location --> Activates Vision at the end of the current point
 */
const short COLS = 3; // Number of columns in WAYPOINT_ARRAY --> DO NOT CHANGE THIS

// Initialize all waypoint states
const short TRANSITION = 1;
const short FAST_TRANSITION = 2;
const short MANUAL = 9;
const short BLACK_BOX = 99;

const short NUM_WAYPOINTS = 9; // Number of waypoints
const double WAYPOINT_ARRAY[NUM_WAYPOINTS][COLS] = 
{{37.3956998, -121.5314804, TRANSITION}};

 
unsigned long STARTUP_TIMER = 0; // Initialize the startup timer
unsigned long COURSE_TIMER = 0; // Initialize the course timer
unsigned long CONE_TIMER = 0; // Initialize the cone search timer

bool FINISHED = false; // Initialize check for completion of course  
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
bool DEAD_MAN = false; // Initialize DEAD_MAN to stop rover mid-autonomous run
bool RC_CONTROL = false; // Initialize RC_CONTROL to switch into RC_CONTROL
bool AUTON_CONTROL = true; // Initialize AUTON_CONTROL boolean variable to FALSE

#define RC_TOGGLE 0 // Initialize RC_TOGGLE check (1 == Check for remote control, 0 == Don't check for remote control)
/*----------------------------------------------------------------------------------------------------------------------*/
// Motor variables
Servo ESC_MOTOR; // Initialize ESC_MOTOR as a Servo object --> on pin 9
Servo TURN_SERVO; // Initialize TURN_SERVO as a Servo object --> on pin 8
//static short VELOCITY = 90; // Declare VELOCITY as global to keep track of speed between function calls

short TURN_SPEED = 83; // Initialize speed of TurnToHeading()
short HOLD_SPEED = 98; // Initialize minimum speed of HeadingHold()
short ALIGN_SPEED = 97; // Initialize speed of alignment in AlignRover()
short PURSUIT_SPEED = 97; // Initialize speed of pursuit in PursueCone()

int VISION_STEERING_VALUE; // Initialize steering value for Vision sensor program

bool ALTERNATE_FRONT = false; // Initialize alternate front collision response
bool ALTERNATE_BACK = false; // Initialize alternate back collision response
/*----------------------------------------------------------------------------------------------------------------------*/
// Structural Timer Variables
elapsedMillis LS_TIMER; // Declare elapsedMillis timer object
#define LS_DELAY 5 // Initialize length of LS_TIMER

elapsedMillis US_TIMER; // Declare elapsedMillis timer object
#define US_DELAY 20 // Initialize length of US_TIMER
volatile unsigned long PING_TIMER; // Non-blocking microsecond timer to send ultrasonic pings

elapsedMillis COM_TIMER; // Declare elapsedMillis timer object
#define COM_DELAY 20 // Initialize length of COM_TIMER

elapsedMillis GPS_TIMER; // Declare elapsedMillis timer object
#define GPS_DELAY 1000 // Initialize length of GPS_TIMER

volatile unsigned long EMERGENCY_TIMER; // Declare the emergency timer that saves rover from being stuck
// Initialize the step checking variables for restarting EMERGENCY_TIMER
volatile uint8_t EMERGENCY_CHECK = 0;
volatile uint8_t NEW_STEP = 1;
volatile uint8_t CURRENT_STEP = NEW_STEP;
/*----------------------------------------------------------------------------------------------------------------------*/
// Vision Sensor Variables
Pixy2SPI_SS pixy; // Pixy2 camera object
#define VISION_ERROR_MARGIN 17 // Error margin for checking OBJECT_CENTER_TO_CAM --> Avoid multiples of 5
#define CAMERA_CENTER 157 // Pixel center of camera screen
#define ROVER_CENTER_ERROR 23 // Error margin for checking if rover is aligned with cone --> Avoid multiples of 5
bool OBJECT_CENTER_TO_CAM; // Check for whether camera is centered to the screen

unsigned int CENTER_OF_OBJECT = 0; // Initialize variable for tracking center of object detected
int DISTANCE_TILL_CENTER = 0; // Distance between CAMERA_CENTER & CENTER_OF_OBJECT
static int VISION_ANGLE; // Pan servo angle of vision sensor --> 0 == RIGHT, 1000 == LEFT
const int ROVER_CENTER = 420; // Angle at which vision sensor is aligned with rover
volatile int CAMERA_ROVER_DIFF = 0; // Difference between VISION_ANGLE & ROVER_CENTER

// Vision states using enumeration
enum visionState
{
  REST,   // Turn off vision sensor
  SEARCH, // Search for the cone
  ALIGN,  // Align rover with cone detected
  PURSUIT // Drive into the cone detected
};

enum visionState STATE;
