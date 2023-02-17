/*
 * GPS Navigation Test File
 */
 #include <LSM303.h>
 #include <Wire.h>
 #include <Servo.h>
 #include <millisDelay.h>

LSM303 COMPASS; // initialize an LSM303 compass object

volatile float CURRENT_LAT = 0; // initialize current latitude reading
volatile float CURRENT_LONG = 0; // initialize current longitude reading
volatile float CURRRENT_HEADING = 0; // initialize current heading
volatile float LAT_DIFF = 0; // initialize latitude difference variable
volatile float LONG_DIFF = 0; // initialize longitude difference variable
volatile float A = 0; // initialize argument of the square root
volatile float C = 0; // initialize arctangent term
volatile float DISTANCE = 0; // initialize final distance variable
volatile float TARGET_HEADING = 0; // initialize target heading
volatile float ANGLE_PROVISIONAL = 0// initialize angle provisional
volatile float ANGLE_TURN = 0// initialize angle provisional
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
  // put your setup code here, to run once:
  ESC_MOTOR.attach(8); // set ESC_MOTOR to pin 8
  TURN_SERVO.attach(9); // set TURN_SERVO to pin 9

}

void loop() {
  // put your main code here, to run repeatedly:

}
