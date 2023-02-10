
/*
 * Cliff detection test file using one ultrasonic sensor
 */

#include <Servo.h>

const int PING_PIN = 22; // Trigger Pin of Ultrasonic Sensor
const int ECHO_PIN = 23; // Echo Pin of Ultrasonic Sensor
IntervalTimer MY_TIMER_1;
IntervalTimer MY_TIMER_2;

int VELOCITY = 90; // declare VELOCITY as global to keep track of speed between function calls
Servo ESC_MOTOR; // initialize ESC_MOTOR as a Servo object
Servo TURN_SERVO; // initialize TURN_SERVO as a Servo object

volatile long OBSTACLE = 0; // initialize cliff distance variable

void UltrasonicSetup(){
  // setup ultrasonic pins
  pinMode(PING_PIN, OUTPUT); //enable PING_PINg as an OUTPUT
  pinMode(ECHO_PIN, INPUT); //enable ECHO_PIN as an INPUT
  MY_TIMER_1.begin(PingPH, 2); //set PING_PIN voltage to high 5V every 2 microseconds
  MY_TIMER_2.begin(PingPL,10); //set PING_PIN volatage to low 0 volts (ground) 10 microsecodns
}

void PingPH(){
  digitalWrite(PING_PIN, HIGH); // set ping pin to HIGH
  }

void PingPL(){
  digitalWrite(PING_PIN, LOW); // set ping pin to LOW
  }

float UltrasonicIn(const int x, const int y){
    // returns distance of detected object in inches
    float duration;
    
    digitalWrite(x, LOW); //set PING_PIN volatage to low 0 volts (ground)  
    duration = pulseIn(y, HIGH); //return duration of pulse and set to duration
    return duration / 74 / 2; //use to convert to inches
}

float UltrasonicCM(const int x, const int y){
    // returns distance of detected object in centimeters
    float duration;
    
    digitalWrite(x, LOW); //set PING_PIN volatage to low 0 volts (ground)
    duration = pulseIn(y, HIGH); //return duration of pulse and set to duration
    // Sound travels at 343 meters per second, which means it needs 29.155 microseconds per centimeter. 
    // So, we have to divide the duration by 29 and then by 2, because the sound has to travel the distance twice. 
    // It travels to the object and then back to the sensor.
    return duration / 29 / 2; //use to convert to centimeters
}

void DriveForward(){
  if(VELOCITY <= 125){ // while rover is not yet at the desired speed
    delay(250);
    VELOCITY++; // gradually increase speed until target speed is reached
  }
  ESC_MOTOR.write(VELOCITY); // pass VELOCITY reading to the motor
}

void Turn(){ 
  // stop --> wait 0.5 sec --> reverse turn 8 o'clock for 1.5 sec --> stop
  ESC_MOTOR.write(90); // stop
  delay(500); // wait 0.5 seconds for bot to stop (speed-dependent)
  TURN_SERVO.write(0); // front wheels turn left, back wheels turn right
  VELOCITY = 80; // slow reverse speed
  ESC_MOTOR.write(VELOCITY); // reverse
  delay(1500); // reverse for 1.5 seconds
  TURN_SERVO.write(90); // wheels straight
  while(VELOCITY <= 90){ // while rover is not yet at the desired speed
    delay(250);
    VELOCITY++; // gradually reduce speed until target speed is reached
    ESC_MOTOR.write(VELOCITY); // pass motor current velocity reading
  } // this loop is trying to "smooth" transition between backup and driving forward again
}

void UltrasonicTest(){
  OBSTACLE = UltrasonicCM(PING_PIN, ECHO_PIN); // distance in cm from rover to the ground
  if(OBSTACLE < 30){ // while ultrasonic reads an object in the way
    Turn(); // enter collision detection routine
  }
}

void setup() {
  Serial.begin(9600); // Starting Serial Terminal
  ESC_MOTOR.attach(8); // attach esc motor to pin 8
  TURN_SERVO.attach(9); // attach steering servo to pin 9
  UltrasonicSetup(); // setup ultrasonics
}

void loop() {
  //DriveForward(); // drive forward until interrupted (250ms blocking delay)
  ESC_MOTOR.write(120);
  delay(10);
  UltrasonicTest(); // check for steep drops in terrain
}
