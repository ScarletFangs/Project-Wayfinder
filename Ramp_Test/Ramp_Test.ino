/*
 * Test file for ramp up/down driving function
 */
#include <Servo.h>
#include <millisDelay.h>
/*----------------------------------------------------------------------------------------------------------------------*/
// Drive motor and steering servo variables
volatile int VELOCITY = 90; // declare VELOCITY as global to keep track of speed between function calls
Servo ESC_MOTOR; // initialize ESC_MOTOR as a Servo object --> on pin 8
Servo TURN_SERVO; // initialize TURN_SERVO as a Servo object --> on pin 9

/* MOTOR SPEEDS AND DIRECTIONS BASED OFF PULSE WIDTH MODULATION VALUES
* 0 <= speed <= 90 == reverse direction for motors
* 90 <= speed <= 180 == forward direction for motors
* 90 == stop motors
* 0 == full speed in reverse
* 180 == full speed forwards
*/

millisDelay PROGRAM_STARTUP;
millisDelay VELOCITY_TIMER;
volatile long REMAINING_VELOCITY_TIME = 0;
volatile int CHECK = 0;

/*----------------------------------------------------------------------------------------------------------------------*/
void SetVelocity(int target_velocity, long time_interval){
  // Using a timer to set velocity gradually

  if(CHECK == 0){
    VELOCITY_TIMER.start(time_interval);
    CHECK++;
  }
  else if(CHECK == 1){
    VELOCITY_TIMER.start(REMAINING_VELOCITY_TIME);
  }
  
  //Serial.print("Initial Timer Check: ");
  //Serial.println(VELOCITY_TIMER.delay());
  if(VELOCITY < target_velocity){
    if(VELOCITY_TIMER.delay() < time_interval / 4){
      // if time interval has been reached
      Serial.print("Speeding Up: ");
      Serial.println(VELOCITY);
      VELOCITY++; // increase velocity
      ESC_MOTOR.write(VELOCITY); // send velocity value to motor
      CHECK = 0;
    }
    else{
      //Serial.print("Check 1: ");
      //Serial.println(VELOCITY_TIMER.delay());
    }
  }
  else if(VELOCITY == target_velocity){
    CHECK = 2;
    Serial.print("Velocity Set: ");
    Serial.println(VELOCITY);
    return 0;
  }
  else{
    if(VELOCITY_TIMER.delay() < time_interval / 4){
      // if time interval has been reached
      Serial.print("Slowing Down: ");
      Serial.println(VELOCITY);
      VELOCITY--; // decrease velocity
      ESC_MOTOR.write(VELOCITY); // send velocity value to motor
      CHECK = 0;
    }
    else{
      //Serial.print("Check 2: ");
      //Serial.println(VELOCITY_TIMER.delay());
    }
  }
  REMAINING_VELOCITY_TIME = VELOCITY_TIMER.remaining();
  VELOCITY_TIMER.stop();
}

void setup() {
  Serial.begin(9600);

  ESC_MOTOR.attach(8); // set ESC_MOTOR to pin 8
  TURN_SERVO.attach(9); // set TURN_SERVO to pin 9

  LEDSetup(); // turn on on-board LED to ensure program is running
  delay(2000);
  Serial.println("In Loop");
}

void loop() {

  //Serial.println("In Loop");
  // NOTE: delay interval is not working as expected. The current value of 100 ms has an actual delay of several seconds
  SetVelocity(110, 100); // SetVelocity(target speed, timer interval [ms])
  
}
