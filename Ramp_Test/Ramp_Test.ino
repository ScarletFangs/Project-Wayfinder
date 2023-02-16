/*
 * Test file for ramp up/down driving function
 */
/*----------------------------------------------------------------------------------------------------------------------*/
#include <Servo.h>
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

unsigned long RAMP_START_TIME = 0; // starting ramp time
unsigned long RAMP_CURRENT_TIME = 0; // current ramp time

/*----------------------------------------------------------------------------------------------------------------------*/
void SetVelocity(int target_velocity, long time_interval){
  // Using a timer to set velocity gradually
  RAMP_CURRENT_TIME = millis(); // start timer in milliseconds
  if(VELOCITY < target_velocity){ // if wanting to speed up
    if(RAMP_CURRENT_TIME == time_interval){
      // if time interval has been reached
      Serial.print("Speeding Up: ");
      Serial.println(VELOCITY);
      VELOCITY++; // increase velocity
      ESC_MOTOR.write(VELOCITY); // send velocity value to motor
      RAMP_CURRENT_TIME = RAMP_START_TIME; // restart timer
    }
    else{
      Serial.print("Check 1: ");
      Serial.println(RAMP_CURRENT_TIME);
    }
  }
  else{ // if wanting to slow down
    if(RAMP_CURRENT_TIME == time_interval){
      // if time interval has been reached
      Serial.print("Slowing Down: ");
      Serial.println(VELOCITY);
      VELOCITY--; // increase velocity
      ESC_MOTOR.write(VELOCITY); // send velocity value to motor
      RAMP_CURRENT_TIME = RAMP_START_TIME; // restart timer
    }
    else{
      Serial.println("Check 2");
    }
  }
}

void setup() {
  Serial.begin(9600);

  ESC_MOTOR.attach(8); // set ESC_MOTOR to pin 8
  TURN_SERVO.attach(9); // set TURN_SERVO to pin 9

  LEDSetup(); // turn on on-board LED to ensure program is running

}

void loop() {

  Serial.println("In Loop");
  SetVelocity(110, 2500); // SetVelocity(target speed, timer interval [ms])
  
}
