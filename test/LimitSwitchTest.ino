/*
 * Test file to see if rover responds correctly to limit switch input 
 */

#include <Servo.h>

#define LEFT_LIMIT_SWITCH 7 // initialize limit switch & pin number
#define RIGHT_LIMIT_SWITCH 8 // initialize limit switch & pin number
Servo escMotor; // initialize escMotor as a Servo object

/* MOTOR SPEEDS AND DIRECTIONS BASED OFF PULSE WIDTH MODULATION VALUES
* 0 <= speed <= 90 == reverse direction for motors
* 90 <= speed <= 180 == forward direction for motors
* 90 == stop motors
* 0 == full speed in reverse
* 180 == full speed forwards
*/
 
void setup() {
  Serial.begin(9600);
  pinMode(LEFT_LIMIT_SWITCH, INPUT_PULLUP); // set the pin to accept inputs
  pinMode(RIGHT_LIMIT_SWITCH, INPUT_PULLUP); // set the pin to accept inputs
  // Note:
  // Limit switches need to be set to "INPUT_PULLUP" this is due to
  // when their circuit is disconnected they have noise that will mess with the actual values
}
// 
void loop() {
//
//  static uint16_t velocity = 90;
//
//  // SETTING THE MOTOR SPEED
//  
//  if(velocity <= 100){ // while rover is not yet at the desired speed
//    velocity++; // gradually increase speed until target speed is reached
//  }
//
//  escMotor.write(velocity); // pass velocity reading to the motor



  // CHECKING LIMIT SWITCH INPUT
  if (digitalRead(LEFT_LIMIT_SWITCH) == LOW || digitalRead(RIGHT_LIMIT_SWITCH) == LOW) // if limit switch is triggered
  {
    Serial.println("A");
  // enter Turn() routine
  }
  else if (digitalRead(LEFT_LIMIT_SWITCH) == HIGH || digitalRead(RIGHT_LIMIT_SWITCH) == HIGH)
  {
    Serial.println("N"); // Do nothing otherwise
  }
  delay(20);
}
