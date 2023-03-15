/*
 * Test file to see if rover responds correctly to limit switch input 
 */

#define LEFT_LIMIT_SWITCH 22 // initialize limit switch & pin number
#define RIGHT_LIMIT_SWITCH 23 // initialize limit switch & pin number

/* MOTOR SPEEDS AND DIRECTIONS BASED OFF PULSE WIDTH MODULATION VALUES
* 0 <= speed <= 90 == reverse direction for motors
* 90 <= speed <= 180 == forward direction for motors
* 90 == stop motors
* 0 == full speed in reverse
* 180 == full speed forwards
*/
 
void setup() {
  Serial.begin(9600);
  pinMode(LEFT_LIMIT_SWITCH, INPUT_PULLDOWN); // set the pin to accept inputs
  pinMode(RIGHT_LIMIT_SWITCH, INPUT_PULLDOWN); // set the pin to accept inputs
  // Note:
  // Limit switches need to be set to "INPUT_PULLDOWN" this is due to
  // when their circuit is disconnected they have noise that will mess with the actual values
}

void loop() {

  // CHECKING LIMIT SWITCH INPUT
  if (digitalRead(LEFT_LIMIT_SWITCH) == HIGH || digitalRead(RIGHT_LIMIT_SWITCH) == HIGH) // if limit switch is triggered
  {
    Serial.println("Activated");
  }
  else if (digitalRead(LEFT_LIMIT_SWITCH) == LOW || digitalRead(RIGHT_LIMIT_SWITCH) == LOW)
  {
    Serial.println("Nothing"); // Do nothing otherwise
  }
  delay(20);
}
