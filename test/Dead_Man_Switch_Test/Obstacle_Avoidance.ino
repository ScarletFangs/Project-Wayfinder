/*
 * Checking functions for collision detection and obstacle avoidance
 */

void Turn(){ // collision response function
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

void LimitSwitchCheck(){ // limit switch collision detection
  // CHECKING LIMIT SWITCH INPUT
  if (digitalRead(LEFT_LIMIT_SWITCH) == HIGH || digitalRead(RIGHT_LIMIT_SWITCH) == HIGH) // if limit switch is triggered
  {
    Serial.println("A");
    Turn(); // enter Turn() routine
  }
  else
  {
    Serial.println("N"); // Do nothing otherwise
  }
}
