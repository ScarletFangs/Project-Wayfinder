/*
 * Checking functions for collision detection and obstacle avoidance
 */
/*----------------------------------------------------------------------------------------------------------------------*/
void CollisionDetection(){
  if(LimitSwitchCollision() || UltrasonicCollision()){ // if a collision is detected
    CollisionResponse(); // Enter the collision response routine
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Collision response routine
void CollisionResponse(){ // collision response function
  // stop --> wait 1.5 sec --> reverse turn 8 o'clock for 1.5 sec --> stop
  ESC_MOTOR.write(90); // stop
  delay(1500); // wait 1.5 seconds for bot to stop (speed-dependent)
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
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit switch collision detection
bool LimitSwitchCollision(){
  // CHECKING LIMIT SWITCH INPUT
  if (digitalRead(RIGHT_LIMIT_SWITCH) == LOW) // if limit switch is triggered
  {
    Serial.println("Collision Detected");
    return true;
  }
  else
  {
    return false; // Do nothing otherwise
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic collision detection
bool UltrasonicCollision(){
  // CHECKING ULTRASONIC TRIG
  if (UltrasonicCM(TRIG_PIN, ECHO_PIN) < 50) // if limit switch is triggered below 50cm
  {
    Serial.println("Collision Detected");
    return true;
  }
  else
  {
    return false; // Do nothing otherwise
  }
}
