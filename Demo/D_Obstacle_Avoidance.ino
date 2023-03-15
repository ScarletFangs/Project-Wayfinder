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

  // If a limit switch is triggered
  if (digitalRead(RIGHT_LIMIT_SWITCH) == HIGH || digitalRead(LEFT_LIMIT_SWITCH) == HIGH || digitalRead(REAR_LIMIT_SWITCH) == HIGH)  {
    Serial.println("Collision Detected");
    return true; // return true if rover has collided with something
  }
  else
  {
    return false; // Do nothing otherwise
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic collision detection
bool UltrasonicCollision(){
  // CHECKING ULTRASONIC READINGS

  UltrasonicSweeping(); // Check front 3 sensors
  
  if (DISTANCE_ARRAY[0] < 10 || DISTANCE_ARRAY[1] < 10 || DISTANCE_ARRAY[2] < 10) // if ultrasonic is triggered below 10cm
  {
    Serial.println("Collision Detected");
    return true; // return true if collision imminent
  }
  else
  {
    return false; // Do nothing otherwise
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void UltrasonicSweeping(){ // Check front 3 ultrasonics in a sweeping fashion
   for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors
    if (millis() >= PING_TIMER_ARRAY[i]) { // Is it this sensor's time to ping?
      
      PING_TIMER_ARRAY[i] += PING_INTERVAL * SONAR_NUM; // Set next time this sensor will be pinged
      
      if (i == 0 && CURRENT_SENSOR == SONAR_NUM - 1){ 
        FindMinSensor(); // Sensor ping cycle complete, do something with the results
      }
      
      SONAR_TIME[CURRENT_SENSOR].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance)
      CURRENT_SENSOR = i;                          // Update sensor being accessed
      DISTANCE_ARRAY[CURRENT_SENSOR] = MAX_DISTANCE; // Make distance zero in case there's no ping echo for this sensor
      SONAR_TIME[CURRENT_SENSOR].ping_timer(EchoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo)
    }
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void EchoCheck(){ 
  // If ping received, update DISTANCE_ARRAY with the new distance
  
  if (SONAR_TIME[CURRENT_SENSOR].check_timer()){
    DISTANCE_ARRAY[CURRENT_SENSOR] = SONAR_TIME[CURRENT_SENSOR].ping_result / US_ROUNDTRIP_CM;
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void FindMinSensor() { 
  // After Sensor ping cycle complete, find the sensors with the smallest distance
  
  unsigned int sensor_max_val = DISTANCE_ARRAY[0]; // create temporary distance value for comparison
  unsigned int min_sensor = 0; // assume smallest distance is detected by right sensor
  for (uint8_t i = 0; i < SONAR_NUM; i++){ // loop through each forward-facing ultrasonic
    if (DISTANCE_ARRAY[i] < sensor_max_val) { // if a smaller distance value is reported by an ultrasonic sensor
         sensor_max_val = DISTANCE_ARRAY[i]; // update the smallest value detected
         min_sensor = i; // 0 = left sensor; 1 = middle sensor; 2 = right Sensor 
      }
    
  }
  GLOBAL_MIN_SENSOR = min_sensor; // Update which sensor has the minimum distance
}
