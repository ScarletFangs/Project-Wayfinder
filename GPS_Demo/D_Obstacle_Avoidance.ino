/*
 * Checking functions for collision detection and obstacle avoidance
 */
/*----------------------------------------------------------------------------------------------------------------------*/
void CollisionDetection(){
//  if(LimitSwitchCollision()){ // If a collision is detected
//    CollisionResponse(); // Enter the collision response routine
//  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Collision response routine
void CollisionResponse(){ // Collision response function
  // Stop --> Wait 1.5 sec --> Reverse turn 8 o'clock for 1.5 sec --> Stop
  
  ESC_MOTOR.write(90); // Stop
  delay(1500); // Wait 1.5 seconds for bot to stop (speed-dependent)
  
  TURN_SERVO.write(0); // Front wheels turn left, back wheels turn right
  VELOCITY = 80; // Slow reverse speed
  ESC_MOTOR.write(VELOCITY); // Reverse
  delay(1500); // Reverse for 1.5 seconds
  TURN_SERVO.write(90); // Wheels straight
  while(VELOCITY <= 90){ // While rover is not yet stopped
    delay(250);
    VELOCITY++; // gradually reduce speed until target speed is reached
    ESC_MOTOR.write(VELOCITY); // pass motor current velocity reading
  } // This loop is trying to "smooth" transition between backup and driving forward again
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit switch collision detection
void LimitSwitchCollision(){
  // CHECKING LIMIT SWITCH INPUT
  
  if (digitalRead(RIGHT_LIMIT_SWITCH) == HIGH || digitalRead(LEFT_LIMIT_SWITCH) == HIGH || digitalRead(REAR_LIMIT_SWITCH) == HIGH)  {
    Serial.println("Collision Detected: Limit Switch");
    CollisionResponse(); // Trigger CollisionResponse() if rover has collided with something
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic collision detection
void UltrasonicCollision(){
  // CHECKING ULTRASONIC READINGS
  
  if (DISTANCE_ARRAY[GLOBAL_MIN_SENSOR] < 30) // if ultrasonic is triggered below 20cm
  {
    Serial.println("Collision Detected: Ultrasonic");
    Serial.println(GLOBAL_MIN_SENSOR);
    Serial.println();
    CollisionResponse(); // Trigger CollisionResponse() if rover has collided with something
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
  
  unsigned int sensor_max_val = DISTANCE_ARRAY[0]; // Create temporary distance value for comparison
  unsigned int min_sensor = 0; // Assume smallest distance is detected by right sensor
  for (uint8_t i = 0; i < SONAR_NUM; i++){ // Loop through each forward-facing ultrasonic
    if (DISTANCE_ARRAY[i] < sensor_max_val) { // If a smaller distance value is reported by an ultrasonic sensor
         sensor_max_val = DISTANCE_ARRAY[i]; // Update the smallest value detected
         min_sensor = i; // 0 = left sensor; 1 = middle sensor; 2 = right Sensor 
      }
    
  }
  GLOBAL_MIN_SENSOR = min_sensor; // Update which sensor has the minimum distance
}
