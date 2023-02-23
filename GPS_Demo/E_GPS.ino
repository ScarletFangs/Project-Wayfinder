/*
 * Converting GPS Latitude and Longitude to distance to drive and target heading
 */

void CurrentCoordinates(){
  // Parse current latitude and longitude from GPS serial messages
  // Check if there is data available on the GPS module
  if (Serial3.available()>0) {
    // Read the data from the GPS module
    String data = Serial3.readStringUntil('\n');

    // Print the data to the serial monitor of the computer

    String inputString = data;
        
    //Serial.print(inputString);
    inputString.remove(12,1);
    int index1 = inputString.indexOf(",N"); // Find the index of "N"
    int index2 = inputString.indexOf(",W"); // Find the index of "W"
    
    // Extract the latitude value and convert it to a float
    String latString = inputString.substring(index1-10, index1);
    //Serial.println(latString);
    CURRENT_LAT = atof(latString.c_str()) / 100;
    // Extract the longitude value and convert it to a float
    String lonString = inputString.substring(index2-11, index2);
    CURRENT_LONG = -1 * atof(lonString.c_str()) / 100;
    //Serial.println(CURRENT_LONG, 7);
  }
}

void CurrentHeading(){
  // Grab current heading of the rover from compass and update global variable CURRENT_HEADING
  COMPASS.read();
  CURRENT_HEADING = COMPASS.heading();
}
 
void GPSUpdate(float initial_lat, float initial_long, float final_lat, float final_long){
  /*
   * Uses haversine formula to calculate distance between two points when given latitudes and longitudes
   */
  // CONST: Do not allow this variable to be changed by anything
  const double R = 6371e3; // meters --> radius of the earth (assumed spherical geometry)
  // convert GPS coordinates from degrees to radians
  initial_lat = initial_lat * M_PI / 180;
  initial_long = initial_long * M_PI / 180;
  final_lat = final_lat * M_PI / 180;
  final_long = final_long * M_PI / 180;
  
  LAT_DIFF = final_lat - initial_lat; // Difference between latitudes of two points
  LONG_DIFF = final_long - initial_long; // Difference between longitudes of two points

  // Argument of the square root in haversine formula
  A = (pow(sin(LAT_DIFF/2), 2)) + cos(initial_lat)*cos(final_lat)*(pow(sin(LONG_DIFF/2), 2));
  
  C = 2*atan2(sqrt(A), sqrt(1-A)); // Great circle distance in radians using: atan2(x, y) = arctan(y/x)
 
  DISTANCE = R * C; // meters --> Final distance calculation

  // Calculate the target heading for the rover to turn to based on target GPS coordinates
  TARGET_HEADING = atan2((cos(initial_lat)*sin(final_lat)) - (sin(initial_lat)*cos(final_lat)*cos(LONG_DIFF)), 
                   sin(LONG_DIFF)*cos(final_lat));

  TARGET_HEADING = TARGET_HEADING * 180 / M_PI; // Convert target heading from radians to degrees (given in -180-180 range)
  TARGET_HEADING = fmod(TARGET_HEADING + 360.0, 360); // Normalize to 0-360 compass bearing
  // fmod = floating point modulus % --> fmod(x, y) == x % y
 }

void TurningAngle(float target_heading, float current_heading){ // function to determine Turning Angle from CURRENT_HEADING & TARGET_HEADING
  
  ANGLE_PROVISIONAL = target_heading - current_heading; // Take the difference of target heading and current heading

  if (ANGLE_PROVISIONAL <= 180 && ANGLE_PROVISIONAL>-180){ 
      ANGLE_TURN = ANGLE_PROVISIONAL;
  }
  else if (ANGLE_PROVISIONAL>180){
      ANGLE_TURN = ANGLE_PROVISIONAL-360;
  }
  else if (ANGLE_PROVISIONAL<=-180){
      ANGLE_TURN = ANGLE_PROVISIONAL+360;
  }
  
}

void SetVelocity(int target_velocity, unsigned long time_interval){
  // Using a timer to set velocity gradually

  if(CHECK == 0){
    VELOCITY_TIMER.start(time_interval); // start timer
    CHECK++; // increment logical check so that timer is not restarted at every function call
  }
  else if(CHECK == 1){
    VELOCITY_TIMER.start(REMAINING_VELOCITY_TIME); // continue timer with previous remaining time
  }
  
  //Serial.print("Initial Timer Check: ");
  //Serial.println(VELOCITY_TIMER.delay());
  if(VELOCITY < target_velocity){ // if velocity value has to increase
    if(VELOCITY_TIMER.delay() < time_interval / 4){
      // if time interval has been reached --> NOTE: timer seems to jump over 0, so instead stopping it a bit short
      //Serial.print("Speeding Up: ");
      //Serial.println(VELOCITY);
      VELOCITY++; // increase velocity
      ESC_MOTOR.write(VELOCITY); // send velocity value to motor
      CHECK = 0; // logical check to restart timer after velocity has been reached
    }
    else{
      //Serial.print("Check 1: ");
      //Serial.println(VELOCITY_TIMER.delay());
    }
  }
  else if(VELOCITY > target_velocity){ // if velocity value has to decrease
    if(VELOCITY_TIMER.delay() < time_interval / 4){
      // if time interval has been reached --> NOTE: timer seems to jump over 0, so instead stopping it a bit short
      //Serial.print("Slowing Down: ");
      //Serial.println(VELOCITY);
      VELOCITY--; // decrease velocity
      ESC_MOTOR.write(VELOCITY); // send velocity value to motor
      CHECK = 0; // logical check to restart timer after velocity has been reached
    }
    else{
      //Serial.print("Check 2: ");
      //Serial.println(VELOCITY_TIMER.delay());
    }
  }
  REMAINING_VELOCITY_TIME = VELOCITY_TIMER.remaining(); // save current time remaining in timer
  VELOCITY_TIMER.stop(); // freeze timer outside of function calls
}

/* MOTOR SPEEDS AND DIRECTIONS BASED OFF PULSE WIDTH MODULATION VALUES
* 0 <= speed <= 90 == reverse direction for motors
* 90 <= speed <= 180 == forward direction for motors
* 90 == stop motors
* 0 == full speed in reverse
* 180 == full speed forwards
*/
void TurnLeft(int speed_, long timer){ // takes in speed for direction and timer for duration
  if (speed_ <= 180 && speed_ >= 90){ //if forward turn wheels 
    TURN_SERVO.write(0);
    //SetVelocity(speed_, timer);
    ESC_MOTOR.write(110);
  }else {
      TURN_SERVO.write(180);
      //SetVelocity(speed_, timer);
  }
}

void TurnRight(int speed_, long timer){ // takes in speed for direction and timer for duration
  if (speed_ <= 180 && speed_ >= 90){ //if forward turn wheels 
    TURN_SERVO.write(180);
    //SetVelocity(speed_, timer);
    ESC_MOTOR.write(110);
  }else {
      TURN_SERVO.write(0);
      //SetVelocity(110, timer);
  }
}

void StopRover(){
   ESC_MOTOR.write(90); // stop
}

 void TurnToHeading(){
  // Uses GPSUpdate() to turn rover towards a desired heading
  //Serial.println("Current Coordinates");
  CurrentCoordinates(); // update current latitude and longitude of the rover
  //Serial.println("Current Heading");
  CurrentHeading(); // update current heading of the rover
  //Serial.println("GPSUpdate");
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // Calculate the target distance and heading from current coordinates and target coordinates
  //Serial.println("Turning Angle");
  TurningAngle(TARGET_HEADING, CURRENT_HEADING); // Calculate turning angle by using current Target heading and current heading

  if (ANGLE_TURN > 0){ // if target heading is to the right of the current heading
   do { // turn right while turn angle difference is greater than 3 degrees
      CurrentHeading(); // update current heading of the rover
      Serial.print("Loop1-Current Heading: ");
      Serial.println(CURRENT_HEADING);
      Serial.print("Loop1-Target Heading: ");
      Serial.println(TARGET_HEADING);
      Serial.print("Loop1-Angle Difference: ");
      Serial.println(abs(CURRENT_HEADING-TARGET_HEADING));
      Serial.println();
      TurnRight(120, 100); // TurnRight(speed, duration)
   }
   while(abs(CURRENT_HEADING-TARGET_HEADING)>=3);
  }else{ // if target heading is to the left of current heading
    do { // turn left while angle difference is greater than 3 degrees
        CurrentHeading(); // update current heading of the rover
        Serial.print("Loop2-Current Heading: ");
        Serial.println(CURRENT_HEADING);
        Serial.print("Loop2-Target Heading: ");
        Serial.println(TARGET_HEADING);
        Serial.print("Loop2-Angle Difference: ");
        Serial.println(abs(CURRENT_HEADING-TARGET_HEADING));
        Serial.println();
        TurnLeft(120, 100); // TurnLeft(speed, duration)
    }
    while(abs(CURRENT_HEADING-TARGET_HEADING)>=3);
  }
  
  
 }
