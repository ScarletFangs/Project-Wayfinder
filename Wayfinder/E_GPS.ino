/*
 * Converting GPS Latitude and Longitude to distance to drive and target heading to turn to. Encapsulating all of that 
 * into a single navigational routine. 
 */
/*----------------------------------------------------------------------------------------------------------------------*/
void CurrentCoordinates(){
  // Parse current latitude and longitude from GPS serial messages
  
  // Check if there is data available on the GPS module
  if (Serial3.available()>0) {
    // Read the data from the GPS module and save as a string
    String input_string = Serial3.readStringUntil('\n');
        
    input_string.remove(12,1); // Remove irrelevant characters from the beginning of the string
    short index1 = input_string.indexOf(",N"); // Look for the index of N
    short index2 = input_string.indexOf(",E"); // Look for the index of E
    bool south = false; // Initialize as false, otherwise false positives will occur
    bool west = false; // Initialize as false, otherwise false positives will occur

    // Check the directions of the parsed coordinate
    if (index1 == -1){ // If N isn't found, the coordinate is South
      index1 = input_string.indexOf(",S");
      south = true;
    }
    if (index2 == -1){ // If E isn't found, the coordinate is West
      index2 = input_string.indexOf(",W");
      west = true;
    }
    
    // Extract the latitude value and convert it to a float
    String lat_string = input_string.substring(index1-10, index1);
    double temp_lat; // declare temp_lat variable for assignment later
    
    if (south){ // If coordinate is South, make the value negative
      temp_lat = -1 * (atof(lat_string.c_str()) / 100); // if South, make negative
    }
    else{ // If coordinate is North, keep positive
      temp_lat = atof(lat_string.c_str()) / 100; // if North, make positive
    }
    
    if (temp_lat != 0){
      CURRENT_LAT = temp_lat; // update CURRENT_LAT only if given good data
    }
    
    // Extract the longitude value and convert it to a float
    String long_string = input_string.substring(index2-11, index2);
    double temp_long; // Declare temp_lat variable for assignment later
    
    if (west){ // If coordinate is West, make value negative
      temp_long = -1 * (atof(long_string.c_str()) / 100);
    }
    else{ // If coordinate is East, keep positive
      temp_long = atof(long_string.c_str()) / 100;
    }
    
    if(temp_long  != 0){
      CURRENT_LONG = temp_long; // Update CURRENT_LONG only if given good data
    }
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void CurrentHeading(){
  // Grab current heading of the rover from compass and update global variable CURRENT_HEADING
  COMPASS.read();
  CURRENT_HEADING = COMPASS.heading() + 14; // Account for difference between Magnetic North and True North
  if(CURRENT_HEADING >= 360){ // If above correction creates an angle > 360
    CURRENT_HEADING = fmod(CURRENT_HEADING, 360); // Flip over 360/0 boundary
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/ 
void GPSUpdate(){
  /*
   * Uses haversine formula to calculate distance and target heading between two points when given coordinates of 
   * current position and target position. Updates TARGET_HEADING and DISTANCE.
   */
  const double R = 6371e3; // meters --> radius of the earth (assumed spherical geometry)
  // Convert GPS coordinates from degrees to radians
  double initial_lat = CURRENT_LAT * M_PI / 180;
  double initial_long = CURRENT_LONG * M_PI / 180;
  double final_lat = TARGET_LAT * M_PI / 180;
  double final_long = TARGET_LONG * M_PI / 180;
  
  double lat_diff = final_lat - initial_lat; // Difference between latitudes of two points
  double long_diff = final_long - initial_long; // Difference between longitudes of two points

  // Argument of the square root in haversine formula
  double a = (pow(sin(lat_diff/2), 2)) + cos(initial_lat)*cos(final_lat)*(pow(sin(long_diff/2), 2));

  // Great circle distance in radians using: atan2(x, y) = arctan(y/x)
  double c = 2*atan2(sqrt(a), sqrt(1-a)); 
 
  DISTANCE = R * c; // meters --> Final distance calculation

  // Calculate the target heading for the rover to turn to based on target GPS coordinates
  double y = sin(long_diff)*cos(final_lat);
  double x = (cos(initial_lat)*sin(final_lat)) - (sin(initial_lat)*cos(final_lat)*cos(long_diff));
  TARGET_HEADING = atan2(y, x); // returns as -pi --> pi

  TARGET_HEADING = TARGET_HEADING * (180 / M_PI); // Convert target heading from radians to degrees (given in -180-180 range)
  TARGET_HEADING = fmod(TARGET_HEADING + 360.0, 360); // Normalize to 0-360 compass bearing
  // fmod = floating point modulus % --> fmod(x, y) == x % y

  if(TARGET_HEADING < 0){ // If target heading becomes negative
    TARGET_HEADING = TARGET_HEADING + 360; // Flip over 0-360 boundary
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void TurningAngle(double target_heading){ // Function to determine ANGLE_TURN from CURRENT_HEADING & TARGET_HEADING
  
  double angle_provisional = target_heading - CURRENT_HEADING; // Take the difference of target heading and current heading

  // Make changes to the angle so that it is always within -180 & 180
  if (angle_provisional <= 180 && angle_provisional> -180){ // If angle is within -180 & 180, leave it
      ANGLE_TURN = angle_provisional;
  }
  else if (angle_provisional > 180){ // If angle is greater than 180, subtract 360
      ANGLE_TURN = angle_provisional - 360;
  }
  else if (angle_provisional <= -180){ // If angle is less than -180, add 360
      ANGLE_TURN = angle_provisional + 360;
  } 
}
/*----------------------------------------------------------------------------------------------------------------------*/
void TurnLeft(short motor_speed){ // Takes in speed for ESC_MOTOR

  int temp_turn_angle = (int)(ANGLE_TURN); // Cast ANGLE_TURN to int
  temp_turn_angle = map(temp_turn_angle, -180, 180, 180, 0); // Map initial turn angle to servo values of 0-180

  if(temp_turn_angle < 100){
    temp_turn_angle = 100; // Set lower limit to 100
  }
  
  TURN_SERVO.write(temp_turn_angle); // Proportional turn between 0-90
  ESC_MOTOR.write(motor_speed);
}
/*----------------------------------------------------------------------------------------------------------------------*/
void TurnRight(short motor_speed){ // takes in speed for ESC_MOTOR

  int temp_turn_angle = (int)(ANGLE_TURN); // Cast ANGLE_TURN to int
  temp_turn_angle = map(temp_turn_angle, -180, 180, 180, 0); // Map initial turn angle to servo values of 0-180

  if(temp_turn_angle > 80){
    temp_turn_angle = 80; // set upper limit to 80
  }
  
  TURN_SERVO.write(temp_turn_angle); // Proportional turn between 0-90
  ESC_MOTOR.write(motor_speed);
}
/*----------------------------------------------------------------------------------------------------------------------*/
void TurnToHeading(short motor_speed){
  if (ANGLE_TURN > 0){ // If target heading is to the right of the current heading 
    // Perform reverse turn
    TurnRight(motor_speed); // TurnRight(speed)
    
    #if SERIAL_DEBUG == 1// If SERIAL_DEBUG condition is true
      SerialMonitor(); // Print rover data to the serial monitor
    #endif
    
    #if BLUETOOTH_DEBUG == 1 // If BLUETOOTH_DEBUG condition is true
      BluetoothTelemetry(); // Print to bluetooth device
    #endif
  }
  else if(ANGLE_TURN < 0){ // If target heading is to the left of current heading 
    // Perform reverse turn
    TurnLeft(motor_speed); // TurnLeft(speed)
    
    #if SERIAL_DEBUG == 1 // If SERIAL_DEBUG condition is true
      SerialMonitor(); // Print rover data to the serial monitor
    #endif
    
    #if BLUETOOTH_DEBUG == 1 // If BLUETOOTH_DEBUG condition is true
      BluetoothTelemetry(); // Print to bluetooth device
    #endif
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void HeadingHold(int motor_speed){
  // Drive straight using compass to course correct
  
  ESC_MOTOR.write(motor_speed); // Drive forward at speed set by motor_speed      
  
  int temp_turn_angle = (int)(ANGLE_TURN); // Cast ANGLE_TURN to int
  
  temp_turn_angle = map(temp_turn_angle, -180, 180, 0, 180); // Map initial turn angle to servo values of 0-180
  temp_turn_angle = constrain(temp_turn_angle, 0, 180);

  if(temp_turn_angle < 105 && temp_turn_angle > 90){
    temp_turn_angle = temp_turn_angle + (temp_turn_angle - 90)*2;
  }
  else if(temp_turn_angle > 67 && temp_turn_angle < 90){
    temp_turn_angle = temp_turn_angle - (90 - temp_turn_angle)*3;
  }

  TURN_SERVO.write(temp_turn_angle); // Tell the rover to turn according to temp_turn_value

  #if SERIAL_DEBUG == 1 // If SERIAL_DEBUG condition is true
    SerialMonitor(); // Print rover data to the serial monitor
  #endif
  
  #if BLUETOOTH_DEBUG == 1 // If BLUETOOTH_DEBUG condition is true
    BluetoothTelemetry(); // Print to bluetooth device
  #endif
  
}
/*----------------------------------------------------------------------------------------------------------------------*/
void UpdateTargetWaypoint(int waypoint){
  // Update the target coordinates and the type of waypoint
      
  TARGET_LAT = WAYPOINT_ARRAY[waypoint][0]; // Update TARGET_LAT
  TARGET_LONG = WAYPOINT_ARRAY[waypoint][1]; // Update TARGET_LONG
  CHECKPOINT = WAYPOINT_ARRAY[waypoint][2]; // Determine the type of waypoint set

}
/*----------------------------------------------------------------------------------------------------------------------*/
void GPSNavigation(){
  // Use GPS navigation functions to run through a course autonomously

  if(EMERGENCY_CHECK == 0 || NEW_STEP != CURRENT_STEP){ // Timer for "stuck" rover == EMERGENCY_TIMER
    // (0 == Refresh stuck-checking timer, 1 == Do not refresh)
    EMERGENCY_TIMER = millis(); // Start emergency timer
    CURRENT_STEP = NEW_STEP; // Update current step rover is in
  }
  if(abs(ANGLE_TURN) >= HEADING_ERROR && TURN_TO == 1){ // While current heading is +- HEADING_ERROR degrees off from target heading
    
    EMERGENCY_CHECK = 1; // Do not restart timer
    NEW_STEP = 1; // Increment which step the rover is on
    
    TurnToHeading(TURN_SPEED); // Turn to target heading
    if(millis() >= EMERGENCY_TIMER + (20*1000)){ // If stuck here for 20s, move forward
      ForwardDiagonalRight(); // End up diagonally back to the right
      EMERGENCY_CHECK = 0; // Restart timer
    }
    return; // Exit GPSNavigation()
  }
  else if(abs(ANGLE_TURN) < HEADING_ERROR && TURN_TO == 1){ // If target heading is reached, exit TurnToHeading()
    TURN_TO = 0; // Exit TurnToHeading()
    EMERGENCY_CHECK = 0; // Restart emergency timer
    
    ESC_MOTOR.write(90); // Stop drive motors before entering HeadingHold()
    //delay(1000); // Wait for 1 second before entering HeadingHold()
    return; // Exit GPSNavigation()
  }
  else if(DISTANCE >= 7.5 && TURN_TO == 0){ // Drive fast to target until within 7.5 meters
    HeadingHold(HOLD_SPEED + 3); // HeadingHold(int ESC_MOTOR speed)
    
    EMERGENCY_CHECK = 1; // Do not restart timer
    NEW_STEP = 2; // Increment which step the rover is on
    
    if(millis() >= EMERGENCY_TIMER + (60*1000)){ // If stuck here, back up
      ReverseDiagonalRight(); // End up diagonally back to the right
      EMERGENCY_CHECK = 0; // Restart timer
    }
    return; // Exit GPSNavigation()
  }
  else if(DISTANCE >= 1 && TURN_TO == 0){ // Drive slow to target until within 1 meter (Drives for ~6.5m)
    if(CHECKPOINT == FAST_TRANSITION){
      HeadingHold(HOLD_SPEED + 3); // HeadingHold(int ESC_MOTOR speed)
    }
    else if(CHECKPOINT == CONE_GRASS){ // Drive faster when expecting grass
      HeadingHold(HOLD_SPEED + 3); // HeadingHold(int ESC_MOTOR speed)
    }
    else{
      HeadingHold(HOLD_SPEED); // HeadingHold(int ESC_MOTOR speed)
    }

    EMERGENCY_CHECK = 1; // Do not restart timer
    NEW_STEP = 3; // Increment which step the rover is on

    if(millis() >= EMERGENCY_TIMER + (60*1000)){ // If stuck here, move forward
      ReverseDiagonalRight(); // Enter front collision response routine
      EMERGENCY_CHECK = 0; // Restart timer
    }
    
    return; // Exit GPSNavigation()
  }
  else if(DISTANCE < 1 && (CHECKPOINT == CONE || CHECKPOINT == CONE_GRASS)){ // At a cone, search for cone and attempt to hit within 30 seconds
    // CANNOT ACCESS ULTRASONICS ONCE HERE
    
    ESC_MOTOR.write(90); // Stop drive motors 

    STATE = SEARCH; // Reset Vision state

    CONE_TIMER = millis(); // Start a timer for intermediate cone check
    while(STATE != REST && millis() <= CONE_TIMER + (30*1000)){ // Run vision sensor until cone is hit or 60 seconds is up
      if(CHECKPOINT == CONE_GRASS){
        ALIGN_SPEED = 98;
        PURSUIT_SPEED = 98;
      }
      else{
        ALIGN_SPEED = 97;
        PURSUIT_SPEED = 97;
      }
      TargetCone(STATE); // Search for cone and drive into it
      if(digitalRead(REAR_LIMIT_SWITCH) == HIGH){
        ForwardDiagonalLeft(); // End up diagonally forward to the left
      }
    }
    
    // Get away from cone before continuing on
    TURN_SERVO.write(90); // Straighten wheels
    ESC_MOTOR.write(90); // Stop drive motors 
    delay(1000);
    ESC_MOTOR.write(80); // Reverse slowly
    delay(2000);
    ESC_MOTOR.write(90); // Stop drive motors
      
    WAYPOINT++; // Go to the next waypoint  
    UpdateTargetWaypoint(WAYPOINT); // Update target coordinates
    GPSUpdate(); // Update DISTANCE and TARGET_HEADING
    TURN_TO = 1; // Enter TurnToHeading() on next function call
    EMERGENCY_CHECK = 0; // Restart timer
    delay(1000); // Wait for 1 second before entering TurnToHeading()
    
    return; // Exit GPSNavigation()
  }
  else if(DISTANCE < 1 && CHECKPOINT == CONE_FINAL){ // At final cone, search endlessly
    // CANNOT ACCESS ULTRASONICS ONCE HERE
    
    ESC_MOTOR.write(90); // Stop drive motors 

    STATE = SEARCH; // Reset Vision state

    CONE_TIMER = millis(); // Start a timer for intermediate cone check
    while(STATE != REST && millis() <= CONE_TIMER + (30*1000)){ // Run vision sensor until cone is hit or 30 seconds is up
      TargetCone(STATE); // Search for cone and drive into it
      if(digitalRead(REAR_LIMIT_SWITCH) == HIGH){
        ForwardDiagonalLeft(); // End up diagonally forward to the left  
      }
    }

    if(STATE != REST){ // If final cone was not hit, backup and redo GPS navigation
      TURN_SERVO.write(90);
      ESC_MOTOR.write(80);
      CurrentCoordinates(); // Update CURRENT_LAT and CURRENT_LONG
      GPSUpdate(); // Update DISTANCE and TARGET_HEADING
      delay(3000);
      return;
    }
    else if(STATE == REST){ // If final cone was hit, set rover to idle state and record time to completion
      
      ESC_MOTOR.write(90); // Stop drive motors 

      COURSE_TIMER = millis() - STARTUP_TIMER; // Record time to complete the course
      int seconds = (int)(COURSE_TIMER) / 1000; // Convert COURSE_TIMER from milliseconds to seconds
      int minutes = seconds / 60; // Record number of minutes to complete the course
      seconds = seconds % 60; // Record the remainder in seconds
  
      // Print the time to complete the course
      Serial8.print("Course Completed. Total Trial Time = ");
      Serial8.print(minutes);
      Serial8.print(":");
      Serial8.println(seconds);
  
      FINISHED = true; // Send rover to idle state

      return; // Exit GPSNavigation()
    }   
  }
  else if(DISTANCE < 1 && CHECKPOINT == TRANSITION){ // If at a transition point
    ESC_MOTOR.write(90); // Stop rover momentarily
    WAYPOINT++; // Go to the next waypoint  
    UpdateTargetWaypoint(WAYPOINT); // Update target coordinates
    GPSUpdate(); // Update DISTANCE and TARGET_HEADING immediately to ensure proper transition
    
    TURN_TO = 1; // Enter TurnToHeading() on next function call
    NEW_STEP = 1; // Update the next step to go to
    
    //delay(250); // Hold rover for 0.25 seconds before moving on
    return; // Exit GPSNavigation() 
  }
  else if(DISTANCE < 1.5 && CHECKPOINT == FAST_TRANSITION){ // If at a fast transition point
    WAYPOINT++; // Go to the next waypoint  
    UpdateTargetWaypoint(WAYPOINT); // Update target coordinates
    GPSUpdate(); // Update DISTANCE and TARGET_HEADING immediately to ensure proper transition
    
    TURN_TO = 0; // Enter HeadingHold() on next function call
    NEW_STEP = 2; // Update the next step to go to
    
    return; // Exit GPSNavigation()
  }
}
