/*
 * Converting GPS Latitude and Longitude to distance to drive and target heading
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
    bool south = false;
    bool west = false;

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
    double temp_long; // declare temp_lat variable for assignment later
    
    if (west){ // If coordinate is West, make value negative
      temp_long = -1 * (atof(long_string.c_str()) / 100); // if West, make negative
    }
    else{ // If coordinate is East, keep positive
      temp_long = atof(long_string.c_str()) / 100; // if East, make positive
    }
    
    if(temp_long  != 0){
      CURRENT_LONG = temp_long; // update CURRENT_LONG only if given good data
    }
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void CurrentHeading(){
  // Grab current heading of the rover from compass and update global variable CURRENT_HEADING
  COMPASS.read();
  CURRENT_HEADING = COMPASS.heading() + 13; // account for difference between Magnetic North and True North
  if(CURRENT_HEADING >= 360){ // If above correction creates an angle > 360
    CURRENT_HEADING = fmod(CURRENT_HEADING, 360); // flip over 360/0 boundary
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/ 
void GPSUpdate(double initial_lat, double initial_long, double final_lat, double final_long){
  /*
   * Uses haversine formula to calculate distance and target heading between two points when given coordinates of 
   * current position and target position.
   */
  const float R = 6371e3; // meters --> radius of the earth (assumed spherical geometry)
  // Convert GPS coordinates from degrees to radians
  initial_lat = initial_lat * M_PI / 180;
  initial_long = initial_long * M_PI / 180;
  final_lat = final_lat * M_PI / 180;
  final_long = final_long * M_PI / 180;
  
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
  TARGET_HEADING = atan2(y, x);

  TARGET_HEADING = TARGET_HEADING * 180 / M_PI; // Convert target heading from radians to degrees (given in -180-180 range)
  TARGET_HEADING = fmod(TARGET_HEADING + 360.0, 360); // Normalize to 0-360 compass bearing
  // fmod = floating point modulus % --> fmod(x, y) == x % y
}
/*----------------------------------------------------------------------------------------------------------------------*/
void TurningAngle(double target_heading, double current_heading){ // function to determine Turning Angle from CURRENT_HEADING & TARGET_HEADING
  
  double angle_provisional = target_heading - current_heading; // Take the difference of target heading and current heading

  // Make changes to the angle so that it is always within -180 & 180
  if (angle_provisional <= 180 && angle_provisional> -180){ // if angle is within -180 & 180, leave it
      ANGLE_TURN = angle_provisional;
  }
  else if (angle_provisional > 180){ // if angle is greater than 180, subtract 360
      ANGLE_TURN = angle_provisional - 360;
  }
  else if (angle_provisional <= -180){ // if angle is less than -180, add 360
      ANGLE_TURN = angle_provisional + 360;
  }
  
}
/*----------------------------------------------------------------------------------------------------------------------*/
void TurnLeft(short motor_speed){ // takes in speed for ESC_MOTOR

  int temp_turn_angle = (int)round(abs(CURRENT_HEADING - TARGET_HEADING)); // check difference between CURRENT_HEADING & TARGET_HEADING and cast to int

  if(temp_turn_angle > 90){ // if temp_turn_angle is too large for servo
    temp_turn_angle = 90; // set upper limit to 90
  }
  
  if (motor_speed <= 180 && motor_speed >= 90){ // Perform forward turn
    TURN_SERVO.write(90 - temp_turn_angle); // proportional turn between 0-90
    ESC_MOTOR.write(motor_speed); // forward slow
  }
  else { // Perform reverse turn
    TURN_SERVO.write(90 + temp_turn_angle); // proportional turn between 90-180
    ESC_MOTOR.write(motor_speed); // double call due to ESC settings
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void TurnRight(short motor_speed){ // takes in speed for ESC_MOTOR

  int temp_turn_angle = (int)round(abs(CURRENT_HEADING - TARGET_HEADING)); // check difference between CURRENT_HEADING & TARGET_HEADING and cast to int

  if(temp_turn_angle > 90){ // if temp_turn_angle is too large for servo
    temp_turn_angle = 90; // set upper limit to 90
  }
  
  if (motor_speed <= 180 && motor_speed >= 90){ // Perform forward turn 
    TURN_SERVO.write(90 + temp_turn_angle); // proportional turn between 90-180
    ESC_MOTOR.write(motor_speed); // forward slow
  }
  else { // Perform reverse turn
    TURN_SERVO.write(90 - temp_turn_angle); // proportional turn between 0-90
    ESC_MOTOR.write(motor_speed); // double call due to ESC settings
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void TurnToHeading(short motor_speed){
  // Turn rover towards target heading with a set speed and precision of turn

  if (ANGLE_TURN > 0){ // if target heading is to the right of the current heading 
    // Perform reverse turn
    TurnRight(motor_speed); // TurnRight(speed)
    
    #ifdef SERIAL_DEBUG // if SERIAL_DEBUG condition is true
      Serial.println("TurnToHeading()"); // Print current function rover is in
      SerialMonitor(); // print rover data to the serial monitor
    #endif
    
    #ifdef BLUETOOTH_DEBUG // if BLUETOOTH_DEBUG condition is true
      Serial8.println("TurnToHeading()"); // Print current function rover is in
      BluetoothTelemetry(); // print to bluetooth device
    #endif
  }
  else if(ANGLE_TURN < 0){ // if target heading is to the left of current heading 
    // Perform reverse turn
    TurnLeft(motor_speed); // TurnLeft(speed)
    
    #ifdef SERIAL_DEBUG // if SERIAL_DEBUG condition is true
      Serial.println("TurnToHeading()"); // Print current function rover is in
      SerialMonitor(); // print rover data to the serial monitor
    #endif
    
    #ifdef BLUETOOTH_DEBUG // if BLUETOOTH_DEBUG condition is true
      Serial8.println("TurnToHeading()"); // Print current function rover is in
      BluetoothTelemetry(); // print to bluetooth device
    #endif
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void HeadingHold(int motor_speed){
  // Drive straight using compass to course correct
  
  ESC_MOTOR.write(motor_speed); // drive forward at speed set by motor_speed      
  
  int temp_turn_value = 90; // create turn value that is casted to an integer for better reading by ESC
  double angle_provisional = TARGET_HEADING - CURRENT_HEADING; // Take the difference of target heading and current heading

  // Make changes to the angle so that it is always within -180 & 180
  if(angle_provisional <= 180 && angle_provisional> -180){ // if angle is within -180 & 180, leave it
      temp_turn_value = (int)angle_provisional;
  }
  else if(angle_provisional > 180){ // if angle is greater than 180, subtract 360
      temp_turn_value = (int)(angle_provisional - 360);
  }
  else if(angle_provisional <= -180){ // if angle is less than -180, add 360
      temp_turn_value = (int)(angle_provisional + 360);
  }

  temp_turn_value = map(temp_turn_value, -180, 180, 30, 150); // Map initial turn angle to a smaller range

  if(temp_turn_value > 135){
    temp_turn_value = 135; // Cap at 135;
  }
  else if(temp_turn_value < 45){
    temp_turn_value = 45; // Cap at 45
  }

  TURN_SERVO.write(temp_turn_value); // Tell the rover to turn according to temp_turn_value

  #ifdef SERIAL_DEBUG // if SERIAL_DEBUG condition is true
    ANGLE_TURN = angle_provisional; // Update ANGLE_TURN for debugging purposes
    Serial.println("HeadingHold()"); // Print current function rover is in
    SerialMonitor(); // Print rover data to the serial monitor
  #endif
  
  #ifdef BLUETOOTH_DEBUG // if BLUETOOTH_DEBUG condition is true
    ANGLE_TURN = angle_provisional; // Update ANGLE_TURN for debugging purposes
    Serial8.println("HeadingHold()"); // Print current function rover is in
    BluetoothTelemetry(); // Print to bluetooth device
  #endif
  
}
/*----------------------------------------------------------------------------------------------------------------------*/
void UpdateTargetWaypoint(int waypoint){
  // Update the target coordinates and the type of waypoint
      
  TARGET_LAT = WAYPOINT_ARRAY[waypoint][0]; // Update TARGET_LAT
  TARGET_LONG = WAYPOINT_ARRAY[waypoint][1]; // Update TARGET_LONG
  CHECKPOINT = WAYPOINT_ARRAY[waypoint][2]; // Check to see if target coordinates is a cone or not

}
/*----------------------------------------------------------------------------------------------------------------------*/
void GPSNavigation(){
  // Use GPS navigation functions to run through a course autonomously
  
  if(abs(ANGLE_TURN) > HEADING_ERROR){ // While current heading is +- 5 degrees off target heading
    TurnToHeading(80); // Turn to target heading
  }
  
  ESC_MOTOR.write(90); // stop drive motors before entering HeadingHold()
  delay(1500); // wait for 1.5 seconds before entering HeadingHold()
  
  if(DISTANCE >= 5){ // drive fast to target until within 5 meters
    HeadingHold(110); // HeadingHold(int ESC_MOTOR speed)
  }
  else if(DISTANCE >= 2.5){ // drive to target until within 2.5 meters
    HeadingHold(100); // HeadingHold(int ESC_MOTOR speed)
  }
  else if(DISTANCE >= 1){ // drive to target until within 1 meter
    HeadingHold(100); // HeadingHold(int ESC_MOTOR speed)
  }
  
  if(DISTANCE < 1){ // If within 1 meter of the target
    if(CHECKPOINT == 1){ // If this was a cone location, activate vision sensor
      Serial.println("Searching For Cone");
      Serial8.println("Searching For Cone");
    }
    WAYPOINT++; // Go to next waypoint  
    UpdateTargetWaypoint(WAYPOINT); // Update target coordinates
  }
}
