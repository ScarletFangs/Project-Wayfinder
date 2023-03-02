/*
 * Converting GPS Latitude and Longitude to distance to drive and target heading
 */

void CurrentCoordinates(){
  // Parse current latitude and longitude from GPS serial messages
  // Check if there is data available on the GPS module
  if (Serial3.available()>0) {
    // Read the data from the GPS module and save as a string
    String input_string = Serial3.readStringUntil('\n');
        
    input_string.remove(12,1);
    int index1 = input_string.indexOf(",N"); // Find the index of "N"
    int index2 = input_string.indexOf(",W"); // Find the index of "W"
    
    // Extract the latitude value and convert it to a float
    String lat_string = input_string.substring(index1-10, index1);
    double temp_lat; // declare temp_lat variable for assignment later
    if (input_string.substring(index1-9) == "-") // checks if the index before the start of LATITUDE begins with a negative symbol then multiplies the value by -1
    {
      temp_lat = -1 * atof(lat_string.c_str()) / 100; // if South, make negative
    }
    else
    {
      temp_lat = atof(lat_string.c_str()) / 100; // if North, make positive
    }
    if(abs(temp_lat) > 1.0){
      CURRENT_LAT = temp_lat; // update CURRENT_LAT only if given good data
    }
    
    // Extract the longitude value and convert it to a float
    String long_string = input_string.substring(index2-11, index2);
    double temp_long; // declare temp_lat variable for assignment later
    if (input_string.substring(index2-12) == "-") // checks if the index before the start of LATITUDE begins with a negative symbol then multiplies the value by -1
    {
      temp_long = -1 * atof(long_string.c_str()) / 100; // if West, make negative
    }
    else
    {
      temp_long = atof(long_string.c_str()) / 100; // if East, make positive
    }
    if(abs(temp_long) > 1.0){
      CURRENT_LAT = temp_long; // update CURRENT_LONG only if given good data
    }
  }
}

void CurrentHeading(){
  // Grab current heading of the rover from compass and update global variable CURRENT_HEADING
  COMPASS.read();
  CURRENT_HEADING = COMPASS.heading();
}
 
void GPSUpdate(float initial_lat, float initial_long, float final_lat, float final_long){
  /*
   * Uses haversine formula to calculate distance and target heading between two points when given coordinates of 
   * current position and target position.
   */
  // CONST: Do not allow this variable to be changed by anything
  const float R = 6371e3; // meters --> radius of the earth (assumed spherical geometry)
  // convert GPS coordinates from degrees to radians
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
  TARGET_HEADING = atan2((cos(initial_lat)*sin(final_lat)) - (sin(initial_lat)*cos(final_lat)*cos(long_diff)), 
                   sin(long_diff)*cos(final_lat));

  TARGET_HEADING = TARGET_HEADING * 180 / M_PI; // Convert target heading from radians to degrees (given in -180-180 range)
  TARGET_HEADING = fmod(TARGET_HEADING + 360.0, 360); // Normalize to 0-360 compass bearing
  // fmod = floating point modulus % --> fmod(x, y) == x % y
 }

void TurningAngle(float target_heading, float current_heading){ // function to determine Turning Angle from CURRENT_HEADING & TARGET_HEADING
  
  float angle_provisional = target_heading - current_heading; // Take the difference of target heading and current heading

  if (angle_provisional <= 180 && angle_provisional>-180){ 
      ANGLE_TURN = angle_provisional;
  }
  else if (angle_provisional>180){
      ANGLE_TURN = angle_provisional-360;
  }
  else if (angle_provisional<=-180){
      ANGLE_TURN = angle_provisional+360;
  }
  
}

/* MOTOR SPEEDS AND DIRECTIONS
* 0 <= speed <= 90 == reverse direction for motors
* 90 <= speed <= 180 == forward direction for motors
* 90 == stop motors
* 0 == full speed in reverse
* 180 == full speed forwards
*/

void TurnLeft(int motor_speed){ // takes in speed for ESC_MOTOR

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
    ESC_MOTOR.write(90); // reverse slow
    ESC_MOTOR.write(motor_speed); // double call due to ESC settings
  }
}

void TurnRight(int motor_speed){ // takes in speed for ESC_MOTOR

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
    ESC_MOTOR.write(90); // reverse slow
    ESC_MOTOR.write(motor_speed); // double call due to ESC settings
  }
}

void StopRover(){
   ESC_MOTOR.write(90); // stop drive motors
}

void TurnToHeading(int motor_speed, int error_margin){
  // Turn rover towards target heading with a set speed and precision of turn
  
  CurrentCoordinates(); // get current GPS coordinates of the rover
  CurrentHeading(); // update current heading of the rover
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // calculate DISTANCE and TARGET_HEADING
  TurningAngle(TARGET_HEADING, CURRENT_HEADING); // Calculate ANGLE_TURN by using current target heading and current heading
  
  if (ANGLE_TURN > 0){ // if target heading is to the right of the current heading
   while(abs(CURRENT_HEADING-TARGET_HEADING) >= error_margin){ // turn right while turn angle difference is greater than error_margin
      CurrentHeading(); // update current heading of the rover
      BluetoothTelemetry(); // print to bluetooth device --> 50ms BLOCKING DELAY
      Serial.print("Loop R-Current Latitude: ");
      Serial.println(CURRENT_LAT);
      Serial.println("TurnToHeading()");
      Serial.print("Loop R-Current Heading: ");
      Serial.println(CURRENT_HEADING);
      Serial.print("Loop R-Target Heading: ");
      Serial.println(TARGET_HEADING);
      Serial.print("Loop R-Angle Difference: ");
      Serial.println(abs(CURRENT_HEADING-TARGET_HEADING));
      Serial.println();
      TurnRight(motor_speed); // TurnRight(speed, duration for SetVelocity())
   }
  }else{ // if target heading is to the left of current heading
    while(abs(CURRENT_HEADING-TARGET_HEADING) >= error_margin){ // turn left while angle difference is greater than error_margin
      CurrentHeading(); // update current heading of the rover
      BluetoothTelemetry(); // print to bluetooth device --> 50ms BLOCKING DELAY
      Serial.print("Loop L-Current Latitude: ");
      Serial.println(CURRENT_LAT);
      Serial.println("TurnToHeading()");
      Serial.print("Loop L-Current Heading: ");
      Serial.println(CURRENT_HEADING);
      Serial.print("Loop L-Target Heading: ");
      Serial.println(TARGET_HEADING);
      Serial.print("Loop L-Angle Difference: ");
      Serial.println(abs(CURRENT_HEADING-TARGET_HEADING));
      Serial.println();
      TurnLeft(motor_speed); // TurnLeft(speed, duration for SetVelocity())
    } 
  }
}

void HeadingHold(int motor_speed){
  // Drive straight using compass to course correct
  
  CurrentCoordinates(); // get current rover GPS coordinates
  CurrentHeading(); // get current compass bearing of the rover
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // update DISTANCE and TARGET_HEADING
  
  ESC_MOTOR.write(motor_speed); // drive forward at speed set by motor_speed      
  CurrentHeading(); // update current heading of the rover
  CurrentCoordinates(); // update current coordinates of the rover
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // update DISTANCE and TARGET_HEADING

  // CURRENT = 10, TARGET = 170, CURRENT-TARGET= -160, +90 = -70
  // CURRENT = 359,  TARGET = 10, CURRENT-TARGET = 349, +90 = 439
  int temp_turn_value = (int)( 90 - (CURRENT_HEADING - TARGET_HEADING)); // create turn value that is casted to an integer for better reading by ESC
  if(temp_turn_value < 0){ // if temp_turn_value is less than 0
    temp_turn_value = abs(temp_turn_value); // if temp_turn_value is negative, change to positive
  }
  else if(temp_turn_value > 180){ // if temp_turn_value is greater than 180
    temp_turn_value = temp_turn_value % 180; // cap temp_turn_value at 180
  }
  BluetoothTelemetry(); // print to bluetooth device --> 50ms BLOCKING DELAY
  Serial.print("Loop S-Current Latitude: ");
  Serial.println(CURRENT_LAT);
  Serial.println("HeadingHold()");
  Serial.print("Loop S-Current Heading: ");
  Serial.println(CURRENT_HEADING);
  Serial.print("Loop S-Target Heading: ");
  Serial.println(TARGET_HEADING);
  Serial.print("Loop S-Angle Difference: ");
  Serial.println(abs(CURRENT_HEADING-TARGET_HEADING));
  Serial.print("Loop S-Turn Servo Angle: ");
  Serial.println(temp_turn_value);
  Serial.println();
  TURN_SERVO.write(temp_turn_value); // turn back to target heading proportional to the error
}
