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
void TurnLeft(int motor_speed, long timer){ // takes in speed for direction and timer for duration used in SetVelocity()

  int temp_turn_angle = (int)round(abs(CURRENT_HEADING - TARGET_HEADING)); // check difference between CURRENT_HEADING & TARGET_HEADING

  if(temp_turn_angle > 90){ // if temp_turn_angle is too large for servo
    temp_turn_angle = 90; // set upper limit to 90
  }
  
  if (motor_speed <= 180 && motor_speed >= 90){ //if forward turn wheels 
    //Serial.print("Turn Servo Angle: ");
    //Serial.println(90 - temp_turn_angle);
    TURN_SERVO.write(90 - temp_turn_angle); // proportional turn between 0-90
    //SetVelocity(speed_, timer); // set velocity using ramping function
    ESC_MOTOR.write(motor_speed); // forward slow
  }else {
    //Serial.print("Turn Servo Angle: ");
    //Serial.println(90 + temp_turn_angle);
    TURN_SERVO.write(90 + temp_turn_angle); // proportional turn between 90-180
    //SetVelocity(speed_, timer); // set velocity using ramping function
    ESC_MOTOR.write(motor_speed); // reverse slow
    ESC_MOTOR.write(motor_speed);
  }
}

void TurnRight(int motor_speed, long timer){ // takes in speed for direction and timer for duration used in SetVelocity()

  int temp_turn_angle = (int)round(abs(CURRENT_HEADING - TARGET_HEADING)); // check difference between CURRENT_HEADING & TARGET_HEADING and cast to int

  if(temp_turn_angle > 90){ // if temp_turn_angle is too large for servo
    temp_turn_angle = 90; // set upper limit to 90
  }
  
  if (motor_speed <= 180 && motor_speed >= 90){ //if forward turn wheels 
    //Serial.print("Turn Servo Angle: ");
    //Serial.println(90 + temp_turn_angle);
    TURN_SERVO.write(90 + temp_turn_angle); // proportional turn between 90-180
    //SetVelocity(speed_, timer); // set velocity using ramping function
    ESC_MOTOR.write(motor_speed); // forward slow
  }else {
    //Serial.print("Turn Servo Angle: ");
    //Serial.println(90 - temp_turn_angle);
    TURN_SERVO.write(90 - temp_turn_angle); // proportional turn between 0-90
    //SetVelocity(110, timer); // set velocity using ramping function
    ESC_MOTOR.write(motor_speed); // reverse slow
    ESC_MOTOR.write(motor_speed);
  }
}

void StopRover(){
   ESC_MOTOR.write(90); // stop
}

void TurnToHeading(int motor_speed, int error_margin){
  
  CurrentCoordinates(); // get current GPS coordinates of the rover
  CurrentHeading(); // update current heading of the rover
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // calculate DISTANCE and TARGET_HEADING
  TurningAngle(TARGET_HEADING, CURRENT_HEADING); // Calculate ANGLE_TURN by using current target heading and current heading
  
  if (ANGLE_TURN > 0){ // if target heading is to the right of the current heading
   while(abs(CURRENT_HEADING-TARGET_HEADING) >= error_margin){ // turn right while turn angle difference is greater than ERROR_MARGIN
      CurrentHeading(); // update current heading of the rover
      Serial.println("TurnToHeading()");
      Serial.print("Loop R-Current Heading: ");
      Serial.println(CURRENT_HEADING);
      Serial.print("Loop R-Target Heading: ");
      Serial.println(TARGET_HEADING, 12);
      Serial.print("Loop R-Angle Difference: ");
      Serial.println(abs(CURRENT_HEADING-TARGET_HEADING));
      Serial.println();
      TurnRight(motor_speed, 100); // TurnRight(speed, duration for SetVelocity())
   }
  }else{ // if target heading is to the left of current heading
    while(abs(CURRENT_HEADING-TARGET_HEADING) >= error_margin){ // turn left while angle difference is greater than ERROR_MARGIN
        CurrentHeading(); // update current heading of the rover
        Serial.println("TurnToHeading()");
        Serial.print("Loop L-Current Heading: ");
        Serial.println(CURRENT_HEADING);
        Serial.print("Loop L-Target Heading: ");
        Serial.println(TARGET_HEADING, 12);
        Serial.print("Loop L-Angle Difference: ");
        Serial.println(abs(CURRENT_HEADING-TARGET_HEADING));
        Serial.println();
        TurnLeft(motor_speed, 100); // TurnLeft(speed, duration for SetVelocity())
    } 
  }
}

void HeadingHold(int motor_speed, int distance_threshold){
  // Drive straight using compass to course correct
  CurrentCoordinates(); // get current rover GPS coordinates
  CurrentHeading(); // get current compass bearing of the rover
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // update DISTANCE and TARGET_HEADING
  ESC_MOTOR.write(motor_speed); // drive forward at speed set by motor_speed
  
    if(DISTANCE >= distance_threshold){ 
      // Drive toward target coordinates until minimum distance threshold is reached
      
      CurrentHeading(); // update current heading of the rover
      CurrentCoordinates(); // update current coordinates of the rover
      GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // update DISTANCE and TARGET_HEADING
      
      int temp_turn_value = (int)( 90 - (CURRENT_HEADING - TARGET_HEADING)); // create turn value that is casted to an integer for better reading by ESC
      if(temp_turn_value < 0){ // if temp_turn_value is less than 0
        temp_turn_value = abs(temp_turn_value); // if temp_turn_value is negative, change to positive
       //CURRENT = 10, TARGET = 170, CURRENT-TARGET= -160, +90 = -7
       //CURRENT = 359,  target 10, CURRENT-TARGET = 349, +90 = 439
      }else if(temp_turn_value > 180){ // if temp_turn_value is greater than 180
        temp_turn_value = temp_turn_value % 180; // cap temp_turn_value at 180
      }
      //BluetoothTelemetry(); // print to bluetooth device --> 50ms blocking delay
      Serial.println("HeadingHold()");
      Serial.print("Loop S-Current Heading: ");
      Serial.println(CURRENT_HEADING);
      Serial.print("Loop S-Target Heading: ");
      Serial.println(TARGET_HEADING, 12);
      Serial.print("Loop S-Angle Difference: ");
      Serial.println(abs(CURRENT_HEADING-TARGET_HEADING));
      Serial.print("Loop S-Turn Servo Angle: ");
      Serial.println(temp_turn_value);
      Serial.println();
      TURN_SERVO.write(temp_turn_value); // turn back to target heading proportional to the error
    }// END WHILE LOOP
}

void GPSNavigation(int error_boundary){
  // Combine navigational functions into one function
  CurrentCoordinates(); // get current rover GPS coordinates
  CurrentHeading(); // get current compass bearing of the rover
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // update DISTANCE and TARGET_HEADING
  if(abs(CURRENT_HEADING - TARGET_HEADING) < error_boundary){ // if within heading_error degrees of target heading, use HeadingHold() function
    HeadingHold(120, error_boundary); // drive straight towards target 
  }
  else if(abs(CURRENT_HEADING - TARGET_HEADING) >= error_boundary){ // if more than turn_error degrees of difference from target heading, use TurnToHeading() function
    TurnToHeading(70, error_boundary); // turn rover towards target
  }
  
}
