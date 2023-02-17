/*
 * Converting GPS Latitude and Longitude to distance to drive and target heading
 */

void ParseGPS(){
  // Parse GPS serial messages into usable data
  
}

void CurrentCoordinates(){
  // Grab current latitude and longitude from GPS
  
}

void CurrentHeading(){
  // Grab current heading of the rover from compass
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
  TARGET_HEADING = atan2(sin(LONG_DIFF)*cos(final_lat), 
  (cos(initial_lat)*sin(final_lat)) - (sin(initial_lat)*cos(final_lat)*cos(LONG_DIFF)));

  TARGET_HEADING = TARGET_HEADING * 180 / M_PI; // Convert target heading from radians to degrees
 }

 void TurnToHeading(){
  // Uses GPSUpdate() to turn rover towards a desired heading
  
  CurrentCoordinates(); // update current latitude and longitude of the rover

  CurrentHeading(); // update current heading of the rover
  
  GPSUpdate(CURRENT_LAT, CURRENT_LONG, TARGET_LAT, TARGET_LONG); // Calculate the target distance and heading from current coordinates and target coordinates

 }
