/*
 * Converting GPS Latitude and Longitude to distance to drive
 */
volatile float LAT_DIFF = 0; // initialize latitude difference variable
volatile float LONG_DIFF = 0; // initialize longitude difference variable
volatile float A = 0; // initialize argument of the square root
volatile float C = 0; // initialize arctangent term
volatile float DISTANCE = 0; // initialize final distance variable
volatile float TARGET_HEADING = 0; // initialize target heading
/*
 * VOLATILE FLOAT --> changing number with a lot of decimal points
 * CONST DOUBLE --> unchanging number of double variable type
 */

/*
 * Converting GPS coordinates to distance to drive and heading
 */
 
void GPSUpdate(float initial_lat, float initial_long, float final_lat, float final_long){
  /*
   * Uses haversine formula to calculate distance between two points when given latitudes and longitudes
   */
  // CONST: Do not allow this variable to be changed by anything
  const double R = 6371e3; // meters --> radius of the earth (assumed spherical geometry)
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

  TARGET_HEADING = TARGET_HEADING * 180 / M_PI;
 }

 // On-board LED to know when program has been successfully downloaded to board
void LEDSetup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on light to ensure board is working
}

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(DISTANCE);
}
