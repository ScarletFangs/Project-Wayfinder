/*
 * GPS Navigation Test File
 */
 #include <LSM303.h>
 #include <Wire.h>

LSM303 COMPASS; // initialize an LSM303 compass object

volatile float CURRENT_LAT = 0; // initialize current latitude reading
volatile float CURRENT_LONG = 0; // initialize current longitude reading
volatile float CURRRENT_HEADING = 0; // initialize current heading
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
 
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
