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
#include <Servo.h>

void setup() {
  Serial.begin(9600);
  
}



void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(DISTANCE);
}
