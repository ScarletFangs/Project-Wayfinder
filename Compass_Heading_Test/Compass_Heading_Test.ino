/*
 * Compass Test File
 */
#include <LSM303.h>
#include <Wire.h>

LSM303 compass; // initialize an LSM303 compass object

void setup() {
  Serial.begin(9600);

  Wire.begin(); // Connect compass' SDA & SCL to SDA0 & SCL0 on board
  // Wire1.begin(); // Connect compass' SDA & SCL to SDA1 & SCL1 on board
  Serial.println(compass.init());
  compass.enableDefault();

  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  
  LEDSetup(); // turn on on-board LED to ensure program is running

}

void loop() {
  compass.read();
  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  
    compass.heading((LSM303::vector<int>){0, 0, 1});
  
  to use the +Z axis as a reference.
  */
  float heading = compass.heading(); // return
  
  Serial.println(heading);
  delay(100);

}
