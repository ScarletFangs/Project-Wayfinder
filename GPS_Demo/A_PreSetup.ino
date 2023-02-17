/*----------------------------------------------------------------------------------------------------------------------*/
// On-board LED to know when program has been successfully downloaded to board
void LEDSetup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on light to ensure board is working
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Setup compass object
void CompassSetup(){
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
}
/*----------------------------------------------------------------------------------------------------------------------*
