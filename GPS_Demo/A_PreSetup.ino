/*----------------------------------------------------------------------------------------------------------------------*/
// On-board LED to know when program has been successfully downloaded to board
void LEDSetup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on light to ensure board is working
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Setup compass object
void CompassSetup(){
  Wire.begin(); // Connect compass' SDA & SCL to SDA2 & SCL2 on board --> Pins 25 24
  // Wire1.begin(); // Connect compass' SDA & SCL to SDA1 & SCL1 on board
  // Serial.println(compass.init());
  COMPASS.init();
  COMPASS.enableDefault();
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  COMPASS.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  COMPASS.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Setup serial port for GPS
void GPSSetup(){
  // Start the hardware serial communication with the BN220 GPS module
  Serial3.begin(9600);
  Serial3.setTX(14); // TX3 pin
  Serial3.setRX(15); // RX3 pin
}
/*----------------------------------------------------------------------------------------------------------------------*/
// RC controller
void RCSetup(){
  pinMode(THROTTLE_PIN, INPUT_PULLUP); // set the pin to accept inputs
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN),ThrottleTimer,CHANGE); // attach interrupt to THROTTLE_PIN from changing states
  pinMode(STEERING_PIN, INPUT_PULLUP); // set the pin to accept inputs
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN),SteeringTimer,CHANGE); // attach interrupt to STEERING_PIN from changing states
  pinMode(DEAD_MAN_PIN, INPUT); // set the pin to accept inputs
}
