/*
 * All pre-setup functions to be called in setup()
 */
/*----------------------------------------------------------------------------------------------------------------------*/
// Setup compass object and timer
void CompassSetup(){
  Wire.begin(); // Connect compass' SDA & SCL to SDA0 & SCL0 on board --> Pins 18 19
  COMPASS.init();
  COMPASS.enableDefault();
}
/*----------------------------------------------------------------------------------------------------------------------*/
void CalibrateCompass(){
  // Calibrate the compass
  
  COMPASS.read();
  
  running_min.x = min(running_min.x, COMPASS.m.x);
  running_min.y = min(running_min.y, COMPASS.m.y);
  running_min.z = min(running_min.z, COMPASS.m.z);

  running_max.x = max(running_max.x, COMPASS.m.x);
  running_max.y = max(running_max.y, COMPASS.m.y);
  running_max.z = max(running_max.z, COMPASS.m.z);

  // Print calibration values to serial monitor
  snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
    running_min.x, running_min.y, running_min.z,
    running_max.x, running_max.y, running_max.z);
  Serial.println(report);
  Serial8.println(report);

  // Update calibration values as compass is calibrated
  X_MIN = running_min.x;
  Y_MIN = running_min.y;
  Z_MIN = running_min.z;
  
  X_MAX = running_max.x;
  Y_MAX = running_max.y;
  Z_MAX = running_max.z;
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Setup serial port for GPS (Serial3 = Pins 14 & 15) and timer
void GPSSetup(){
  // Start the hardware serial communication with the BN220 GPS module
  Serial3.begin(9600);
  Serial3.setTX(14); // TX3 pin
  Serial3.setRX(15); // RX3 pin
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit Switches
void LimitSwitchSetup(){
  pinMode(LEFT_LIMIT_SWITCH, INPUT_PULLDOWN); // Set the pin to accept inputs
  pinMode(RIGHT_LIMIT_SWITCH, INPUT_PULLDOWN); // Set the pin to accept inputs
  pinMode(REAR_LIMIT_SWITCH, INPUT_PULLDOWN); // Set the pin to accept inputs
  // Note:
  // Limit switches need to be set to "INPUT_PULLDOWN" this is due to
  // when their circuit is disconnected they have noise that will mess with the actual values
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonics
void UltrasonicSetup(){
  pinMode(LEFT_TRIG, OUTPUT); // Sets the trigPin as an Output
  pinMode(LEFT_ECHO, INPUT); // Sets the echoPin as an Input
  
  pinMode(CENTER_TRIG, OUTPUT); // Sets the trigPin as an Output
  pinMode(CENTER_ECHO, INPUT); // Sets the echoPin as an Input
  
  pinMode(RIGHT_TRIG, OUTPUT); // Sets the trigPin as an Output
  pinMode(RIGHT_ECHO, INPUT); // Sets the echoPin as an Input
}
/*----------------------------------------------------------------------------------------------------------------------*/
// RC controller
void RCSetup(){
  pinMode(THROTTLE_PIN, INPUT_PULLUP); // Set the pin to accept inputs
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN),ThrottleTimer,CHANGE); // Attach interrupt to THROTTLE_PIN from changing states
  pinMode(STEERING_PIN, INPUT_PULLUP); // Set the pin to accept inputs
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN),SteeringTimer,CHANGE); // Attach interrupt to STEERING_PIN from changing states
  pinMode(DEAD_MAN_PIN, INPUT); // Set the pin to accept inputs
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Bluetooth telemetry setup
void BluetoothSetup(){
  Serial8.begin(9600);
  Serial8.setTX(34); // TX8 pin
  Serial8.setRX(35); // RX8 pin
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Vision sensor setup
void VisionSetup(){
  pixy.init();
  pixy.setLamp(1,1);
  STATE = SEARCH; // Start vision sensor in search mode
  VISION_ANGLE = ROVER_CENTER; // Start VISION_ANGLE at center of rover
  pixy.setServos(VISION_ANGLE, 25); // Set vision sensor pan and tilt positions
}
