/*----------------------------------------------------------------------------------------------------------------------*/
// On-board LED to know when program has been successfully downloaded to board
void LEDSetup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on light to ensure board is working
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Setup compass object and timer
void CompassSetup(){
  //COMPASS_TIMER.start(3000); // Set delay interval for COMPASS_TIMER
  Wire.begin(); // Connect compass' SDA & SCL to SDA0 & SCL0 on board --> Pins 18 19
  COMPASS.init();
  COMPASS.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
//  COMPASS.m_min = (LSM303::vector<int16_t>){-285, -450, -611};
//  COMPASS.m_max = (LSM303::vector<int16_t>){+295, +220, -379};
//  COMPASS.m_min = (LSM303::vector<int16_t>){-411, -494, -613};
//  COMPASS.m_max = (LSM303::vector<int16_t>){+347, +192, -344};
  COMPASS.m_min = (LSM303::vector<int16_t>){-169, -302, -546};
  COMPASS.m_max = (LSM303::vector<int16_t>){+372, +200, -483};
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Setup serial port for GPS (Serial3 = Pins 14 & 15) and timer
void GPSSetup(){
  // Start the hardware serial communication with the BN220 GPS module
  Serial3.begin(9600);
  Serial3.setTX(14); // TX3 pin
  Serial3.setRX(15); // RX3 pin
  GPS_TIMER.start(2000); // Set delay interval for GPS_TIMER
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit Switches
void LimitSwitchSetup(){
  pinMode(LEFT_LIMIT_SWITCH, INPUT_PULLDOWN); // set the pin to accept inputs
  pinMode(RIGHT_LIMIT_SWITCH, INPUT_PULLDOWN); // set the pin to accept inputs
  // Note:
  // Limit switches need to be set to "INPUT_PULLDOWN" this is due to
  // when their circuit is disconnected they have noise that will mess with the actual values
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonics
void UltrasonicSetup(){
  // setup ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT); //enable TRIG_PINg as an OUTPUT
  pinMode(ECHO_PIN, INPUT); //enable ECHO_PIN as an INPUT
  MY_TIMER_1.begin(PingPH, 10); //set TRIG_PIN voltage to high 5V every 2 microsecodns
  MY_TIMER_2.begin(PingPL, 11); //set TRIG_PIN voltage to low 0 volts (ground) 10 microsecodns
}

float UltrasonicIn(const int x, const int y){
    // returns distance of detected object in inches
    float duration;
    
    digitalWrite(x, LOW); //set TRIG_PIN volatage to low 0 volts (ground)  
    duration = pulseIn(y, HIGH); //return duration of pulse and set to duration
    return duration / 74 / 2; //use to convert to inches
}

float UltrasonicCM(const int x, const int y){
    // returns distance of detected object in centimeters
    float duration;
    
    digitalWrite(x, LOW); // set TRIG_PIN volatage to low 0 volts (ground)
    duration = pulseIn(y, HIGH); // return duration of pulse and set to duration
    // Sound travels at 343 meters per second, which means it needs 29.155 microseconds per centimeter... 
    // So, we have to divide the duration by 29 and then by 2, because the sound has to travel the distance twice... 
    // It travels to the object and then back to the sensor.
    return duration / 29 / 2; // use to convert to centimeters
}

void PingPH(){
  digitalWrite(TRIG_PIN, HIGH); // set ping pin to HIGH
  }

void PingPL(){
  digitalWrite(TRIG_PIN, LOW); // set ping pin to LOW
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
/*----------------------------------------------------------------------------------------------------------------------*/
// Bluetooth telemetry setup
void BluetoothSetup(){
  Serial8.begin(9600);
  Serial8.setTX(34); // TX8 pin
  Serial8.setRX(35); // RX8 pin
}
