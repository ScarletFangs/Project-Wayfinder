/*
 * Setup functions for limit switches, LED, RC control
 */
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
  pinMode(PING_PIN, OUTPUT); //enable PING_PINg as an OUTPUT
  pinMode(ECHO_PIN, INPUT); //enable ECHO_PIN as an INPUT
  MY_TIMER_1.begin(PingPH, 10); //set PING_PIN voltage to high 5V every 2 microsecodns
  MY_TIMER_2.begin(PingPL, 11); //set PING_PIN volatage to low 0 volts (ground) 10 microsecodns
}

float UltrasonicIn(const int x, const int y){
    // returns distance of detected object in inches
    float duration;
    
    digitalWrite(x, LOW); //set PING_PIN volatage to low 0 volts (ground)  
    duration = pulseIn(y, HIGH); //return duration of pulse and set to duration
    return duration / 74 / 2; //use to convert to inches
}

float UltrasonicCM(const int x, const int y){
    // returns distance of detected object in centimeters
    float duration;
    
    digitalWrite(x, LOW); // set PING_PIN volatage to low 0 volts (ground)
    duration = pulseIn(y, HIGH); // return duration of pulse and set to duration
    // Sound travels at 343 meters per second, which means it needs 29.155 microseconds per centimeter... 
    // So, we have to divide the duration by 29 and then by 2, because the sound has to travel the distance twice... 
    // It travels to the object and then back to the sensor.
    return duration / 29 / 2; // use to convert to centimeters
}

void PingPH(){
  digitalWrite(PING_PIN, HIGH); // set ping pin to HIGH
  }

void PingPL(){
  digitalWrite(PING_PIN, LOW); // set ping pin to LOW
  }
/*----------------------------------------------------------------------------------------------------------------------*/
// On-board LED to know when program has been successfully downloaded to board
void LEDSetup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on light to ensure board is working
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
