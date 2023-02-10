/*
 * Setup functions for limit switches, LED, RC control
 */

void LimitSwitchSetup(){
  pinMode(LEFT_LIMIT_SWITCH, INPUT_PULLDOWN); // set the pin to accept inputs
  pinMode(RIGHT_LIMIT_SWITCH, INPUT_PULLDOWN); // set the pin to accept inputs
  // Note:
  // Limit switches need to be set to "INPUT_PULLDOWN" this is due to
  // when their circuit is disconnected they have noise that will mess with the actual values
}

void LEDSetup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on light to ensure board is working
}

void RCSetup(){
  pinMode(THROTTLE_PIN, INPUT_PULLUP); // set the pin to accept inputs
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN),ThrottleTimer,CHANGE); // attach interrupt to THROTTLE_PIN from changing states
  pinMode(STEERING_PIN, INPUT_PULLUP); // set the pin to accept inputs
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN),SteeringTimer,CHANGE); // attach interrupt to STEERING_PIN from changing states
  pinMode(DEAD_MAN_PIN, INPUT); // set the pin to accept inputs
}
