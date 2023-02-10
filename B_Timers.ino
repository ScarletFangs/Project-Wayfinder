/*
 * Timer functions for non-blocking interrupts of throttle, steering
 */

void ThrottleTimer(){
  // checking THROTTLE_PIN interrupt
  TH_CURRENT_TIME = micros();
  if (TH_CURRENT_TIME > TH_START_TIME){
    THROTTLE_PULSE = TH_CURRENT_TIME - TH_START_TIME;
    TH_START_TIME = TH_CURRENT_TIME;
  }
}

void SteeringTimer(){
  // checking STEERING_PIN interrupt
  ST_CURRENT_TIME = micros();
  if (ST_CURRENT_TIME > ST_START_TIME){
    STEERING_PULSE = ST_CURRENT_TIME - ST_START_TIME;
    ST_START_TIME = ST_CURRENT_TIME;
  }
}
