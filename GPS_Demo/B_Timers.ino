/*
 * Timer functions for non-blocking interrupts of throttle, steering
 */
/*----------------------------------------------------------------------------------------------------------------------*/
void ThrottleTimer(){
  // Checking THROTTLE_PIN interrupt
  TH_CURRENT_TIME = micros();
  if (TH_CURRENT_TIME > TH_START_TIME){
    THROTTLE_PULSE = TH_CURRENT_TIME - TH_START_TIME;
    TH_START_TIME = TH_CURRENT_TIME;
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void SteeringTimer(){
  // Checking STEERING_PIN interrupt
  ST_CURRENT_TIME = micros();
  if (ST_CURRENT_TIME > ST_START_TIME){
    STEERING_PULSE = ST_CURRENT_TIME - ST_START_TIME;
    ST_START_TIME = ST_CURRENT_TIME;
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void SensorTimerSetup(){
  TIMER_50MS.start(50); // Start 50ms timer
  TIMER_1S.start(1000); // Start 1s timer
}
/*----------------------------------------------------------------------------------------------------------------------*/
void SensorTimers(){
  // Every 50ms, check compass
  if(TIMER_50MS.justFinished()){
    CurrentHeading(); // Update current heading of the rover
    LimitSwitchCollision(); // Check if a limit switch has hit something
    TIMER_50MS.repeat(); // Restart TIMER_50MS
  }

  // Every second, check current GPS coordinates
  if(TIMER_1S.justFinished()){
    CurrentCoordinates(); // Update current coordinates of the rover
    GPSUpdate();
    TIMER_1S.repeat(); // Restart TIMER_1S
  }
}
