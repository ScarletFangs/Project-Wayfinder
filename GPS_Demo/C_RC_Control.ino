/*
 * Read and Drive functions for RC Control
 */

void RCRead() {
  // return PWM values for RC throttle and steering
  if(THROTTLE_PULSE < 2200){
    THROTTLE_PW = THROTTLE_PULSE;
  }
//  Serial.print("Throttle: ");
//  Serial.println(THROTTLE_PULSE); // print throttle value

  if(STEERING_PULSE < 2200){
    STEERING_PW = STEERING_PULSE;
  }
//  Serial.print("Steering: ");
//  Serial.println(STEERING_PW); // print steering value
}
void RCDrive() {
  // pass values from RCRead() to throttle and steering servo
  RCRead();
  THROTTLE_VALUE = map(THROTTLE_PW, 1150, 1960, 0, 180); // map RC throttle values to 0-180
  //Serial.println(THROTTLE_VALUE);
  ESC_MOTOR.write(THROTTLE_VALUE); // write mapped values to motor esc

  STEERING_VALUE = map(STEERING_PW, 1075, 1750, 0, 180); // map RC steering values to 0-180
  //Serial.println(STEERING_VALUE);
  TURN_SERVO.write(STEERING_VALUE); // write mapped values to turning servo
}

void DeadManSwitch() { // Auton|RC Control toggle function
  
  DEAD_MAN_VALUE = pulseIn(DEAD_MAN_PIN, HIGH); // read value from RC buttons between ~2000 & ~1000
  
  if(DEAD_MAN_VALUE < 2100 && DEAD_MAN_VALUE > 1900) { // if top button is pressed use RC control
    RCDrive(); // return PWM values for RC throttle and steering  
    //LimitSwitchCollision(); // if limit switch is triggered, interrupt RC control and do collision response
    //UltrasonicCollision(); // if sonar is triggered, interrupt RC control and do collision response
    Serial.println("RC Control");
  }
  else{ // if bottom button is pressed, use autonomous routine
    TurnToHeading(); // Use GPS navigation
    //LimitSwitchCollision(); // if limit switch is triggered, interrupt navigation and do collision response
    //UltrasonicCollision(); // if sonar is triggered, interrupt navigation and do collision response
    Serial.println("Autonomous");
  }
}
