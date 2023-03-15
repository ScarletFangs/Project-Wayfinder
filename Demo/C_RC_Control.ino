/*
 * Read and Drive functions for RC Control
 */
/*----------------------------------------------------------------------------------------------------------------------*/
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
}
/*----------------------------------------------------------------------------------------------------------------------*/
void RCDrive() {
  // pass values from RCRead() to throttle and steering servo
  RCRead();
  THROTTLE_VALUE = map(THROTTLE_PW, THROTTLE_CENTER - 300, THROTTLE_CENTER + 300, 0, 180); // map RC throttle values to 0-180
  ESC_MOTOR.write(THROTTLE_VALUE); // write mapped values to motor esc

  STEERING_VALUE = map(STEERING_PW, STEERING_CENTER - 300, STEERING_CENTER + 300, 0, 180); // map RC steering values to 0-180
  TURN_SERVO.write(STEERING_VALUE); // write mapped values to turning servo
}
/*----------------------------------------------------------------------------------------------------------------------*/
void DeadManSwitch() { // Auton|RC Control toggle function
  
  DEAD_MAN_VALUE = pulseIn(DEAD_MAN_PIN, HIGH); // read value from RC buttons between ~2000 & ~1000
  
  if(DEAD_MAN_VALUE < 1100 && DEAD_MAN_VALUE > 900) { // if bottom button is pressed go to AUTON control
    RC_CONTROL = false;
    AUTON_CONTROL = true;
  }
  else if(DEAD_MAN_VALUE < 2100 && DEAD_MAN_VALUE > 1900){ // if top button is pressed, use RC routine
    RC_CONTROL = true;
    AUTON_CONTROL = false;
  }
  else{
    RC_CONTROL = false;
    AUTON_CONTROL = false;
  }
}
