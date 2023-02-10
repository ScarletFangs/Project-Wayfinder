/*
 * Read and Drive functions for RC Control
 */

void RCRead() {
  // return PWM values for RC throttle and steering
  if(THROTTLE_PULSE < 2200){
    THROTTLE_PW = THROTTLE_PULSE;
  }
  //Serial.print("Throttle: ");
  //Serial.println(THROTTLE_PW); // print throttle value

  if(STEERING_PULSE < 2200){
    STEERING_PW = STEERING_PULSE;
  }
  //Serial.print("Steering: ");
  //Serial.println(STEERING_PW); // print steering value
}
void RCDrive() {
  // pass values from RCRead() to throttle and steering servo
  RCRead();
  THROTTLE_VALUE = map(THROTTLE_PW, 1020, 1860, 0, 180); // map RC throttle values to 0-180
  // Serial.println(THROTTLE_VALUE);
  ESC_MOTOR.write(THROTTLE_VALUE); // write mapped values to motor esc

  STEERING_VALUE = map(STEERING_PW, 959, 1890, 0, 180); // map RC steering values to 0-180
  // Serial.println(STEERING_VALUE);
  TURN_SERVO.write(STEERING_VALUE); // write mapped values to turning servo
}
