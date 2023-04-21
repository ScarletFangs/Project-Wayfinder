/*
 * GPS Navigation test file designed to navigate the rover to a waypoint using the GPS module & compass as the main 
 * means of navigation. Pixy2 vision sensor turns on when the rover thinks it is near a cone and attempts to run into 
 * the cone. Collision detection functions are used in conjuction with RCDrive() and GPSNavigation(). Also includes 
 * telemetry through the means of a bluetooth module. Intended for use with a companion drone for autonomous recovery
 * of a black box item.
 * 04/21/2023
 */ 
#include <LSM303.h> // Include compass library 
#include <Wire.h> // Include I2C library for communication with compass
#include <Servo.h> // Include servo/motor library for driving esc and turn servo
#include <elapsedMillis.h> // Include delay timer library for preemptive multitasking
#include <NewPing.h> // Include NewPing library for ultrasonics
#include <Pixy2SPI_SS.h> // Pixy2 camera library
#include <Pixy2CCC.h> // Pixy2 camera library
#include "A_Global_Variables.h" // Global variable header file 

void setup(){

  Serial.begin(115200);
  
  ESC_MOTOR.attach(9); // Set ESC_MOTOR to pin 9
  TURN_SERVO.attach(8); // Set TURN_SERVO to pin 8

  // Send ESC and Turn servo 90 signal to ensure proper signal centering
  ESC_MOTOR.write(90);
  TURN_SERVO.write(90);

  CompassSetup(); // Setup compass

  GPSSetup(); // Setup GPS serial communication

  BluetoothSetup(); // Setup bluetooth telemetry

  LimitSwitchSetup(); // Setup limit switches

  UltrasonicSetup(); // Setup ultrasonic sensors

  VisionSetup(); // Setup vision sensor
    
  Serial.println("Warming Up, Please Turn On ESC.");
  Serial8.println("Warming Up, Please Turn On ESC.");

  delay(2000); // Send ESC and Servo 0 signal until program starts
  
/*----------------------------------------------------------------------------------------------------------------------*/
  #if DEBUG_MODE == 0
    Serial.println("Begin Compass Calibration. Activate Left Limit Switch to Exit Calibration.");
    Serial8.println("Begin Compass Calibration. Activate Left Limit Switch to Exit Calibration.");
  
    delay(2000);
    
    while(true){
      CalibrateCompass(); // Calibrate the compass until limit switch is triggered
      
      // If limit switch is pressed, stop compass calibration
      if(digitalRead(LEFT_LIMIT_SWITCH) == HIGH){
        // Establish compass bearing using calibration values from CalibrateCompass()
        COMPASS.m_min = (LSM303::vector<int16_t>){X_MIN, Y_MIN, Z_MIN};
        COMPASS.m_max = (LSM303::vector<int16_t>){X_MAX, Y_MAX, Z_MAX};
        break; // Exit calibration
      }
    }
  
/*----------------------------------------------------------------------------------------------------------------------*/
    Serial.println("Waiting for GPS Lock...");
    Serial8.println("Waiting for GPS Lock...");
  
    // Wait until GPS lock is obtained
    while(CURRENT_LAT == 0 && digitalRead(RIGHT_LIMIT_SWITCH) == LOW){
      CurrentCoordinates(); // Get initial GPS coordinates of the rover
      CurrentHeading(); // Get initial heading of the rover
      UpdateTargetWaypoint(WAYPOINT); // Update target coordinates
      GPSUpdate(); // Calculate initial DISTANCE and TARGET_HEADING
      TurningAngle(TARGET_HEADING); // Calculate initial turning angle ANGLE_TURN
    }
  
    delay(1000);
  
    Serial.println("GPS Lock Confirmed.");
    Serial8.println("GPS Lock Confirmed.");
  
    delay(1000);
  #endif
/*----------------------------------------------------------------------------------------------------------------------*/
    #if RC_TOGGLE == 1 // If using remote control

    RCSetup(); // Setup RC control
    RCRead(); // Read current signal from remote control
    THROTTLE_CENTER = THROTTLE_PW; // Set throttle zero point
    STEERING_CENTER = STEERING_PW; // Set steering zero point
    
    Serial.println("Please Select Control Mode. Top Button for RC Control, Bottom Button for Autonomous Control.");
    Serial8.println("Please Select Control Mode.");
  
    // Wait for user to select control mode
    while(!RC_CONTROL && !AUTON_CONTROL){ 
      DeadManSwitch(); 
      if(RC_CONTROL){ // If RC Control was detected
        Serial.println("RC Control Selected.");
        delay(1000);
      }
      else if(AUTON_CONTROL){ // If Autonomous Control was detected
        Serial.println("Autonomous Control Selected.");
        delay(1000);
      }
    }
    
    if(RC_CONTROL == true){
      Serial.println("Exiting Setup. Press Rear Limit Switch to Begin in RC Control.");
      delay(1000);
  
      Serial8.println("Exiting Setup. Press Rear Limit Switch to Begin in RC Control.");
      delay(1000);
    }
    else if(AUTON_CONTROL == true){
      Serial.println("Exiting Setup. Press Rear Limit Switch to Begin Autonomous Run.");
      delay(1000);
  
      Serial8.println("Exiting Setup. Press Rear Limit Switch to Begin Autonomous Run.");
      delay(1000);
    }
/*----------------------------------------------------------------------------------------------------------------------*/ 
    #elif RC_TOGGLE == 0 // If not using remote control
    Serial.println("Exiting Setup. Press Rear Limit Switch to Begin Autonomous Run.");
    delay(1000);
  
    Serial8.println("Exiting Setup. Press Rear Limit Switch to Begin Autonomous Run.");
    delay(1000);
    #endif
}
/*----------------------------------------------------------------------------------------------------------------------*/ 
void loop(){
  SensorTimers(); // Update sensors on regular timing intervals
  
  CollisionDetection(); // If a collision was detected, enter a collision response routine
  
  GPSNavigation(); // Main GPS navigational routine
}
/*----------------------------------------------------------------------------------------------------------------------*/ 
void SensorTimers(){
  // Update limit switch states every 5ms
  if(LS_TIMER > LS_DELAY){
    LS_TIMER = 0; // Reset LS_TIMER
    LimitSwitchCollision(); // Update Limit switch state
  }
  
  // Update Ultrasonic states every 40ms
  if(US_TIMER > US_DELAY){
    US_TIMER = 0; // Reset US_TIMER
    UltrasonicCollision(); // Update collision state for one ultrasonic
    
    if(CURRENT_SENSOR == 2){
      CURRENT_SENSOR = 0; // Restart ultrasonic cycle
    }
    else{
      CURRENT_SENSOR++; // Go to next ultrasonic sensor
    }
  }

  // Update compass bearing every 20ms
  if(COM_TIMER > COM_DELAY){
    COM_TIMER = 0; // Reset COM_TIMER
    CurrentHeading(); // Update LS_FRONT & LS_REAR
  }

  // Update GPS coordinates every 1s
  if(GPS_TIMER > GPS_DELAY){
    GPS_TIMER = 0; // Reset GPS_TIMER
    GPSUpdate(); // Update CURRENT_LAT & CURRENT_LONG
  }
}
