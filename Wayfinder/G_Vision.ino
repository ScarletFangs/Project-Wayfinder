/*
 * Using Pixy2 Vision Camera to search for, detect, track, and drive into an orange cone
 */
/*----------------------------------------------------------------------------------------------------------------------*/
// Update the state of the vision sensor
void TargetCone(enum visionState){
  
  pixy.ccc.getBlocks(); // Look for a cone

  switch(STATE)
  {
    case REST: // End vision sensor routine
      ESC_MOTOR.write(90); // Stop rover and enter idle state
      break;
    case SEARCH: // Cone not found, searching for cone
      SearchMode(); // Enter search mode
      break;
    case ALIGN: // Cone found, aligning rover with cone
      CONE_TIMER = millis(); // Start a timer for intermediate cone check
      AlignRover(1, 2); // Enter alignmet mode
      break;
    case PURSUIT: // Rover aligned with cone, attempting to collide with cone
      CONE_TIMER = millis(); // Start a timer for intermediate cone check
      PursueCone(); // Enter cone pursuit
      break;
    default:
      Serial.println("no state set / broken");
  }

}
/*----------------------------------------------------------------------------------------------------------------------*/
// Pan servo until a cone is detected. If no cone found, turn 120 degrees and search again
void SearchMode(){
  
  VISION_ANGLE = ROVER_CENTER;
  pixy.setServos(VISION_ANGLE, 25); // Center vision sensor

  while(VISION_ANGLE != 0 && STATE != ALIGN){ // Pan to the right
    VISION_ANGLE -= 10;
    pixy.setServos(VISION_ANGLE, 25);
    if(pixy.ccc.getBlocks() > 0){ // If cone detected, enter pursuit mode
      ESC_MOTOR.write(90); // Stop rover
      STATE = ALIGN;
      return;
    }
  }
  while(VISION_ANGLE != 1000 && STATE != ALIGN){ // Pan to the left
    VISION_ANGLE += 10;
    pixy.setServos(VISION_ANGLE, 25);
    if(pixy.ccc.getBlocks() > 0){ // If cone detected, enter pursuit mode
      ESC_MOTOR.write(90); // Stop rover
      STATE = ALIGN;
      return;
    }
  }

  // Reverse turn in circle while vision sensor pans left to right
  TURN_SERVO.write(0);
  ESC_MOTOR.write(83);
  
  if(pixy.ccc.getBlocks() > 0){ // If cone detected, enter pursuit mode
    ESC_MOTOR.write(90); // Stop rover
    STATE = ALIGN;
    return;
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Align rover, vision sensor, and cone
void AlignRover(int signature1, int signature2){
  
  if(pixy.ccc.blocks[0].m_signature == signature1) // If blocks detected are cones
  {  
    CENTER_OF_OBJECT = pixy.ccc.blocks[0].m_x; // Find the center of the object detected
    DISTANCE_TILL_CENTER = CENTER_OF_OBJECT - CAMERA_CENTER; // Distance between object center and camera center
    
    if(abs(DISTANCE_TILL_CENTER) <= VISION_ERROR_MARGIN){ 
      OBJECT_CENTER_TO_CAM = true; // Object is centered to the camera
    }
    else{
      OBJECT_CENTER_TO_CAM = false; // Object is not centered to the camera
    }

    CAMERA_ROVER_DIFF =  VISION_ANGLE - ROVER_CENTER; // Difference between vision sensor angle and rover center  
    if(abs(CAMERA_ROVER_DIFF) > ROVER_CENTER_ERROR && OBJECT_CENTER_TO_CAM){ // If rover is not yet aligned with cone
      // Proportional turn based on CAMERA_ROVER_DIFF
      STEERING_VALUE = map(CAMERA_ROVER_DIFF, -500, 500, 150, 30);  
      TURN_SERVO.write(STEERING_VALUE);
      ESC_MOTOR.write(ALIGN_SPEED);
    }
    else if(abs(CAMERA_ROVER_DIFF) <= ROVER_CENTER_ERROR && OBJECT_CENTER_TO_CAM){ // If rover is aligned with cone
      TURN_SERVO.write(90);
      ESC_MOTOR.write(90);
      STATE = PURSUIT; // Enter pursuit mode
    }
    else if(!OBJECT_CENTER_TO_CAM){ // If object is not centered on camera
      if(DISTANCE_TILL_CENTER < 0){ // If object is to the left
        if(VISION_ANGLE >= 1000){ // If cone is found at the edge of vision sensor range, turn anyways
          VISION_ANGLE = 1000; // Cap value of VISION_ANGLE at 1000
          TURN_SERVO.write(0);
          ESC_MOTOR.write(ALIGN_SPEED);
          pixy.setServos(VISION_ANGLE, 25);
        }
        else{ // Keep panning vision sensor to the left
          VISION_ANGLE += 5;
          pixy.setServos(VISION_ANGLE, 25);
        }
      }
      else if(DISTANCE_TILL_CENTER > 0){ // If object is to the right
        if(VISION_ANGLE <= 0){ // If cone is found at the edge of vision sensor range, turn anyways
          VISION_ANGLE = 0; // Cap value of VISION_ANGLE at 0
          TURN_SERVO.write(180);
          ESC_MOTOR.write(ALIGN_SPEED);
          pixy.setServos(VISION_ANGLE, 25);
        }
        else{ // Keep panning vision sensor to the right
          VISION_ANGLE -= 5;
          pixy.setServos(VISION_ANGLE, 25);
        }
      }
    }
  }
  else{ // If object lost, enter search mode again
    STATE = SEARCH;
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Vision-guided pursuit of cone
void PursueCone(){
  
  if(pixy.ccc.getBlocks() > 0) // If blocks detected are cones
  { 
    if(!LIMIT_COLLISION_FRONT){ // Run until cone is hit
      
      CENTER_OF_OBJECT = pixy.ccc.blocks[0].m_x; // Find the center of the object detected
      DISTANCE_TILL_CENTER = CENTER_OF_OBJECT - CAMERA_CENTER; // Distance between object center and camera center
      if(abs(DISTANCE_TILL_CENTER) <= VISION_ERROR_MARGIN + 150){ 
        OBJECT_CENTER_TO_CAM = true; // Object is centered to the camera, go forward
      }
      else{
        OBJECT_CENTER_TO_CAM = false; // Object is not centered to the camera
        STATE = SEARCH;
        return;
      }
      
      LimitSwitchCollision(); // Check for limit switch collision
  
      // Recalculate TURN_SERVO angle to hit cone
      if(DISTANCE_TILL_CENTER < 30){ // If cone is to the left
        STEERING_VALUE = 50;
      }
      else if(DISTANCE_TILL_CENTER > -30){ // If cone is to the right
        STEERING_VALUE = 130;
      }
      else{ // If cone is close to center
        STEERING_VALUE = 90;
      }
  
      TURN_SERVO.write(STEERING_VALUE);
      ESC_MOTOR.write(PURSUIT_SPEED);
    }
    else{
      STATE = REST; // End vision sensor loop
    }
  }
  else{
    STATE = SEARCH; // Reenter search mode
  }
}
