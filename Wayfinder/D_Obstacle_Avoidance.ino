/*
 * Checking functions and response routines for collision detection and obstacle avoidance
 */
/*----------------------------------------------------------------------------------------------------------------------*/
void CollisionDetection(){
  // Directionality checks for collision response routine selection
  bool collision_front = true, collision_rear = false, collision_left = true, collision_right = false;
  
  // If a collision is detected...
  if(LIMIT_COLLISION_FRONT || ULTRASONIC_COLLISION){ // ... from the front
    collision_front = true;
    collision_rear = false;
    COLLISION_TIMER = 0; // Reset collision response timer
    if(GLOBAL_MIN_SENSOR == LEFT_US){
      ReverseDiagonalRight(); // End up diagonally back to the right
      collision_right = true;
      collision_left = false;
    }
    else if(GLOBAL_MIN_SENSOR == RIGHT_US){
      ReverseDiagonalLeft(); // End up diagonally back to the left
      collision_right = false;
      collision_left = true;
    }
    else{ // If there is no preferable direction to reverese to
      if(ALTERNATE_FRONT){
        ReverseDiagonalRight(); // End up diagonally back to the right
        ALTERNATE_FRONT = false; // Flip direction of collision response
      }
      else{
        ReverseDiagonalLeft(); // End up diagonally back to the left
        ALTERNATE_FRONT = true; // Flip direction of collision response
      }
    }
    COLLISION_FINISHED = false;
    return;
  }
  else if(LIMIT_COLLISION_REAR){ // ... from the rear
    collision_front = false;
    collision_rear = true;
    COLLISION_TIMER = 0; // Reset collision response timer
    // Flip direction of response on each trigger of the rear limit switch
    if(ALTERNATE_BACK){
      ForwardDiagonalRight(); // End up diagonally forward to the right
      collision_right = true;
      collision_left = false;
      ALTERNATE_FRONT = false; // Flip direction of collision response
    }
    else{
      ForwardDiagonalLeft(); // End up diagonally forward to the left
      collision_right = false;
      collision_left = true;
      ALTERNATE_FRONT = true; // Flip direction of collision response
    }
    COLLISION_FINISHED = false;
    return;
  }
  else if(!COLLISION_FINISHED && collision_front){
    if(collision_right){
      ReverseDiagonalLeft(); // End up diagonally back to the left
    }
    else if(collision_left){
      ReverseDiagonalRight(); // End up diagonally back to the right
    }
  }
  else if(!COLLISION_FINISHED && collision_rear){
    if(collision_right){
      ForwardDiagonalLeft(); // End up diagonally forward to the Left
    }
    else if(collision_left){
      ForwardDiagonalRight(); // End up diagonally forward to the right
    }
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Limit switch collision detection
void LimitSwitchCollision(){
  // CHECKING LIMIT SWITCH INPUT
  
  if (digitalRead(LEFT_LIMIT_SWITCH) == HIGH || digitalRead(RIGHT_LIMIT_SWITCH) == HIGH){
    //Serial.println("Collision Detected: Limit Switch");
    LIMIT_COLLISION_FRONT = true; // Trigger CollisionResponse()
    LIMIT_COLLISION_REAR = false; // Do nothing otherwise
  }
  else if(digitalRead(REAR_LIMIT_SWITCH) == HIGH){
    LIMIT_COLLISION_REAR = true; // Trigger CollisionResponse()
    LIMIT_COLLISION_FRONT = false; // Do nothing otherwise
  }
  else{
    LIMIT_COLLISION_FRONT = false; // Do nothing otherwise
    LIMIT_COLLISION_REAR = false; // Do nothing otherwise
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Ultrasonic collision detection
void UltrasonicCollision(){
  long duration = 0;
  
  if(CURRENT_SENSOR == LEFT_US){
    digitalWrite(LEFT_TRIG, HIGH); // Send a ping out
    delayMicroseconds(10);
    digitalWrite(LEFT_TRIG, LOW);
    duration = pulseIn(LEFT_ECHO, HIGH); // Duration of ping in ms --> returns 38ms if nothing detected
    DISTANCE_ARRAY[CURRENT_SENSOR] = duration * 0.034 / 2; // Distance in cm
  }
  else if(CURRENT_SENSOR == CENTER_US){
    digitalWrite(CENTER_TRIG, HIGH); // Send a ping out
    delayMicroseconds(10);
    digitalWrite(CENTER_TRIG, LOW);
    duration = pulseIn(CENTER_ECHO, HIGH); // Duration of ping in ms --> returns 38ms if nothing detected
    DISTANCE_ARRAY[CURRENT_SENSOR] = duration * 0.034 / 2; // Distance in cm
  }
  else if(CURRENT_SENSOR == RIGHT_US){
    digitalWrite(CENTER_TRIG, HIGH); // Send a ping out
    delayMicroseconds(10);
    digitalWrite(CENTER_TRIG, LOW);
    duration = pulseIn(CENTER_ECHO, HIGH); // Duration of ping in ms --> returns 38ms if nothing detected
    DISTANCE_ARRAY[CURRENT_SENSOR] = duration * 0.034 / 2; // Distance in cm
  }

  uint16_t min_distance = 500; // Set minimum distance check
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    // If current US is reading a smaller distance, update GLOBAL_MIN_SENSOR
    if(DISTANCE_ARRAY[i] < min_distance){ 
      min_distance = DISTANCE_ARRAY[i];
      GLOBAL_MIN_SENSOR = i;
    }
  }
  
  // Trigger CollisionResponse() if an ultrasonic detects an object close by
  if (DISTANCE_ARRAY[0] < OBS_DISTANCE || DISTANCE_ARRAY[1] < OBS_DISTANCE || DISTANCE_ARRAY[2] < OBS_DISTANCE){ 
    ULTRASONIC_COLLISION = true; 
  }
  else{
    ULTRASONIC_COLLISION = false; // Do nothing otherwise
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
// Collision Response Routines
void ReverseDiagonalRight() 
{
  while (COLLISION_TIMER < DELAY500MS){
   ESC_MOTOR.write(90); // Stop rover
  }
      if (COLLISION_TIMER > DELAY1500MS){
      TURN_SERVO.write(180); // Turn wheels right
      ESC_MOTOR.write(80); // Reverse right
      TOTALDELAY += DELAY1500MS;
      }
      else if (COLLISION_TIMER > TOTALDELAY){
        TOTALDELAY += DELAY500MS;
        TURN_SERVO.write(90);
        }
      else if (COLLISION_TIMER > TOTALDELAY){
        TOTALDELAY += DELAY500MS; 
        TURN_SERVO.write(0);
        }
      else if (COLLISION_TIMER > TOTALDELAY){
        TOTALDELAY += DELAY1500MS;
        TURN_SERVO.write(90);
        }
      else if (COLLISION_TIMER > TOTALDELAY){
        TOTALDELAY += DELAY500MS;
        ESC_MOTOR.write(90);
        }
      else if (COLLISION_TIMER >= TOTALDELAY) 
      {
        COLLISION_FINISHED = true;
        TOTALDELAY = 0;
      }
}

void ReverseDiagonalLeft()
{
  while (COLLISION_TIMER < DELAY500MS){
    ESC_MOTOR.write(90); // stop rover
  }
    if (COLLISION_TIMER > DELAY500MS){
      TURN_SERVO.write(0); // Turn wheels left
      ESC_MOTOR.write(80); // Reverse left
      TOTALDELAY += DELAY500MS;
    }
    else if (COLLISION_TIMER > TOTALDELAY){
      TOTALDELAY += DELAY1500MS;
      TURN_SERVO.write(90);
    }
    else if (COLLISION_TIMER > TOTALDELAY){
      TURN_SERVO.write(180);
      TOTALDELAY += DELAY1500MS;
    }
    else if (COLLISION_TIMER > TOTALDELAY){
      TURN_SERVO.write(90);
      TOTALDELAY += DELAY500MS;
    }
    else if (COLLISION_TIMER > TOTALDELAY){
      ESC_MOTOR.write(90);
      TOTALDELAY += DELAY500MS;  
    }
    else if (COLLISION_TIMER >= TOTALDELAY)
    {
      COLLISION_FINISHED = true;
      TOTALDELAY = 0;
    }

}

void ForwardDiagonalRight()
{
  while (COLLISION_TIMER < DELAY500MS){
   ESC_MOTOR.write(90); // Stop rover 
  }
   if (COLLISION_TIMER > DELAY1500MS){
    TURN_SERVO.write(90);
    ESC_MOTOR.write(90);
    TOTALDELAY += DELAY1500MS;
   }
   else if (COLLISION_TIMER > TOTALDELAY){
    TURN_SERVO.write(180);
    ESC_MOTOR.write(100);
    TOTALDELAY += DELAY1500MS;
   }
   else if (COLLISION_TIMER > TOTALDELAY){
    TURN_SERVO.write(90);
    TOTALDELAY += DELAY500MS;
   }
   else if (COLLISION_TIMER > TOTALDELAY){
    TURN_SERVO.write(0);
    TOTALDELAY += DELAY1500MS;
   }
   else if (COLLISION_TIMER > TOTALDELAY){
    TURN_SERVO.write(90);
    TOTALDELAY += DELAY500MS;
   }
   else if (COLLISION_TIMER > TOTALDELAY){
    ESC_MOTOR.write(90);
    TOTALDELAY += DELAY500MS;
   }
   else if (COLLISION_TIMER >= TOTALDELAY){
    COLLISION_FINISHED = true;
    TOTALDELAY = 0;
   }
}

void ForwardDiagonalLeft()
{
  while (COLLISION_TIMER < DELAY500MS){
    ESC_MOTOR.write(90); 
  }
      if (COLLISION_TIMER > TOTALDELAY){
        TURN_SERVO.write(0);
        ESC_MOTOR.write(100);
        TOTALDELAY += DELAY1500MS;
      }
    else if (COLLISION_TIMER > TOTALDELAY){
      TURN_SERVO.write(90);
      TOTALDELAY += DELAY500MS;
    }
    else if (COLLISION_TIMER > TOTALDELAY){
      TURN_SERVO.write(180);
      TOTALDELAY += DELAY1500MS;
    }
    else if (COLLISION_TIMER > TOTALDELAY){
      TURN_SERVO.write(90);
      TOTALDELAY += DELAY500MS;
    }
    else if (COLLISION_TIMER > TOTALDELAY){
      ESC_MOTOR.write(90);
      TOTALDELAY += DELAY500MS;
    }
    else if (COLLISION_TIMER >= TOTALDELAY){
      COLLISION_FINISHED = true;
      TOTALDELAY  = 0;
    }
}
