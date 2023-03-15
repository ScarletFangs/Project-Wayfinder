/*
 * Printing rover data through serial monitor and bluetooth telemetry
 */
/*----------------------------------------------------------------------------------------------------------------------*/ 
void BluetoothTelemetry(){
  // Keep reading from HC‐06 bluetooth module and send to Arduino Serial Monitor

  digitalWrite(13, HIGH); // turn on light
  
  Serial8.print("Distance To Target: ");
  Serial8.println(DISTANCE);
  
  Serial8.print("Current Lat: ");
  Serial8.println(CURRENT_LAT, 12);
  
  Serial8.print("Current Long: ");
  Serial8.println(CURRENT_LONG, 12);
  
  Serial8.print("Target Lat: ");
  Serial8.println(TARGET_LAT, 12);
  
  Serial8.print("Target Long: ");
  Serial8.println(TARGET_LONG, 12);
  
  Serial8.print("Current Heading: ");
  Serial8.println(CURRENT_HEADING);
  
  Serial8.print("Target Heading: ");
  Serial8.println(TARGET_HEADING);
  
  Serial8.print("Angle Difference: ");
  Serial8.println(ANGLE_TURN);
  Serial8.println();
  
  digitalWrite(13, LOW); // turn off light


  
  // Keep reading from Arduino Serial Monitor and send to HC‐06
  if (Serial.available()) // if bluetooth receiving device wants to send a message back to the Teensy board
  {
    Serial8.write(Serial.read());
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void SerialMonitor(){ 
  // Print rover positional data to the serial monitor
  
  digitalWrite(13, HIGH); // turn on light
  
  Serial.print("Distance To Target: ");
  Serial.println(DISTANCE);
  
  Serial.print("Current Latitude: ");
  Serial.println(CURRENT_LAT, 12);
  
  Serial.print("Current Longitude: ");
  Serial.println(CURRENT_LONG, 12);
  
  Serial.print("Target Lat: ");
  Serial.println(TARGET_LAT, 12);
  
  Serial.print("Target Long: ");
  Serial.println(TARGET_LONG, 12);
  
  Serial.print("Current Heading: ");
  Serial.println(CURRENT_HEADING);
  
  Serial.print("Target Heading: ");
  Serial.println(TARGET_HEADING);
  
  Serial.print("Angle Difference: ");
  Serial.println(ANGLE_TURN);
  Serial.println();
  
  digitalWrite(13, LOW); // turn off light
  
}
