/*
 * Printing rover data through serial monitor and bluetooth telemetry
 */
/*----------------------------------------------------------------------------------------------------------------------*/ 
void BluetoothTelemetry(){
  // Keep reading from HC‐06 bluetooth module and send to Arduino Serial Monitor
  
  Serial8.print("Distance: ");
  Serial8.println(DISTANCE);
  
  Serial8.print("C Lat: ");
  Serial8.println(CURRENT_LAT, 7);
  
  Serial8.print("C Long: ");
  Serial8.println(CURRENT_LONG, 7);
  
  Serial8.print("T Lat: ");
  Serial8.println(TARGET_LAT, 7);
  
  Serial8.print("T Long: ");
  Serial8.println(TARGET_LONG, 7);
  
  Serial8.print("C Heading: ");
  Serial8.println(CURRENT_HEADING);
  
  Serial8.print("T Heading: ");
  Serial8.println(TARGET_HEADING);
  
  Serial8.print("ANGLE_TURN: ");
  Serial8.println(ANGLE_TURN);
  Serial8.println();
  
  // Keep reading from Arduino Serial Monitor and send to HC‐06
  if (Serial.available()) // if bluetooth receiving device wants to send a message back to the Teensy board
  {
    Serial8.write(Serial.read());
  }
}
/*----------------------------------------------------------------------------------------------------------------------*/
void SerialMonitor(){ 
  // Print rover positional data to the serial monitor
  
  Serial.print("Distance: ");
  Serial.println(DISTANCE);
  
  Serial.print("C Lat: ");
  Serial.println(CURRENT_LAT, 7);
  
  Serial.print("C Long: ");
  Serial.println(CURRENT_LONG, 7);
  
  Serial.print("T Lat: ");
  Serial.println(TARGET_LAT, 7);
  
  Serial.print("T Long: ");
  Serial.println(TARGET_LONG, 7);
  
  Serial.print("C Heading: ");
  Serial.println(CURRENT_HEADING);
  
  Serial.print("T Heading: ");
  Serial.println(TARGET_HEADING);
  
  Serial.print("ANGLE_TURN: ");
  Serial.println(ANGLE_TURN);
  Serial.println();
  
}
