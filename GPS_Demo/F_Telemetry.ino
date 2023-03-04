void BluetoothTelemetry(){
// Keep reading from HC‐06 bluetooth module and send to Arduino Serial Monitor

    Serial8.print("Distance To Target: ");
    Serial8.println(DISTANCE);
    Serial8.print("Current Heading: ");
    Serial8.println(CURRENT_HEADING);
    Serial8.print("Target Heading: ");
    Serial8.println(TARGET_HEADING);
    Serial8.print("Current Lat: ");
    Serial8.println(CURRENT_LAT, 15);
    Serial8.print("Current Long: ");
    Serial8.println(CURRENT_LONG, 15);
    Serial8.print("Target Lat: ");
    Serial8.println(TARGET_LAT, 15);
    Serial8.print("Target Long: ");
    Serial8.println(TARGET_LONG, 15);
    Serial8.println();
    delay(500);
    digitalWrite(13, LOW); // turn on light to ensure board is working


  
  // Keep reading from Arduino Serial Monitor and send to HC‐06
  if (Serial.available()) // if bluetooth receiving device wants to send a message back to the Teensy board
  {
    Serial8.write(Serial.read());
  }
}
