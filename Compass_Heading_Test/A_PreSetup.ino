/*----------------------------------------------------------------------------------------------------------------------*/
// On-board LED to know when program has been successfully downloaded to board
void LEDSetup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on light to ensure board is working
}
/*----------------------------------------------------------------------------------------------------------------------*/
