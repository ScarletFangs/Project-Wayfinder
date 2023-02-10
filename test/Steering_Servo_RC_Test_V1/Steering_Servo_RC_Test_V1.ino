#include <Servo.h>
#define TRANSMITTER_PIN 4
#define SERVO_PIN 6

unsigned long duration;
 
Servo TILT_WHEELS;

void setup() {
  Serial.begin(9600);
  TILT_WHEELS.attach(SERVO_PIN); // The pin from the servo that connects to the arduino board.
  pinMode(TRANSMITTER_PIN, INPUT);
}

void loop() 
{
  duration = pulseIn(TRANSMITTER_PIN, HIGH);
  delay(15);
  if (duration > 950 && duration < 1890)
  {
  duration = map(duration, 950, 1890, 0, 180);
  // The reason the mapping is required is due to the controller(?) giving values that are 
  // much more out of range than the servo. To fix this, the ratio of the values is mapped
  // to the ratio of the servo. 
  Serial.println(duration);
  TILT_WHEELS.write(duration);
 }
}
