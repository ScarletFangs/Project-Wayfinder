const int PING_PIN = 22; // Trigger Pin of Ultrasonic Sensor
const int ECHO_PIN = 23; // Echo Pin of Ultrasonic Sensor
IntervalTimer MY_TIMER_1;
IntervalTimer MY_TIMER_2;

void UltrasonicSetup(){
  // setup ultrasonic pins
  pinMode(PING_PIN, OUTPUT); //enable PING_PINg as an OUTPUT
  pinMode(ECHO_PIN, INPUT); //enable ECHO_PIN as an INPUT
  MY_TIMER_1.begin(PingPH, 2); //set PING_PIN voltage to high 5V every 2 microsecodns
  MY_TIMER_2.begin(PingPL,10); //set PING_PIN volatage to low 0 volts (ground) 10 microsecodns
}


void setup() {
  Serial.begin(9600); // Starting Serial Terminal
  UltrasonicSetup(); // setup ultrasonics
}

void PingPH(){
  digitalWrite(PING_PIN, HIGH); // set ping pin to HIGH
  }

void PingPL(){
  digitalWrite(PING_PIN, LOW); // set ping pin to LOW
  }

float UltrasonicIn(const int x, const int y){
    // returns distance of detected object in inches
    float duration;
    
    digitalWrite(x, LOW); //set PING_PIN volatage to low 0 volts (ground)  
    duration = pulseIn(y, HIGH); //return duration of pulse and set to duration
    return duration / 74 / 2; //use to convert to inches
      
}

float UltrasonicCM(const int x, const int y){
    // returns distance of detected object in centimeters
    float duration;
    
    digitalWrite(x, LOW); //set PING_PIN volatage to low 0 volts (ground)
    duration = pulseIn(y, HIGH); //return duration of pulse and set to duration
    //Sound travels at 343 meters per second, which means it needs 29.155 microseconds per centimeter. So, we have to divide the duration by 29 and then by 2, because the sound has to travel the distance twice. It travels to the object and then back to the sensor.
    return duration / 29 / 2; //use to convert to centimeters
}

void loop() {
   float in, cm;
   in = UltrasonicIn(PING_PIN, ECHO_PIN );
   Serial.print(" ");
   Serial.print(in);
   Serial.print("in, ");
   cm = UltrasonicCM(PING_PIN, ECHO_PIN);
   Serial.print(cm);
   Serial.print(", cm");
   Serial.println();
  
}
