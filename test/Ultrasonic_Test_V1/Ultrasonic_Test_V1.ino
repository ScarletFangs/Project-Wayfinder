<<<<<<< Updated upstream
const int PING_PIN = 22; // Trigger Pin of Ultrasonic Sensor
const int ECHO_PIN = 23; // Echo Pin of Ultrasonic Sensor
IntervalTimer MY_TIMER_1; // Declare
IntervalTimer MY_TIMER_2; // Declare
=======
#include <millisDelay.h> // include delay timer library (from SafeString library)

// Ultrasonic variables
#define LEFT_TRIG 16    // Trigger Pin of Left Ultrasonic Sensor to pin 16
#define LEFT_ECHO 17    // Echo Pin of Left Ultrasonic Sensor to pin 17
#define CENTER_TRIG 13  // Trigger Pin of Center Ultrasonic Sensor to pin 13 --> CURRENTLY CONFLICTING WITH PROGRAM_LED
#define CENTER_ECHO 20  // Echo Pin of Center Ultrasonic Sensor to pin 20
#define RIGHT_TRIG 41   // Trigger Pin of Right Ultrasonic Sensor to pin 41
#define RIGHT_ECHO 40   // Echo Pin of Right Ultrasonic Sensor to pin 40
#define REAR_TRIG 39    // Trigger Pin of Rear Ultrasonic Sensor to pin 39
#define REAR_ECHO 38    // Echo Pin of Rear Ultrasonic Sensor to pin 38

#define SONAR_NUM 3 // Number of ultrasonic sensors
#define MAX_DISTANCE 500 // Maximum distance (in cm) to ping
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo)

unsigned long PING_TIMER_ARRAY[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor
unsigned int DISTANCE_ARRAY[SONAR_NUM];         // Where the ping distances are stored
uint8_t CURRENT_SENSOR = 0;          // Keeps track of which sensor is active
unsigned int GLOBAL_MIN_SENSOR = 0;         // Sensor with the smallest distance 0 = Right Sensor; 1 = Middle Sensor; 2 = Left Sensor 
unsigned int OBS_DISTANCE = 50; // Minimum distance for ultra sonics to a avoid obstacle

millisDelay 110MS_TIMER; // Initialize millisDelay timer object

NewPing SONAR_TIME[SONAR_NUM] = // Sonar timing object array using NewPing library 
   {NewPing(LEFT_TRIG, LEFT_ECHO, MAX_DISTANCE),      // Left ultrasonic setup
    NewPing(CENTER_TRIG, CENTER_ECHO, MAX_DISTANCE),  // Right ultrasonic setup
    NewPing(RIGHT_TRIG, RIGHT_ECHO, MAX_DISTANCE)};   // Right ultrasonic setup
>>>>>>> Stashed changes

void UltrasonicSetup(){
  // Setup timers
  PING_TIMER_ARRAY[0] = millis() + 75; // First ping starts at 75ms, gives time for the Arduino to chill before starting
  for (uint8_t i = 1; i < SONAR_NUM; i++){ // Set the starting time for each sensor.
  PING_TIMER_ARRAY[i] = PING_TIMER_ARRAY[i - 1] + PING_INTERVAL;
  }
}

void UltrasonicSweeping(){ // Check front 3 ultrasonics in a sweeping fashion
   for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors
    if (millis() >= PING_TIMER_ARRAY[i]) { // Is it this sensor's time to ping?
      
      PING_TIMER_ARRAY[i] += PING_INTERVAL * SONAR_NUM; // Set next time this sensor will be pinged
      
      if (i == 0 && CURRENT_SENSOR == SONAR_NUM - 1){ 
        FindMinSensor(); // Sensor ping cycle complete, do something with the results
      }
      
      SONAR_TIME[CURRENT_SENSOR].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance)
      CURRENT_SENSOR = i;                             // Update sensor being accessed
      DISTANCE_ARRAY[CURRENT_SENSOR] = MAX_DISTANCE;  // Make distance zero in case there's no ping echo for this sensor
      SONAR_TIME[CURRENT_SENSOR].ping_timer(EchoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo)
    }
  }
}

void EchoCheck() { 
  // If ping received, update DISTANCE_ARRAY with the new distance
  
  if (SONAR_TIME[CURRENT_SENSOR].check_timer()){
    DISTANCE_ARRAY[CURRENT_SENSOR] = SONAR_TIME[CURRENT_SENSOR].ping_result / US_ROUNDTRIP_CM;
    Serial.println(DISTANCE_ARRAY[CURRENT_SENSOR]); 
  }
}

void setup() {
  Serial.begin(9600); // Starting Serial Terminal
  UltrasonicSetup(); // setup ultrasonics
  110MS_TIMER.begin(110); // Start 110MS_TIMER timer
}

void loop() {
  if(110MS_TIMER.justFinished()){
    UltrasonicSweeping();
    Serial.print(DISTANCE_ARRAY[0]);
    Serial.print(", ");
    Serial.print(DISTANCE_ARRAY[1]);
    Serial.print(", ");
    Serial.print(DISTANCE_ARRAY[2]);
    Serial.print("\n");
    110MS_TIMER.repeat();
  }
}
