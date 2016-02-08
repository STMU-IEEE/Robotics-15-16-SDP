// ---------------------------------------------------------------------------
// Based on Example NewPing library sketch
// ---------------------------------------------------------------------------

#include <NewPing.h>

//first sensor is SR04
#define TRIGGER_PIN_1  44  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_1     45  // Arduino pin tied to echo pin on the ultrasonic sensor.
//second sensor is SRF05 (single pin mode)
#define TRIGGER_PIN_2  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_2     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//create NewPing objects
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

/*  Purpose of offset is to measure microseconds between sensors
	that should be returning identical readings when parallel to object
	i.e. t1 == t2 + offset
*/
const unsigned int offset = 79;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  //read us from each sensor
  unsigned int t1 = sonar1.ping();
  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int t2 = sonar2.ping() + offset;
  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  
  //compute difference
  unsigned int difference_us = abs((int)(t1 - t2));
  
  //convert difference to cm and print
  unsigned int difference_cm = NewPing::convert_cm(difference_us);
  
  Serial.print("Difference: ");
  Serial.print(difference_cm);
  Serial.println("cm");
}