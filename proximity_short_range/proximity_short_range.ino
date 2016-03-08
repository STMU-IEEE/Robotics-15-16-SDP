#include <DistanceGP2Y0A21YK.h>
#include <DistanceGP2Y0A21YK_LUTs.h>

DistanceGP2Y0A21YK Dist;
int distance;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Dist.begin(0);
 

}

void loop() {
  // put your main code here, to run repeatedly:
  distance = Dist.getDistanceCentimeter();
  Serial.print("\nDistance in centimers: ");
  Serial.print(distance);  
  delay(500); //make it readable

  

}
