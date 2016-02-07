void srfTest() {
  //static int8_t nextSensor = 0;
  unsigned long timeNow;
  do
    timeNow = millis();
  while(timeNow - lastSRF < 50);
  lastSRF = timeNow;
  Serial.print("Left ping: ");
  Serial.print(srf_L.ping_cm());
  Serial.println("cm");   

  do
    timeNow = millis();
  while(timeNow - lastSRF < 50);
  lastSRF = timeNow;
  Serial.print("Right ping: ");
  Serial.print(srf_R.ping_cm());
  Serial.println(" cm");
  
  do
    timeNow = millis();
  while(timeNow - lastSRF < 50);
  lastSRF = timeNow;
  Serial.print("Front ping: ");
  Serial.print(srf_F.ping_cm());
  Serial.println(" cm");
  
  do
    timeNow = millis();
  while(timeNow - lastSRF < 50);
  lastSRF = timeNow;
  Serial.print("Front right ping: ");
  Serial.print(srf_FR.ping_cm());
  Serial.println(" cm");
}

//follows wall for 10s forwards and backwards
void wallFollower(NewPing& srf){
  ST.drive(30);
  unsigned long start = millis();
  while(millis() - start < 10000)
    followSRF(srf,false);
  ST.drive(-30);
  start = millis();
  while(millis() - start < 10000)
    followSRF(srf,true);
  ST.stop();
}

