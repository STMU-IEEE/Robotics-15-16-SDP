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
	
  /*do
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
  
 /* do
    timeNow = millis();
  while(timeNow - lastSRF < 50);
  lastSRF = timeNow;
  Serial.print("Front right ping: ");
  Serial.print(srf_FR.ping_cm());
  Serial.println(" cm");
  */
  do
    timeNow = millis();
  while(timeNow - lastSRF < 50);
  lastSRF = timeNow;
  Serial.print("Front left ping: ");
  Serial.print(srf_FL.ping_cm());
  Serial.println(" cm");
}

int srfOffset(){
  unsigned long timeNow;
  do
    timeNow = millis();
  while(timeNow - lastSRF < 50);
  lastSRF = timeNow;
  unsigned int uS_R = srf_R.ping();

  do
    timeNow = millis();
  while(timeNow - lastSRF < 50);
  lastSRF = timeNow;
  unsigned int uS_FR = srf_FR.ping(); 
  Serial.print("Offset: ");
  int result = (int)(uS_FR - uS_R);
  Serial.print(result);
  Serial.println(" uS");
  return result;
}

//follows wall for 10s forwards and backwards
void wallFollower(NewPing& srf_front, NewPing& srf_center){
  const int drive_power = 35;
  ST.drive(drive_power);
  unsigned long start = millis();
  while(millis() - start < 10000)
    followSRFs(srf_front,srf_center,false);
  ST.drive(-drive_power);
  start = millis();
  while(millis() - start < 10000)
    followSRFs(srf_front,srf_center,true);
  ST.stop();
}

