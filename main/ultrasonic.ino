void srfTest() {
  //static int8_t nextSensor = 0;
  unsigned long timeNow;
  
  do
    timeNow = millis();
  while(timeNow - last_SRF_trigger < 50);
  last_SRF_trigger = timeNow;
  Serial.print("Left ping: ");
  Serial.print(srf_L.ping_cm());
  Serial.println("cm");   
	
  /*do
    timeNow = millis();
  while(timeNow - last_SRF_trigger < 50);
  last_SRF_trigger = timeNow;
  Serial.print("Right ping: ");
  Serial.print(srf_R.ping_cm());
  Serial.println(" cm");
  
  do
    timeNow = millis();
  while(timeNow - last_SRF_trigger < 50);
  last_SRF_trigger = timeNow;
  Serial.print("Front ping: ");
  Serial.print(srf_F.ping_cm());
  Serial.println(" cm");
  
 /* do
    timeNow = millis();
  while(timeNow - last_SRF_trigger < 50);
  last_SRF_trigger = timeNow;
  Serial.print("Front right ping: ");
  Serial.print(srf_FR.ping_cm());
  Serial.println(" cm");
  */
  do
    timeNow = millis();
  while(timeNow - last_SRF_trigger < 50);
  last_SRF_trigger = timeNow;
  Serial.print("Front left ping: ");
  Serial.print(srf_FL.ping_cm());
  Serial.println(" cm");
}

int srfOffset(){
  unsigned long timeNow;
  do
    timeNow = millis();
  while(timeNow - last_SRF_trigger < 50);
  last_SRF_trigger = timeNow;
  unsigned int uS_R = srf_R.ping();

  do
    timeNow = millis();
  while(timeNow - last_SRF_trigger < 50);
  last_SRF_trigger = timeNow;
  unsigned int uS_FR = srf_FR.ping(); 
  Serial.print("Offset: ");
  int result = (int)(uS_FR - uS_R);
  Serial.print(result);
  Serial.println(" uS");
  return result;
}

//follows wall for 10s forwards and backwards
void wallFollower(NewPing& srf_front, NewPing& srf_center){
  const int8_t drive_power = 35;
  const unsigned int test_distance = 13;
  ST.drive(drive_power);
  unsigned long start = millis();
  while(millis() - start < 10000)
    followSRFs(srf_front,srf_center,false,test_distance);
  ST.drive(-drive_power);
  start = millis();
  while(millis() - start < 10000)
    followSRFs(srf_front,srf_center,true,test_distance);
  ST.stop();
}

