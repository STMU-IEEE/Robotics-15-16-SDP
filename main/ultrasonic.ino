void srfTest() {
  //perform srf left test
  delay(50);
  Serial.print("Left ping: ");
  Serial.print(srf_L.ping_cm());
  Serial.println("cm");   

  //perform srf right test
  delay(50);
  Serial.print("Right ping: ");
  Serial.print(srf_R.ping_cm());
  Serial.println(" cm");
  
  //perform srf front test
  delay(50);
  Serial.print("Front ping: ");
  Serial.print(srf_F.ping_cm());
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

