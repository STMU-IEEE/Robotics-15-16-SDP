//place inside your own while loop, e.g. while(true)
//returns distance for reading, or 0 if no reading (same as 
unsigned long findOpening(NewPing SRF){ 
  unsigned long timeNow = millis();
  unsigned long srf_reading;  //for debugging
  if(timeNow - lastSRF > 50){
    lastSRF = timeNow;
    srf_reading = SRF.ping_cm();
    Serial.println(srf_reading);
  }
  if(srf_reading < 3)
    break;
  followGyro();
}
