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

