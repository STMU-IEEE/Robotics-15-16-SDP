void encoderTest() {
  Serial.println("Encoder position:");
  Serial.println("L\t\tR");
  while(true){
    Serial.print(motor_L_encoder.read());
    Serial.print("\t\t");
    Serial.println(motor_R_encoder.read());
    Serial.print("\n");
  }
}

