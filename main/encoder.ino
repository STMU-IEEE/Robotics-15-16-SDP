//Continuously print encoder position--move wheels manually to change readings
void encoderReadTest() {
  Serial.println("Encoder position:");
  Serial.println("L\t\tR");
  while(true){
    Serial.print(motor_L_encoder.read());
    Serial.print("\t\t");
    Serial.println(motor_R_encoder.read());
    Serial.print("\n");
  }
}

//go straight using gyro PID until one forward rotation of right motor
void encoderDistanceTest() {
  Serial.print ("Enter rotations: ");
  while(Serial.available() < 2);
  long n = Serial.parseInt();
  Serial.println(n);
  Serial.println("Zeroing encoders...");
  motor_L_encoder.write(0);
  motor_R_encoder.write(0);

  //initialize PID
  angle = 0;             //start with angle 0
  gyro_PID_setpoint = 0; //keep angle at 0
  gyro_PID_output = 64; //start without turning
  
  //go forward (or backwards)
  ST.drive(25);
  ST.turn(0);

  //wait for n full turns forward
  while(motor_L_encoder.read() <= n * MOTOR_COUNTS_PER_REVOLUTION)
    if(followGyro())
      Serial.println(gyro_PID_output);
    
  //stop
  ST.stop();
  
  //stop PID
  gyroPID.SetMode(MANUAL);
  
  Serial.println("Final positions:");
  Serial.print("L\t");
  Serial.println(motor_L_encoder.read());
  Serial.print("R\t");
  Serial.println(motor_R_encoder.read());
}

