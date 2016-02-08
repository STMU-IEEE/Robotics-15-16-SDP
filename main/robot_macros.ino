//blink color sensor LED once
void ledBlink(unsigned long delay_ms) {
  digitalWrite(COLOR_LED_PIN, HIGH);
  delay(delay_ms/2);
  digitalWrite(COLOR_LED_PIN, LOW);
  delay(delay_ms/2);
}

//assumes driving, gyro PID enabled (mode == AUTOMATIC)
void findOpening(NewPing srf){  
  unsigned long timeNow;
  unsigned long srf_reading;
  //wait until opening to lane 2 on left
  while(true){
    timeNow = millis();
    if(timeNow - lastSRF >= 50){
      lastSRF = timeNow;
      srf_reading = srf.ping_cm();
      //Serial.println(srf_reading);
    }
    if(srf_reading > 36)
      break;
    followGyro();
  }

  //save encoder value for opening
  int32_t encoder_opening = motor_L_encoder.read();

  while(motor_L_encoder.read() > (encoder_opening - (MOTOR_COUNTS_PER_REVOLUTION / 2)))
    followGyro();
  
  //wait until opening to lane 2 on left 
  while(true){
    timeNow = millis();
    if(timeNow - lastSRF >= 50){
      lastSRF = timeNow;
      srf_reading = srf.ping_cm();
      //Serial.println(srf_reading);
    }
    if(srf_reading < 25)
      break;
    followGyro();
  }
    //save encoder value for wall
  int32_t encoder_wall = motor_L_encoder.read();
  
  //Serial.print("Opening:\t");
  //Serial.println(encoder_opening);
  //Serial.print("Wall:\t");
  //Serial.println(encoder_wall);

  //reverse to middle of opening
  ST.drive(-25);
  while(motor_L_encoder.read() < ((encoder_opening + encoder_wall)/ 2))
    followGyro();

  //Serial.print("Stop:\t");
  ST.stop();
  //Serial.println(motor_L_encoder.read());
}

void followSRF(NewPing& srf, bool is_driving_backwards){
  unsigned long timeNow = millis();
  if(timeNow - lastSRF >= 50){
    lastSRF = timeNow;
    //depending on which sensor is given, turn the robot left or right (change sign of drive power)
    int turn_sign;
    const int TURN_POWER = 2;
    if(&srf == &srf_L)
      turn_sign = is_driving_backwards ? -1 : 1;
    else if (&srf == &srf_R)
      turn_sign = is_driving_backwards ? 1 : -1;
    //else: not a valid srf???
    
    unsigned long srf_reading = srf.ping_cm();
    Serial.println(srf_reading);
    if(srf_reading < 9)
      //turn away from wall
      ST.turn((-TURN_POWER) * turn_sign);
    //else if(srf_reading == 9)
      //go straight
      //ST.turn(0);
    else //if(srf_reading > 9)
      //turn toward wall
      ST.turn(TURN_POWER * turn_sign);
  }
}
