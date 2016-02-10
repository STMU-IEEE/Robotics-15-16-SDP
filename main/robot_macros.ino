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

void get_E_city(){
 
  //lower arm
  arm_servo.write(ARM_DOWN);
  //open grabber
  grabber_servo.write(GRABBER_OPEN);
  delay(500);
   ST.drive(35);
  while(photogateAverage() > PHOTOGATE_LOW){
    
   followSRFs(srf_FR,srf_R,false,9);// its moving foward and the minimum distance is 9cm
    
  }
  
  while(photogateAverage() < PHOTOGATE_HIGH);
    

    //stop
  ST.stop();

  //close grabber
  grabber_servo.write(GRABBER_CLOSE);
  //wait for grabber to close
  delay(500);
  //raise arm
  arm_servo.write(ARM_UP);
  delay(1000);
  testColor();

   
   
  /*do
  {
  }while(srf_R.convert_cm(last_SRF_R<number);
  */
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

/*
Better(?) wall following algorithm using two sensors
(no idea if described somewhere online already)

Accounts for two problematic cases in old algorithm:
 -	sensor says "too far" but already facing toward wall,
 	turning more toward wall causing robot to crash
 -	sensor says "too close" but already facing away from wall,
 	turning more away causing robot to stray from path
 	(and maybe into wall on other side)
 
 New algorithm accounts for orientation of robot relative to wall;
 these cases are now handled by keeping robot straight instead of turning,
 which *should* cause robot to converge to desired distance from wall.
 
 t1 is distance of front sensor, t2 is distance of center sensor
 t2 is adjusted by T2_OFFSET_X, where
 	srf_1.ping() - (srf_2.ping() + D2_OFFSET_X) == 0
 when the robot is oriented parallel to the wall,
 and X is either L or R representing each side of the robot.
 (Theoretically, D2_OFFSET is positive if center sensor is closer
 due to standoff mount, etc., but it has been observed to still be negative,
 about ~100uS below the expected amount.
 A slight change in orientation greatly affects t1-t2,
 probably such that T2_OFFSET_X is negligible.)
 
 Table of cases:
        
        Forward, right wall:
                        |   t1 < t2         |   t1 > t2        |
                        |   (facing wall)   |   (facing away)  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 < target    |   turn away (-)   |   go straight    |
         (too close)    |                   |                  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 > target    |   go straight     |   turn toward (+)|
         (too far)      |                   |                  |
         ---------------+-------------------+------------------+
         
         
         Backwards, right wall:
                        |   t1 < t2         |   t1 > t2        |
                        |   (facing away)   |   (facing wall)  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 < target    |   go straight     |   turn toward (-)|
         (too close)    |                   |                  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 > target    |    turn away (+)  |   go straight    |
         (too far)      |                   |                  |
         ---------------+-------------------+------------------+
         
         
         
         Forward, left wall:
                        |   t1 < t2         |   t1 > t2        |
                        |   (facing wall)   |   (facing away)  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 < target    |   turn away (+)   |   go straight    |
         (too close)    |                   |                  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 > target    |   go straight     |   turn toward (-)|
         (too far)      |                   |                  |
         ---------------+-------------------+------------------+
         
         
         Backwards, left wall:
                        |   t1 < t2         |   t1 > t2        |
                        |   (facing away)   |   (facing wall)  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 < target    |   go straight     |   turn toward (+)|
         (too close)    |                   |                  |
                        |                   |                  |
         ---------------+-------------------+------------------+
                        |                   |                  |
         d2 > target    |    turn away (-)  |   go straight    |
         (too far)      |                   |                  |
         ---------------+-------------------+------------------+
 
*/
//The following declaration Must Be On One Line
//in order for the prototype to be generated.
//cf. https://github.com/arduino/arduino-builder/issues/80
void followSRFs(NewPing& srf_front, NewPing& srf_center, bool is_driving_backwards, unsigned int target_distance){
	unsigned long timeNow = millis();
	if(timeNow - lastSRF >= 50){
		//depending on which sensor is given, turn the robot left or right
		//by changing sign of drive power
		int turn_power;
		unsigned int t2_offset;
		const int TURN_AMOUNT = 5; //adjust if needed
		if      (&srf_center == &srf_L){
			turn_power = is_driving_backwards ? TURN_AMOUNT : -TURN_AMOUNT;
			t2_offset = T2_OFFSET_L;
		}
		else if (&srf_center == &srf_R){
			turn_power = is_driving_backwards ? -TURN_AMOUNT : TURN_AMOUNT;
			t2_offset = T2_OFFSET_R;
		}
		//else: not a valid srf???
		
		//take readings
		lastSRF = timeNow;
		unsigned int t2 = srf_center.ping() + t2_offset;
		do{
			timeNow = millis();
		} while(timeNow - lastSRF < 50);
		lastSRF = timeNow;
		unsigned int t1 = srf_front.ping();
		
		Serial.print("d1: ");
		Serial.println(srf_center.convert_cm(t1));
		
		unsigned int d2 = srf_center.convert_cm(t2);
		Serial.print("d2: ");
		Serial.println(d2);
		
		//compare readings
		if(!is_driving_backwards){
			if(		(d2 < target_distance)
				&&	(t1 < t2)){
				//turn away from wall
				ST.turn(-turn_power);
				Serial.println("Away");
			}
			else if((d2 > target_distance)
				&&	(t1 > t2)){
				//turn toward wall
				ST.turn(turn_power);
				Serial.println("Toward");
			}
			else{
				//go straight
				ST.turn(0);
				Serial.println("Straight");
			}
		}
		else{
			//backwards
			if(		(d2 > target_distance)
				&&	(t1 < t2)){
				//turn away from wall
				ST.turn(turn_power);
				Serial.println("Away");
			}
			else if((d2 < target_distance)
				&&	(t1 > t2)){
				//turn toward wall
				ST.turn(-turn_power);
				Serial.println("Toward");
			}
			else{
				//go straight
				ST.turn(0);
				Serial.println("Straight");
			}
		}
	}
}


