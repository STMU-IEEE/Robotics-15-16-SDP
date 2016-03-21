//blink color sensor LED once
void led_blink(unsigned long delay_ms) {
    digitalWrite(COLOR_LED_PIN, HIGH);
    delay(delay_ms/2);
    digitalWrite(COLOR_LED_PIN, LOW);
    delay(delay_ms/2);
}


void leave_starting_area() {
	Serial.println("leave_starting_area()");
	
    motor_L_encoder.write(0);
    motor_R_encoder.write(0);
    
    angle = 0;             //start with angle 0
    gyro_PID_setpoint = 0; //keep angle at 0
    
    //go to opening to lane 2
    find_opening(srf_L,30);
    
    //turn left 90 degrees
    ST.drive(0);
    ST.turn(-16);
    gyro_angle(-90);
    
}

void L1_to_L2(){
	Serial.println("L1_to_L2()");
    
    //go forward toward wall
    gyro_PID_setpoint = -90;
    ST.drive(35);
    do {
        if(millis() - last_srf_trigger_ms > 50){
            last_srf_trigger_ms = millis();
            last_srf_F_echo_us = srf_F.ping_cm();
            Serial.println(last_srf_F_echo_us);
        }
        follow_gyro();
    } while (last_srf_F_echo_us >= 4);
    
    //turn right 45 degrees in place
    ST.drive(0);
    ST.turn(16);
    gyro_angle(-45);
    
    ST.drive(10);
    ST.turn(10);
    gyro_angle(-5); //don't crash into L1-L2
    
}

victim_color get_E_city(){
	Serial.println("get_E_city()");
    //lower arm
    arm_servo.write(ARM_DOWN);
    //open grabber
    grabber_servo.write(GRABBER_OPEN);
    ST.drive(40); //can't make too fast (e.g. 60)--will get stuck against wall
    while(photogate_average() > PHOTOGATE_LOW){
        follow_srf(srf_FR,srf_R,false,7);// its moving foward and the minimum distance is 7cm
    }
    while(photogate_average() < PHOTOGATE_HIGH);
    
    //stop
    ST.stop();
    
    //close grabber
    grabber_servo.write(GRABBER_CLOSE);
    //wait for grabber to close
    delay(500);
    //raise arm
    arm_servo.write(ARM_UP);
    delay(1000);
    victim_color result = get_color();
    
    ST.drive(-40);
    
    //go until opening to lane 1
    do {
        follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
    } while (srf_R.convert_cm(last_srf_R_echo_us) < 30);
    
    return result;
}

void dropoff_R(){
	Serial.println("dropoff_R()");
    
    ST.turn(0);
    ST.drive(60);
    
    
    
    do {
        while(!follow_srf(srf_FR,srf_R,false,14));// its moving forward and the minimum distance is 14cm
        
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_F_echo_us = srf_F.ping();
        Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
        
    } while (srf_F.convert_cm(last_srf_F_echo_us) >= 20); //need enough room to drop arm
    
    ST.stop();
    //drop victim
    arm_servo.write(ARM_DOWN);
    delay(300);
    grabber_servo.write(GRABBER_OPEN);
    delay(300);
    arm_servo.write(ARM_UP);
    delay(300);
    grabber_servo.write(GRABBER_CLOSE);
    ST.turn(0);
    ST.drive(-60);
    do {
        while(!follow_srf(srf_FR,srf_R,true,14));// its moving backwards and the minimum distance is 14cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 36); //find L1-L2 opening
    ST.stop();
}

void depart_from_R_dropoff(){
	Serial.println("depart_from_R_dropoff()");
    
    find_opening(srf_L, -25);
    
    angle = 0;
    ST.drive(0);
    ST.turn(-16);
    gyro_angle(-90);
    ST.stop();
    
}

void R_dropoff_to_start(){
	Serial.println("R_dropoff_to_start()");
    ST.turn(0);
    ST.drive(-25);
    while(rear_average() < PROXIMITY_THRESHOLD){
        follow_srf(srf_FR,srf_R,true,14);
    }
}

void dropoff_E_city_Y(){
	Serial.println("dropoff_E_city_Y()");
    //swing turn back toward lane 1
    angle = 0;
    ST.drive(-10);
    ST.turn(-10);
    gyro_angle(-90);
    //swing turn into dropoff
    ST.drive(10);
    ST.turn(-10);
    gyro_angle(-180);
    ST.stop();
    
    //drop victim
    arm_servo.write(ARM_DOWN);
    delay(300);
    grabber_servo.write(GRABBER_OPEN);
    delay(300);
    arm_servo.write(ARM_UP);
    delay(300);
    grabber_servo.write(GRABBER_CLOSE);
    
}

void depart_from_Y_1(){
	Serial.println("depart_from_Y_1()");
    //back up to opening
    ST.drive(-40);
    
    //go until opening to lane 3
    do {
        follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
    } while (srf_R.convert_cm(last_srf_R_echo_us) < 30);
    
    //turn facing lane 3
    ST.drive(-10);
    ST.turn(-10);
    gyro_angle(angle-90);
    ST.stop();
}

void depart_from_Y_2(){
	Serial.println("depart_from_Y_2()");
    //back up to opening
    ST.drive(-30);
    
    //compensate encoders before turning at end of lane 2
    encoder_compensate_initialize();
    
    //go until 1st opening to lane 3
    do {
        while(!follow_srf(srf_FR,srf_R,true,7));// its moving backwards and the minimum distance is 7cm
        encoder_compensate_sample();
    } while (srf_R.convert_cm(last_srf_R_echo_us) < 30);
    
    //keep going to L2-L3 center wall
    do {
        while(!follow_srf(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_R_echo_us = srf_R.ping();
        Serial.println(srf_R.convert_cm(last_srf_R_echo_us));
        encoder_compensate_sample();
    } while (srf_R.convert_cm(last_srf_R_echo_us) > 30); //find L2-L3 center wall
    
    //go until 2nd opening to lane 3
    do {
        while(!follow_srf(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_R_echo_us = srf_R.ping();
        Serial.println(srf_R.convert_cm(last_srf_R_echo_us));
        encoder_compensate_sample();
    } while (srf_R.convert_cm(last_srf_R_echo_us) < 36); //find L2-L3 center wall
    
    //go until last L2-L3 wall
    do {
        while(!follow_srf(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_R_echo_us = srf_R.ping();
        Serial.println(srf_R.convert_cm(last_srf_R_echo_us));
        encoder_compensate_sample();
    } while (srf_R.convert_cm(last_srf_R_echo_us) > 30); //find L2-L3 center wall
    
    //go back toward opening slightly
    ST.turn(0);
    ST.drive(16);
    motor_R_encoder.write(0);
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 4));
    
    //swing turn facing lane 3
    //encoder_compensate_apply(true);
    ST.drive(10);
    ST.turn(10);
    gyro_angle(angle+45);
    ST.turn(0);
    ST.drive(16);
    motor_R_encoder.write(0);
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    ST.drive(10);
    ST.turn(10);
    gyro_angle(angle+45);
    ST.stop();
}
//victim_color get_W_city(){
void get_W_city(){
	Serial.println("get_W_city()");
    //go backward from wall
    ST.turn(0);
    ST.drive(-25);
    gyro_PID_setpoint = angle;
    do {
        if(millis() - last_srf_trigger_ms > 50){
            last_srf_trigger_ms = millis();
            last_srf_F_echo_us = srf_F.ping_cm();
            Serial.println(last_srf_F_echo_us);
        }
        follow_gyro();
    } while (last_srf_F_echo_us < 20);
    
    //turn facing W city victim
    ST.turn(10);
    ST.drive(-10);
    gyro_angle(angle+90);
    
    //go until wall on right
    ST.turn(0);
    ST.drive(40);
    motor_L_encoder.write(0);
    gyro_PID_setpoint = angle;
    while( -motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 3)/2)
        follow_gyro();
    
    //follow wall on left until victim
    //lower arm
    arm_servo.write(ARM_DOWN);
    //open grabber
    grabber_servo.write(GRABBER_OPEN);
    //delay(500);
    while(photogate_average() > PHOTOGATE_LOW){
        follow_srf(srf_FL,srf_L,false,8);// its moving foward and the minimum distance is 7cm
    }
    while(photogate_average() < PHOTOGATE_HIGH);
    
    //stop
    ST.stop();
    
    //close grabber
    grabber_servo.write(GRABBER_CLOSE);
    //wait for grabber to close
    delay(500);
    //raise arm
    arm_servo.write(ARM_UP);
    delay(300);
    
    ST.drive(-40);
    
    //go until opening to lane 2
    do {
        follow_srf(srf_FL,srf_L,true,8);// its moving backwards and the minimum distance is 7cm
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 30);
    
    ST.turn(0);
    
    //swing turn backwards into lane 2
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+45);
    //ST.turn(0);
    //ST.drive(-16);
    //motor_L_encoder.write(0);
    //while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+45);
    ST.stop();
   
}


//not for E city
void dropoff_Y (){
	Serial.println("dropoff_Y()");
    ST.drive(25);
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
        
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_F_echo_us = srf_F.ping();
        Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
        
    } while (srf_F.convert_cm(last_srf_F_echo_us) >= 17); //need enough room to drop arm
    
    ST.stop();
    //drop victim
    arm_servo.write(ARM_DOWN);
    delay(300);
    grabber_servo.write(GRABBER_OPEN);
    delay(300);
    arm_servo.write(ARM_UP);
    delay(300);
    grabber_servo.write(GRABBER_CLOSE);
}


//for red dropoff
void L2_W_to_L1_E(){
    Serial.println("L2_W_to_L1_E()");
    //find opening on left while following wall on right
    ST.drive(30);
    ST.turn(0);
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 36); //find L1-L2 wall
    
    //find inside of Y drop off on other side of opening
    //go 1/4 turn before detecting
    motor_R_encoder.write(0);
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 4));
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) > 25); //find inside of Y drop off
    
    //turn into lane 1 using swing turns
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+45);
    
    //go backwards before completing turn
    ST.turn(0);
    ST.drive(-16);
    motor_L_encoder.write(0);
    while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    
    //complete turn
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+45);
    
    //go backwards into S wall
    gyro_PID_setpoint = angle;
    ST.turn(0);
    ST.drive(-20);
    while(rear_average() < PROXIMITY_THRESHOLD){
        follow_gyro();
    }
    
    //swing turn toward R dropoff
    ST.drive(10);
    ST.turn(10);
    gyro_angle(angle+90);
    
    ST.stop();
}

//return to starting area
void Y_dropoff_to_start(){
    Serial.println("Y_dropoff_to_start()");
    //find opening on left while following wall on right
    ST.drive(-20);
    ST.turn(0);
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));//moving backwards, target distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 36); //find L1-L2 wall
    
    //turn into lane 1 using swing turns
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+45);
    
    //go backwards before completing turn
    ST.turn(0);
    ST.drive(-16);
    motor_L_encoder.write(0);
    while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    
    //complete turn
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+45);
    
    //go backwards into S wall
    gyro_PID_setpoint = angle;
    ST.turn(0);
    ST.drive(-20);
    while(rear_average() < PROXIMITY_THRESHOLD){
        follow_gyro();
    }
    
    //swing turn toward R dropoff
    ST.drive(10);
    ST.turn(10);
    gyro_angle(angle+90);
    
    //go backwards into starting area
    ST.turn(0);
    ST.drive(-20);
    while(rear_average() < PROXIMITY_THRESHOLD){
        follow_srf(srf_FR,srf_R,true,14); //going backwards, target distance is 14cm
    }
    
    ST.stop();
}

void back_into_Y_then_face_L1(){
	Serial.println("back_into_Y_then_face_L1()");
    //start before opening to L1, facing E
    //keep backing up until inside Y dropoff
    gyro_PID_setpoint = angle;
    ST.turn(0);
    ST.drive(-20);
    do {
        while(!follow_srf(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_R_echo_us = srf_R.ping();
    } while (srf_R.convert_cm(last_srf_R_echo_us) > 25);
    //swing turn forward right toward L1
    ST.drive(10);
    ST.turn(10);
    angle = 0;
    gyro_angle(45);
    //go forward 1/6 turn
    motor_R_encoder.write(0);
    ST.turn(0);
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    //keep swing turning toward L1
    ST.turn(10);
    gyro_angle(90);
}

//for R drop off
void L2_to_L1() {
    Serial.println("L2_to_L1()");
    //go forward toward wall
    gyro_PID_setpoint = angle;
    ST.turn(0);
    ST.drive(40);
    
    do {
        if(millis() - last_srf_trigger_ms > 50){
            last_srf_trigger_ms = millis();
            last_srf_F_echo_us = srf_F.ping_cm();
            Serial.println(last_srf_F_echo_us);
        }
        follow_gyro();
    } while (last_srf_F_echo_us >= 6);
    
    //turn left 90 degrees in place
    ST.drive(0);
    ST.turn(-16);
    gyro_angle(angle-90);
    
    ST.stop();
}

void L2_E_to_L2_S_B(){
	Serial.println("L2_E_to_L2_S_B()");
    //go forward to L2-L3 W opening
    ST.drive(35);
    do {
        while(!follow_srf(srf_FR,srf_R,false,9));// its moving forward and the minimum distance is 9cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 36);
    
    //keep going to L2-L3 center wall
    do {
        while(!follow_srf(srf_FR,srf_R,false,9));// its moving forward and the minimum distance is 9cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) > 30); //find L2-L3 center wall
    angle = 0;
    //swing backward turning right, face L1-L2 wall
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(45);
    //back up enough to clear wall
    ST.turn(0);
    motor_L_encoder.write(0);
    while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    //complete turn
    ST.turn(10);
    gyro_angle(90);
}

void L2_E_to_L2_N() {
	Serial.println("L2_E_to_L2_N()");
    //compensate encoders before turning into opening
    encoder_compensate_initialize();
    //go forward to L2-L3 W opening
    ST.drive(40);
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        encoder_compensate_sample();
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 36);
    
    //keep going to L2-L3 center wall
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        encoder_compensate_sample();
    } while (srf_L.convert_cm(last_srf_L_echo_us) > 30); //find L2-L3 center wall
    
    //keep going to second L2-L3 opening
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        encoder_compensate_sample();
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 36); //find L2-L3 center wall
    
    encoder_compensate_apply(true);
    //swing turn into L2-L3 opening
    ST.drive(10);
    ST.turn(-10);
    gyro_angle(angle-90);
    ST.stop();
    delay(1000);
}


victim_color get_E_offroad(){
	Serial.println("get_E_offroad()");
    victim_color result; //to be updated and returned at end of function
    
    //back into L1-L2 for reference
    ST.turn(0);
    ST.drive(-20);
    while(rear_average() < PROXIMITY_THRESHOLD);
    
    //go forward 2.67 rotations before swing turn (rev*8/3)
    ST.drive(20);
    motor_R_encoder.write(0);
    gyro_PID_setpoint = angle;
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 3)){
        follow_gyro();
    }
    
    ST.drive(15);
    ST.turn(15);
    gyro_angle(angle+90);
    
    //move foward .5 rotations.
    ST.drive(25);
    motor_R_encoder.write(0);
    gyro_PID_setpoint += 90;
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 2) / 3){
        follow_gyro();
    }
    
    //swing turn left 90 degrees
    ST.drive(15);
    ST.turn(-15);
    gyro_angle(angle-90);
    
    
    //its going to put the grabber down, follow the wall and its going to detect if NE victim is present
    ST.stop();
    //lower arm
    arm_servo.write(ARM_DOWN);
    //open grabber
    grabber_servo.write(GRABBER_OPEN);
    ST.drive(40);
    motor_R_encoder.write(0);
    
    bool is_ENE_victim_present;
    //its going to follow the wall a certain distance or it will detect the NE victim
    
    while(true){
        follow_srf(srf_FR,srf_R,false,7);// its moving foward and the minimum distance is 7cm
        if(photogate_average() < PHOTOGATE_LOW){
            is_ENE_victim_present=true;
            break;
        }
        if(motor_R_encoder.read() > (MOTOR_COUNTS_PER_REVOLUTION * 7)/2){
            is_ENE_victim_present=false;
            break;
        }
    }
    // it we have the NE victim, then wait until its in the correct position and get it.
    //pick up victim
    if(is_ENE_victim_present){
        while((photogate_average() < PHOTOGATE_HIGH));
        ST.stop();
        //close grabber
        grabber_servo.write(GRABBER_CLOSE);
        //wait for grabber to close
        delay(500);
        //raise arm
        arm_servo.write(ARM_UP);
        delay(1000);
        result = get_color();
        //delay(5000);//debugging: read results
        ST.stop();
        
        ST.turn(0);
        ST.drive(-20);
        while(rear_average() < PROXIMITY_THRESHOLD){
            follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
        }
        ST.stop();
        delay(500);
        ST.drive(0);
        ST.turn(-15);
        gyro_angle(angle-90);
        ST.stop();
        ST.drive(20);
        motor_R_encoder.write(0);
        gyro_PID_setpoint = angle;
        while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 7)/3){
            follow_gyro();
        }
        
        ST.stop();
        ST.drive(0);
        ST.turn(-15);
        gyro_angle(angle-90);
    }
    // lets get victim NNE but we have to be in the right position
    else{
        //stop after 3 rotations, raise the arm up
        ST.stop();
        arm_servo.write(ARM_UP);
        
        //turn to the left 180 degrees. no we are facing the city section
        ST.drive(0);
        ST.turn(-20);
        gyro_angle(angle-180);
        ST.stop();
        
        ST.turn(0);
        ST.drive(-20);
        while(rear_average() < PROXIMITY_THRESHOLD){
            follow_srf(srf_FL,srf_L,true,7);// its moving backwards and the minimum distance is 7cm
        }
        ST.stop();
        delay(500);
        
        ST.drive(20);
        motor_R_encoder.write(0);
        gyro_PID_setpoint = angle;
        while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 1) / 4){
            follow_gyro();
        }
        ST.stop();
        delay(500);
        
        ST.drive(0);
        ST.turn(15);
        gyro_angle(angle+90);
        ST.stop();
        
        ST.turn(0);
        ST.drive(-20);
        while(rear_average() < PROXIMITY_THRESHOLD){
            follow_srf(srf_FR,srf_R,true,6);// its moving backwards and the minimum distance is 7cm
        }
        
        //its going to put the grabber down, follow the wall and its going to detect if NE victim is present
        ST.stop();
        //lower arm
        arm_servo.write(ARM_DOWN);
        //open grabber
        grabber_servo.write(GRABBER_OPEN);
        delay(500);
        
        ST.drive(30); //can't make too fast (e.g. 60)--will get stuck against wall
        while(photogate_average() > PHOTOGATE_LOW){
            follow_srf(srf_FR,srf_R,false,6);// its moving foward and the minimum distance is 7cm
        }
        while(photogate_average() < PHOTOGATE_HIGH);
        
        //stop
        ST.stop();
        
        //close grabber
        grabber_servo.write(GRABBER_CLOSE);
        //wait for grabber to close
        delay(500);
        //raise arm
        arm_servo.write(ARM_UP);
        delay(1000);
        victim_color result = get_color();
        
        follow_N_wall();
    }
    ST.stop();
}

void follow_N_wall(){

    ST.drive(-20);
    while(rear_average() < PROXIMITY_THRESHOLD){
        follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
    }
    
    // now lets get back to the dropoff zones.
    ST.turn(0);
    ST.drive(20);
    motor_R_encoder.write(0);
    gyro_PID_setpoint = angle;
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 1) / 4){
        follow_gyro();
    }
    ST.stop();
    
    ST.drive(0);
    ST.turn(-20);
    gyro_angle(angle-90);
    ST.stop();
    delay(500);
    
    ST.drive(30);
    motor_R_encoder.write(0);
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 2)){
        follow_srf(srf_FL,srf_L,false,7);// its moving foward and the minimum distance is 7cm
    }
    
    ST.stop();
    
    delay(300);
    
    ST.drive(0);
    ST.turn(25);
    gyro_angle(angle+180);
    
    ST.turn(0);
    ST.drive(-30);
    while(rear_average() < PROXIMITY_THRESHOLD){
        follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
    }
    ST.stop();
    
    
    delay(500);
    ST.drive(0);
    ST.turn(-15);
    gyro_angle(angle-90);
    ST.stop();
    
    
    ST.drive(20);
    motor_R_encoder.write(0);
    gyro_PID_setpoint = angle;
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 7)/3){
        follow_gyro();
    }
    
    ST.stop();
    ST.drive(0);
    ST.turn(-15);
    gyro_angle(angle-90);
    ST.stop();
}

void get_W_offroad() {
	Serial.println("get_W_offroad()");
    //go backward from wall
    ST.turn(0);
    ST.drive(-20);
    gyro_PID_setpoint = angle;
    
    //go back from L1-L2 and watch for L2-L3 and L3-offroad walls
    for(int i = 1; i <= 2; i++){
        //go 1/3 turn before detecting next wall
        motor_L_encoder.write(0);
        while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 3))
            follow_gyro();
        
        do {
            if(millis() - last_srf_trigger_ms > 50){
                last_srf_trigger_ms = millis();
                last_srf_FR_echo_us = srf_FR.ping();
                Serial.print("Front right distance: ");
                Serial.println(srf_FR.convert_cm(last_srf_FR_echo_us));
            }
            follow_gyro();
        } while(srf_FR.convert_cm(last_srf_FR_echo_us) > 10);
        Serial.print("Wall ");
        Serial.println(i);
        
        //go 1/3 turn before detecting next opening
        motor_L_encoder.write(0);
        while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 3))
            follow_gyro();
        
        do {
            if(millis() - last_srf_trigger_ms > 50){
                last_srf_trigger_ms = millis();
                last_srf_FR_echo_us = srf_FR.ping();
                Serial.print("Front right distance: ");
                Serial.println(srf_FR.convert_cm(last_srf_FR_echo_us));
            }
            follow_gyro();
        } while(srf_FR.convert_cm(last_srf_FR_echo_us) < 20);
        Serial.print("Opening ");
        Serial.println(i);
    }
    //turn facing WNW victim location
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+45);
    ST.turn(0);
    ST.drive(16);
    motor_R_encoder.write(0);
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    ST.drive(10);
    ST.turn(10);
    gyro_angle(angle+45);
    
    
    //lower arm, open grabber
    grabber_servo.write(GRABBER_OPEN);
    arm_servo.write(ARM_DOWN);
    
    //go forward until either victim in grabber, or encoders exceed limit
    motor_R_encoder.write(0);
    ST.turn(0);
    ST.drive(20);
    bool is_WNW_victim_present;
    encoder_compensate_initialize();
    while(true){
        if(motor_R_encoder.read() > (MOTOR_COUNTS_PER_REVOLUTION * 7) / 2) {
            is_WNW_victim_present = false;
            break;
        }
        if(photogate_average() < PHOTOGATE_LOW) {
            is_WNW_victim_present = true;
            break;
        }
        if(follow_srf(srf_FL,srf_L,false,8)){
            encoder_compensate_sample();
        }
    }
    if(is_WNW_victim_present){
        Serial.println("WNW victim");
        //pick up victim and return (may need to move some of this to after the else case)
        while(photogate_average() < PHOTOGATE_HIGH);
        ST.stop();
        grabber_servo.write(GRABBER_CLOSE);
        delay(500);
        arm_servo.write(ARM_UP);
        delay(300);
        ST.drive(-25);
        //encoder_compensate_initialize();
        do{
            while(!follow_srf(srf_FL,srf_L,true,8));
            encoder_compensate_sample();
        } while(srf_L.convert_cm(last_srf_L_echo_us) < 25);
        encoder_compensate_apply(true);
        //go specified encoder distance backwards
        motor_L_encoder.write(0);
        gyro_PID_setpoint = angle;
        ST.drive(-25);
        while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 7) / 2){
            follow_gyro();
        }
        //face toward L2-L3 wall
        Serial.println("Face L2-L3");
        gyro_PID_setpoint = angle;
        ST.drive(0);
        ST.turn(-10);
        gyro_angle(90);
        
    }
    else{
        Serial.println("NNW victim");
        //get NNW victim
        
        //raise arm to make turn to move to next location
        ST.stop();
        arm_servo.write(ARM_UP);
        delay(500);
        
        ST.turn(0);
        ST.drive(30);
        do {
            if(millis() - last_srf_trigger_ms > 50){
                last_srf_trigger_ms = millis();
                last_srf_F_echo_us = srf_F.ping_cm();
                Serial.println(last_srf_F_echo_us);
            }
        } while (last_srf_F_echo_us >= 6);
        ST.stop();
        
        gyro_PID_setpoint = angle;
        
        ST.stop();
        ST.drive(0);
        ST.turn(16);
        gyro_angle(angle+130); //angle+135 puts at 45 degrees to run parallel with the river
        ST.stop();
        
        //Drive forward relying on the gyro to keep parallel with the river
        ST.drive(30);
        ST.turn(0);
        motor_R_encoder.write(0);
        while(motor_R_encoder.read() < MOTOR_COUNTS_PER_REVOLUTION * 6);  //drive forward 5 rotations measure what the approximate distance is
        
        //Swing turn left after passing the river
        ST.drive(5);
        ST.turn(-16);
        gyro_angle(angle-120);
        
        //Follow wall to turn at NW corner of field
        ST.drive(30);
        motor_R_encoder.write(0);
        do{
            follow_srf(srf_FR,srf_R,false,7);
        } while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 4) );
        ST.stop();
        
        //Turn left to face NNW victim
        ST.stop();
        gyro_PID_setpoint = angle;
        ST.drive(0);
        ST.turn(-10);
        gyro_angle(angle-80);
        
        //lower arm
        arm_servo.write(ARM_DOWN);
        //open grabber
        grabber_servo.write(GRABBER_OPEN);
        //delay(500);
        ST.drive(20);
        motor_R_encoder.write(0);
        while(photogate_average() > PHOTOGATE_LOW){
            follow_srf(srf_FR,srf_R,false,7);// its moving foward and the minimum distance is 7cm
        }
        while(photogate_average() < PHOTOGATE_HIGH);
        
        //stop
        ST.stop();
        //close grabber
        grabber_servo.write(GRABBER_CLOSE);
        //wait for grabber to close
        delay(500);
        //raise arm
        arm_servo.write(ARM_UP);
        delay(300);

//Reverse to North Wall
        ST.drive(-20);
        while(rear_average() < PROXIMITY_THRESHOLD){
            follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
        }

//Turn facing West Wall
        ST.drive(0);
        ST.turn(-20);
        gyro_angle(angle+80);
        ST.stop();
        delay(500);
    } //End of else NNW statement

    ST.stop();
}

void return_offroad(){
	Serial.println("return_offroad()");
    //go forward
    gyro_PID_setpoint = angle;
    ST.turn(0);
    ST.drive(25);
    do {
        if(millis() - last_srf_trigger_ms > 50){
            last_srf_trigger_ms = millis();
            last_srf_F_echo_us = srf_F.ping_cm();
            Serial.println(last_srf_F_echo_us);
        }
        follow_gyro();
    } while (last_srf_F_echo_us > 5);
    
    //turn away from wall facing W
    angle = 0;
    ST.turn(10);
    ST.drive(0);
    gyro_angle(90);
    Serial.println("Follow L2-L3");
    ST.turn(0);
    ST.drive(20);
    encoder_compensate_initialize();
    do {
        while(!follow_srf(srf_FL,srf_L,false,7)); // its moving forward and the minimum distance is 7cm
        encoder_compensate_sample();
    } while (srf_FL.convert_cm(last_srf_FL_echo_us) < 30);
    ST.stop();
    encoder_compensate_apply(true);
    
    Serial.println("Go to opening for L2");
    //L2_W_to_L2_S();
    /*
     //swing turn forward left
     ST.drive(10);
     ST.turn(-10);
     gyro_angle(angle-45);
     //go forward 1/6 turn
     ST.drive(10);
     ST.turn(0);
     motor_R_encoder.write(0);
     while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
     //complete swing turn forward left
     ST.drive(10);
     ST.turn(-10);
     gyro_angle(angle-45);
     */
    /*
     find_opening(srf_L, 20);
     ST.drive(0);
     ST.turn(-10);
     gyro_angle(angle-90);
     */
    angle = 0;
    //swing turn forward right
    ST.drive(10);
    ST.turn(10);
    gyro_angle(45);
    //go forward 1/6 turn
    ST.drive(16);
    ST.turn(0);
    motor_R_encoder.write(0);
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    //complete swing turn forward right
    ST.drive(10);
    ST.turn(10);
    gyro_angle(90);
    //swing turn backwards left
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(135);
    //go backwards 1/6 turn
    ST.drive(-10);
    ST.turn(0);
    motor_L_encoder.write(0);
    while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6));
    //swing turn backwards right
    ST.drive(-10);
    ST.turn(-10);
    gyro_angle(90);
    ST.stop();
}

void L3_to_L2(){
    //backup to L1-L2 wall
    ST.turn(0);
    ST.drive(-16);
    angle = 90;
    gyro_PID_setpoint = 90;
    while(rear_average() < PROXIMITY_THRESHOLD){
        follow_gyro();
    }
    
    //turn facing Y dropoff
    ST.turn(-10);
    ST.drive(0);
    gyro_angle(0);
    
    //approach Y dropoff
    ST.turn(0);
    ST.drive(30);
    //go forward until L1-L2 opening
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));// its moving forward and the minimum distance is 7cm
        
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 30); //need enough room to drop arm
    
    ST.stop();
}