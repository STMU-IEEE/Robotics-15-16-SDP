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
            last_srf_F_echo_us = srf_F.ping();
            Serial.print("Front distance: ");
            Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
        }
        follow_gyro();
    } while (srf_F.convert_cm(last_srf_F_echo_us) >= 4);
    
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
	approach_victim();
    ST.drive(40); //can't make too fast (e.g. 60)--will get stuck against wall
    while(photogate_average() > PHOTOGATE_LOW){
        follow_srf(srf_FR,srf_R,false,7);// its moving foward and the minimum distance is 7cm
    }
    unsigned long photogate_blocked_ms = millis(); //wait up to 1s for photogate to become unblocked
    while((photogate_average() < PHOTOGATE_HIGH) && (millis() - photogate_blocked_ms < 1000));
        
    ST.stop();
    
    pick_up_victim();
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
        Serial.print("Front distance: ");
        Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
        
    } while (srf_F.convert_cm(last_srf_F_echo_us) >= 20); //need enough room to drop arm
    
    ST.stop();
    drop_victim();
    ST.turn(0);
    ST.drive(-60);
    motor_L_encoder.write(0);
    //follow S wall for 6 rotations
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while (motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 6)){
        follow_srf(srf_FR,srf_R,true,14);
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
    }
    	
    //slow down before finding opening
    ST.drive(-30);
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
    ST.drive(-40);
    while(rear_average() < proximity_threshold){
        follow_srf(srf_FR,srf_R,true,14);
    }
    ST.stop();
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
    
    drop_victim();
    
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
    
    motor_L_encoder.write(0);
    //keep going to L2-L3 center wall
    do {
        while(!follow_srf(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_R_echo_us = srf_R.ping();
        Serial.println(srf_R.convert_cm(last_srf_R_echo_us));
        encoder_compensate_sample();
    } while ((srf_R.convert_cm(last_srf_R_echo_us) > 30) || (motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 4))); //find L2-L3 center wall
    
    //go until 2nd opening to lane 3
    do {
        while(!follow_srf(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_R_echo_us = srf_R.ping();
        Serial.println(srf_R.convert_cm(last_srf_R_echo_us));
        encoder_compensate_sample();
    } while ((srf_R.convert_cm(last_srf_R_echo_us) < 36) || (motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 4))); //find L2-L3 E opening
    
    //slow down
    ST.turn(0);
    ST.drive(-20);
    motor_L_encoder.write(0);
    //go until last L2-L3 wall
    do {
        while(!follow_srf(srf_FL,srf_L,true,7));// its moving backwards and the minimum distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_R_echo_us = srf_R.ping();
        Serial.println(srf_R.convert_cm(last_srf_R_echo_us));
        encoder_compensate_sample();
    } while ((srf_R.convert_cm(last_srf_R_echo_us) > 30) || (motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 4))); //find L2-L3 E wall
    
    //go back toward opening slightly
    ST.turn(0);
    ST.drive(16);
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write;
    
    //swing turn facing lane 3
    //encoder_compensate_apply(true);
    ST.drive(10);
    ST.turn(10);
    gyro_angle(angle+45);
    ST.turn(0);
    ST.drive(16);
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write;
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
            last_srf_F_echo_us = srf_F.ping();
            Serial.print("Front distance: ");
            Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
        }
        follow_gyro();
    } while (srf_F.convert_cm(last_srf_F_echo_us) < 20);
    
    //turn facing W city victim
    ST.turn(10);
    ST.drive(-10);
    gyro_angle(angle+90);
    
    //go until wall on right
    ST.turn(0);
    ST.drive(40);
    motor_L_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    gyro_PID_setpoint = angle;
    while( -motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 3)/2){
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        follow_gyro();
    }
    
    //follow wall on left until victim
    approach_victim();
    
    while(photogate_average() > PHOTOGATE_LOW){
        follow_srf(srf_FL,srf_L,false,8);// its moving foward and the minimum distance is 7cm
    }
    unsigned long photogate_blocked_ms = millis(); //wait up to 1s for photogate to become unblocked
    while((photogate_average() < PHOTOGATE_HIGH) && (millis() - photogate_blocked_ms < 1000));
        
    //stop
    ST.stop();
    
    pick_up_victim();
    
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
        Serial.print("Front distance: ");
        Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
        
    } while (srf_F.convert_cm(last_srf_F_echo_us) >= 17); //need enough room to drop arm
    
    ST.stop();
    
    drop_victim();
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
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 4))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
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
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
    
    //complete turn
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+45);
    
    //go backwards into S wall
    gyro_PID_setpoint = angle;
    ST.turn(0);
    ST.drive(-35);
    while(rear_average() < proximity_threshold){
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
    //make sure to be inside of Y dropoff
    ST.drive(20);
    ST.turn(0);
    do {
        while(!follow_srf(srf_FR,srf_R,false,7));//moving backwards, target distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) > 25); //find inside of Y dropoff
    
    //find opening on left while following wall on right
    ST.drive(-20);
    ST.turn(0);
    do {
        while(!follow_srf(srf_FR,srf_R,true,7));//moving backwards, target distance is 7cm
        while(millis() - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = millis();
        last_srf_L_echo_us = srf_L.ping();
        Serial.println(srf_L.convert_cm(last_srf_L_echo_us));
        
    } while (srf_L.convert_cm(last_srf_L_echo_us) < 36); //find L1-L2 opening
    
    //turn into lane 1 using swing turn
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(angle+90);
    
    //go backwards into S wall
    gyro_PID_setpoint = angle;
    ST.turn(0);
    ST.drive(-45);
    while(rear_average() < proximity_threshold){
        follow_gyro();
    }
    
    //swing turn toward R dropoff
    ST.drive(10);
    ST.turn(10);
    gyro_angle(angle+90);
    
    //go backwards into starting area
    ST.turn(0);
    ST.drive(-45);
    while(rear_average() < proximity_threshold){
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
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    ST.turn(0);
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
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
            last_srf_F_echo_us = srf_F.ping();
            Serial.print("Front distance: ");
            Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
        }
        follow_gyro();
    } while (srf_F.convert_cm(last_srf_F_echo_us) >= 6);
    
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
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
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
}


victim_color get_E_offroad(){
	Serial.println("get_E_offroad()");
    victim_color result; //to be updated and returned at end of function
    
    //back into L1-L2 for reference
    ST.turn(0);
    ST.drive(-20);
    while(rear_average() < proximity_threshold);
    
	//look for walls on right (as done for W offroad)
	ST.turn(0);
	ST.drive(25);
	angle = 0;
	gyro_PID_setpoint = 0;
	for(int i = 1; i <= 2; i++){
        //go 1/3 turn before detecting next wall
        motor_R_encoder.write(0);
        digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
        while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 3)){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
            follow_gyro();
        }
        
        do {
            if(millis() - last_srf_trigger_ms > 50){
                last_srf_trigger_ms = millis();
                last_srf_R_echo_us = srf_R.ping();
                Serial.print("Right distance: ");
                Serial.println(srf_R.convert_cm(last_srf_R_echo_us));
            }
            follow_gyro();
        } while(srf_R.convert_cm(last_srf_R_echo_us) > 16);
        Serial.print("Wall ");
        Serial.println(i);
        
        //go 1/3 turn before detecting next opening
        motor_R_encoder.write(0);
        digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
        while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 3)){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
            follow_gyro();
        }
        
        do {
            if(millis() - last_srf_trigger_ms > 50){
                last_srf_trigger_ms = millis();
                last_srf_R_echo_us = srf_R.ping();
                Serial.print("Right distance: ");
                Serial.println(srf_R.convert_cm(last_srf_R_echo_us));
            }
            follow_gyro();
        } while(srf_R.convert_cm(last_srf_R_echo_us) < 25);
        Serial.print("Opening ");
        Serial.println(i);
    }
    
    //maneuver around L3 wall and ENE obstacle
    //point turn left 135 degrees
    ST.drive(0);
    ST.turn(-16);
    gyro_angle(-135);
    
    //move backwards 2/3 rotations.
    ST.turn(0);
    ST.drive(-20);
    motor_L_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    gyro_PID_setpoint = -135;
    while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 2) / 3){
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        follow_gyro();
    }
    
    //point turn right 45 degrees
    ST.drive(0);
    ST.turn(16);
    gyro_angle(-90);
    
    //back into E wall
    ST.drive(-25);
    ST.turn(0);
    gyro_PID_setpoint = -90;
    while(rear_average() < proximity_threshold){
        follow_gyro();
    }
    
    //go forward 1/3 turn
    ST.drive(20);
    ST.turn(0);
    gyro_PID_setpoint = -90;
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 3)){
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        follow_gyro();
    }
    
    //point turn right 90 degrees
    ST.drive(0);
    ST.turn(16);
    gyro_angle(0);
    
    //back into L3 wall
    ST.drive(-25);
    ST.turn(0);
    gyro_PID_setpoint = 0;
    while(rear_average() < proximity_threshold){
        follow_gyro();
    }
    
    //its going to put the grabber down, follow the wall and its going to detect if NE victim is present
    ST.stop();
    approach_victim();
    ST.drive(40);
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    
    bool is_ENE_victim_present;
    //its going to follow the wall a certain distance or it will detect the NE victim
    
    while(true){
        follow_srf(srf_FR,srf_R,false,9);// its moving foward and the minimum distance is 9cm
        if(photogate_average() < PHOTOGATE_LOW){
            is_ENE_victim_present=true;
            break;
        }
        if(motor_R_encoder.read() > (MOTOR_COUNTS_PER_REVOLUTION * 7)/2){
            is_ENE_victim_present=false;
            break;
        }
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
    }
    // it we have the NE victim, then wait until its in the correct position and get it.
    //pick up victim
    if(is_ENE_victim_present){
        unsigned long photogate_blocked_ms = millis(); //wait up to 1s for photogate to become unblocked
        while((photogate_average() < PHOTOGATE_HIGH) && (millis() - photogate_blocked_ms < 1000));
        ST.stop();
        pick_up_victim();
        result = get_color();
        ST.stop();
        
        follow_E_wall();
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
        
        //back into N wall
        ST.turn(0);
        ST.drive(-30);
        while(rear_average() < proximity_threshold){
            follow_srf(srf_FL,srf_L,true,7);// its moving backwards and the minimum distance is 7cm
        }
        ST.stop();
        
        //go forward from N wall
        ST.drive(20);
        motor_R_encoder.write(0);
        digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
        gyro_PID_setpoint = angle;
        while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 1) / 4){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
            follow_gyro();
        }
        
        //point turn right facing NNE victim
        ST.drive(0);
        ST.turn(15);
        gyro_angle(angle+90);
        ST.stop();
        
        //back into E wall
        ST.turn(0);
        ST.drive(-20);
        while(rear_average() < proximity_threshold){
            follow_srf(srf_FR,srf_R,true,6);// its moving backwards and the minimum distance is 7cm
        }
        
        //its going to put the grabber down, follow the wall and its going to detect if NE victim is present
        ST.stop();
        
        approach_victim();
        delay(500);
        
        ST.drive(30); //can't make too fast (e.g. 60)--will get stuck against wall
        while(photogate_average() > PHOTOGATE_LOW){
            follow_srf(srf_FR,srf_R,false,6);// its moving foward and the minimum distance is 7cm
        }
        unsigned long photogate_blocked_ms = millis(); //wait up to 1s for photogate to become unblocked
        while((photogate_average() < PHOTOGATE_HIGH) && (millis() - photogate_blocked_ms < 1000));
        
        //stop
        ST.stop();
        
        
        pick_up_victim();
        result = get_color();
        
        follow_N_wall();
    }
    ST.stop();
    return result;
}

void follow_N_wall(){
	//start following along N wall toward NE corner
    ST.drive(-45);
    while(rear_average() < proximity_threshold){
        follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
    }
    
    //go forward from E wall
    ST.turn(0);
    ST.drive(20);
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    gyro_PID_setpoint = angle;
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 1) / 4){
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        follow_gyro();
    }
    ST.stop();
    
    //turn facing S
    ST.drive(0);
    ST.turn(-20);
    gyro_angle(angle-90);
    ST.stop();
    
    //go forward S for 2 rotations
    ST.drive(40);
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 2)){
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        follow_srf(srf_FL,srf_L,false,7);// its moving foward and the minimum distance is 7cm
    }
    
    //turn around
    ST.drive(0);
    ST.turn(25);
    gyro_angle(angle+180);
    
    follow_E_wall();
}

void follow_E_wall(){
	Serial.println("follow_E_wall()");
    //back up until L3 wall
    ST.turn(0);
    ST.drive(-40);
    while(rear_average() < proximity_threshold){
        follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
    }
    
    //go forward 1/4 turn
    ST.drive(20);
    ST.turn(0);
    angle = 0;
    gyro_PID_setpoint = 0;
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 4)){
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        if(follow_gyro()){
            Serial.print("R Encoder: ");
            Serial.println(motor_R_encoder.read());
            Serial.flush();
        }
    }
    
    //point turn left facing W
    ST.drive(0);
    ST.turn(-20);
    gyro_angle(-90);
    ST.stop();
    
    //go forward W until L2-L3 on left
    ST.drive(35);
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    gyro_PID_setpoint = -90;
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 7)/3){
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        follow_gyro();
    }
    
    //face S to L3-L2
    ST.drive(0);
    ST.turn(-20);
    gyro_angle(-180);
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
        digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
        while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 3)){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
            follow_gyro();
        }
        
        do {
            if(millis() - last_srf_trigger_ms > 50){
                last_srf_trigger_ms = millis();
                last_srf_FR_echo_us = srf_FR.ping();
                Serial.print("Front right distance: ");
                Serial.println(srf_FR.convert_cm(last_srf_FR_echo_us));
            }
            follow_gyro();
        } while(srf_FR.convert_cm(last_srf_FR_echo_us) > 15);
        Serial.print("Wall ");
        Serial.println(i);
        
        //go 1/3 turn before detecting next opening
        motor_L_encoder.write(0);
        digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
        while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 3)){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
            follow_gyro();
        }
        
        do {
            if(millis() - last_srf_trigger_ms > 50){
                last_srf_trigger_ms = millis();
                last_srf_FR_echo_us = srf_FR.ping();
                Serial.print("Front right distance: ");
                Serial.println(srf_FR.convert_cm(last_srf_FR_echo_us));
            }
            follow_gyro();
        } while(srf_FR.convert_cm(last_srf_FR_echo_us) < 40);
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
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 3))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
    ST.drive(0);
    ST.turn(20);
    gyro_angle(angle+45);

    approach_victim();
    
    //go forward until either victim in grabber, or encoders exceed limit
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    ST.turn(0);
    ST.drive(30);
    bool is_WNW_victim_present;
    //encoder_compensate_initialize();
    while(true){
        if(motor_R_encoder.read() > (MOTOR_COUNTS_PER_REVOLUTION * 7) / 2) {
            is_WNW_victim_present = false;
            break;
        }
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        if(photogate_average() < PHOTOGATE_LOW) {
            is_WNW_victim_present = true;
            break;
        }
        if(follow_srf(srf_FL,srf_L,false,9)){
            //encoder_compensate_sample();
        }
    }
    if(is_WNW_victim_present){
        Serial.println("WNW victim");
        //pick up victim and return (may need to move some of this to after the else case)
        unsigned long photogate_blocked_ms = millis(); //wait up to 1s for photogate to become unblocked
        while((photogate_average() < PHOTOGATE_HIGH) && (millis() - photogate_blocked_ms < 1000));
        ST.stop();
        pick_up_victim();
        delay(300);
        //encoder_compensate_initialize();
        ST.drive(-25);
        do{
            while(!follow_srf(srf_FL,srf_L,true,9));
            //encoder_compensate_sample();
        } while(srf_L.convert_cm(last_srf_L_echo_us) < 25);
        //encoder_compensate_apply(true);
        //go specified encoder distance backwards
        motor_L_encoder.write(0);
        digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
        angle = 0;
        gyro_PID_setpoint = angle;
        ST.drive(-25);
        while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 7) / 2){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
            follow_gyro();
        }
        //face toward L2-L3 wall
        Serial.println("Face L2-L3");
        ST.drive(0);
        ST.turn(-10);
        gyro_angle(-90);
        
    }
    else{
        Serial.println("NNW victim");
        //get NNW victim
        //raise arm to make turn to move to next location
        ST.stop();
        arm_servo.write(ARM_UP);
        delay(500);
        
        ST.turn(0);
        ST.drive(25);
        gyro_PID_setpoint = angle;
        do {
            follow_gyro();
            if(millis() - last_srf_trigger_ms > 50){
                last_srf_trigger_ms = millis();
                last_srf_F_echo_us = srf_F.ping();
                Serial.print("Front distance: ");
                Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
            }
        } while (srf_F.convert_cm(last_srf_F_echo_us) >= 6);
        ST.stop();
        
        //encoder_compensate_apply(true);
        
        angle = 0;
        gyro_PID_setpoint = angle;
        
        ST.stop();
        ST.drive(0);
        ST.turn(16);
        gyro_angle(132); //angle+135 puts at 45 degrees to run parallel with the river (use <135 to avoid W fixed obstacle)
        ST.stop();
        
        //Drive forward relying on the gyro to keep parallel with the river
        ST.drive(40);
        ST.turn(0);
        motor_R_encoder.write(0);
        digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
        gyro_PID_setpoint = 132;
        while(motor_R_encoder.read() < MOTOR_COUNTS_PER_REVOLUTION * 6){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
        	follow_gyro();
        }  //drive forward 5 rotations measure what the approximate distance is
        
        //turn facing S after passing the river
        ST.drive(0);
        ST.turn(16);
        gyro_angle(270);
        
        gyro_PID_setpoint = 270;
        ST.turn(0);
        ST.drive(-20);
        //back up to N wall
        while(rear_average() < proximity_threshold){
        	follow_gyro();
        }
        //swing turn facing E
        ST.turn(-16);
        ST.drive(16);
        gyro_angle(angle-90);
        
        //Follow wall to turn at NW corner of field
        ST.turn(0);
        ST.drive(-30);
        motor_R_encoder.write(0);
        do{
            follow_srf(srf_FL,srf_L,true,9);
        } while(rear_average() < proximity_threshold);
        ST.stop();
        
        //go forward before turning
        ST.turn(0);
        ST.drive(20);
        motor_R_encoder.write(0);
        digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
        gyro_PID_setpoint = angle;
        while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 1) / 4){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
            follow_gyro();
        }
        //Turn right to face NNW victim
        ST.stop();
        gyro_PID_setpoint = angle;
        ST.drive(0);
        ST.turn(16);
        gyro_angle(angle+90);
        
        //Reverse to North Wall
        ST.turn(0);
        ST.drive(-20);
        while(rear_average() < proximity_threshold){
            follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
        }       
        
        approach_victim();
        
        ST.drive(20);
        motor_R_encoder.write(0);
        while(photogate_average() > PHOTOGATE_LOW){
            follow_srf(srf_FR,srf_R,false,7);// its moving foward and the minimum distance is 7cm
        }
        unsigned long photogate_blocked_ms = millis(); //wait up to 1s for photogate to become unblocked
        while((photogate_average() < PHOTOGATE_HIGH) && (millis() - photogate_blocked_ms < 1000));
        
        ST.stop();
        pick_up_victim();

//Reverse to North Wall
        ST.drive(-35);
        while(rear_average() < proximity_threshold){
            follow_srf(srf_FR,srf_R,true,7);// its moving backwards and the minimum distance is 7cm
        }
        //swing turn to face E
        ST.turn(-16);
        ST.drive(16);
        gyro_angle(angle-90);
        
        //go forward along N wall until safe to turn around
        ST.turn(0);
		ST.drive(45);
		motor_R_encoder.write(0);
		digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
		while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION * 4)){
            digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
			follow_srf(srf_FL,srf_L,false,7);// its moving foward and the minimum distance is 7cm
		}
		
		ST.drive(0);
		ST.turn(25);
		gyro_angle(angle+180);
		
		follow_N_wall();
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
            last_srf_F_echo_us = srf_F.ping();
            Serial.print("Front distance: ");
            Serial.println(srf_F.convert_cm(last_srf_F_echo_us));
        }
        follow_gyro();
    } while (srf_F.convert_cm(last_srf_F_echo_us) > 8);
    
    //swing turn toward from wall facing W
    angle = 0;
    ST.turn(10);
    ST.drive(10);
    gyro_angle(90);
    
    Serial.println("Follow L2-L3");
    ST.turn(0);
    ST.drive(20);
    encoder_compensate_initialize();
    do {
        while(!follow_srf(srf_FL,srf_L,false,9)); // its moving forward and the minimum distance is 9cm
        encoder_compensate_sample();
    } while (srf_FL.convert_cm(last_srf_FL_echo_us) < 30);
    ST.stop();
    encoder_compensate_apply(true);
    
    Serial.println("Go to opening for L2");
    angle = 0;
    //swing turn forward right
    ST.drive(10);
    ST.turn(10);
    gyro_angle(45);
    //go forward 1/6 turn
    ST.drive(16);
    ST.turn(0);
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
    //complete swing turn forward right
    ST.drive(10);
    ST.turn(10);
    gyro_angle(90);
    //swing turn backwards left
    ST.drive(-10);
    ST.turn(10);
    gyro_angle(135);
    //go backwards 1/6 turn
    ST.drive(-16);
    ST.turn(0);
    motor_L_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_L_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 6))
        digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
    //swing turn backwards right
    ST.drive(-10);
    ST.turn(-10);
    gyro_angle(90);
    ST.stop();
}

void L3_to_L2(){
    //backup to L1-L2 wall
    ST.turn(0);
    ST.drive(-35);
    angle = 90;
    gyro_PID_setpoint = 90;
    while(rear_average() < proximity_threshold){
        follow_gyro();
    }
    
    //go far enough from wall for safe swing turn
    ST.drive(20);
    ST.turn(0);
    motor_R_encoder.write(0);
    digitalWrite(COLOR_LED_PIN,HIGH); //debug encoder write
    while(motor_R_encoder.read() < ((MOTOR_COUNTS_PER_REVOLUTION * 3)/ 4))
    	digitalWrite(COLOR_LED_PIN,LOW); //debug encoder write
    
    //turn facing Y dropoff
    ST.turn(-10);
    ST.drive(-10);
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
