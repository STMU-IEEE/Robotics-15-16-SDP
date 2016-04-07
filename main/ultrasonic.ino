//Uncomment which sensors to test
//#define TEST_L_SRF
//#define TEST_R_SRF
//#define TEST_F_SRF
//#define TEST_FR_SRF
//#define TEST_FL_SRF

void srf_test() {
    //static int8_t nextSensor = 0;
    unsigned long time_now;
    
#ifdef TEST_L_SRF
    do
        time_now = millis();
    while(time_now - last_srf_trigger_ms < 50);
    last_srf_trigger_ms = time_now;
    Serial.print("Left ping: ");
    Serial.print(srf_L.ping_cm());
    Serial.println("cm");
#endif
    
#ifdef TEST_R_SRF
    do
        time_now = millis();
    while(time_now - last_srf_trigger_ms < 50);
    last_srf_trigger_ms = time_now;
    Serial.print("Right ping: ");
    Serial.print(srf_R.ping_cm());
    Serial.println(" cm");
#endif
    
#ifdef TEST_F_SRF
    do
        time_now = millis();
    while(time_now - last_srf_trigger_ms < 50);
    last_srf_trigger_ms = time_now;
    Serial.print("Front ping: ");
    Serial.print(srf_F.ping_cm());
    Serial.println(" cm");
#endif
    
#ifdef TEST_FR_SRF
    do
        time_now = millis();
    while(time_now - last_srf_trigger_ms < 50);
    last_srf_trigger_ms = time_now;
    Serial.print("Front right ping: ");
    Serial.print(srf_FR.ping_cm());
    Serial.println(" cm");
#endif
    
#ifdef TEST_FL_SRF
    do
        time_now = millis();
    while(time_now - last_srf_trigger_ms < 50);
    last_srf_trigger_ms = time_now;
    Serial.print("Front left ping: ");
    Serial.print(srf_FL.ping_cm());
    Serial.println(" cm");
#endif
    
}

int srf_offset(){
    unsigned long time_now;
    do
        time_now = millis();
    while(time_now - last_srf_trigger_ms < 50);
    last_srf_trigger_ms = time_now;
    last_srf_R_echo_us = srf_R.ping();
    
    do
        time_now = millis();
    while(time_now - last_srf_trigger_ms < 50);
    last_srf_trigger_ms = time_now;
    last_srf_FR_echo_us = srf_FR.ping();
    Serial.print("Offset: ");
    int result = (int)(last_srf_FR_echo_us - last_srf_R_echo_us);
    Serial.print(result);
    Serial.println(" uS");
    return result;
}

//follows wall for 10s forwards and backwards
void wall_follower(NewPing& srf_front, NewPing& srf_center){
    const int8_t drive_power = 35;
    const unsigned int test_distance = 13;
    ST.drive(drive_power);
    unsigned long start = millis();
    while(millis() - start < 10000)
        follow_srf(srf_front,srf_center,false,test_distance);
    ST.drive(-drive_power);
    start = millis();
    while(millis() - start < 10000)
        follow_srf(srf_front,srf_center,true,test_distance);
    ST.stop();
}


/*
 Put the robot centered with opening facing toward it.
 The robot will point turn left, and then measure the forward encoder count to the wall,
 the reverse to find the wall on other side of opening.
 The robot will report the encoder counts for both, and the average
 (which is the offset to use in find_opening).
 
 03/10/16: typical value for 12 inch opening is -582
 
 */
int32_t find_opening_offset_helper() {
    Serial.println("find opening offset helper");
    Serial.println("turning left...");
    angle = 0;
    ST.drive(0);
    ST.turn(-10);
    gyro_angle(-90);
    ST.stop();
    Serial.println("zeroing encoder...");
    motor_R_encoder.write(0);
    gyro_PID_setpoint = -90;
    Serial.print("front wall count: ");
    ST.drive(10);
    do {
        unsigned long time_now = millis();
        if(time_now - last_srf_trigger_ms >= 50){
            last_srf_trigger_ms = time_now;
            last_srf_R_echo_us = srf_R.ping();
        }
        follow_gyro();
    } while(srf_R.convert_cm(last_srf_R_echo_us) > 30);
    ST.stop();
    int32_t front_wall_count = motor_R_encoder.read();
    Serial.println(front_wall_count);
    Serial.print("rear wall count: ");
    ST.drive(-10);
    do {
        unsigned long time_now = millis();
        if(time_now - last_srf_trigger_ms >= 50){
            last_srf_trigger_ms = time_now;
            last_srf_R_echo_us = srf_R.ping();
        }
        follow_gyro();
    } while((motor_R_encoder.read() > 0)
            || (srf_R.convert_cm(last_srf_R_echo_us) > 30));
    ST.stop();
    int32_t rear_wall_count = motor_R_encoder.read();
    Serial.println(rear_wall_count);
    Serial.print("average count: ");
    int32_t average_count = (front_wall_count + rear_wall_count) / 2;
    Serial.println(average_count);
    return average_count;
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
bool follow_srf(NewPing& srf_front, NewPing& srf_center, bool is_driving_backwards, unsigned int target_distance){
    unsigned long time_now = millis();
    if(time_now - last_srf_trigger_ms >= 50){
        //depending on which sensor is given, turn the robot left or right
        //by changing sign of drive power
        int turn_power;
        unsigned int t2_offset;
        const int TURN_AMOUNT = 5; //adjust if needed
        unsigned int *last_SRF_front_echo, *last_SRF_center_echo;
        if      (&srf_center == &srf_L){
            turn_power = is_driving_backwards ? TURN_AMOUNT : -TURN_AMOUNT;
            t2_offset = T2_OFFSET_L;
            last_SRF_front_echo = &last_srf_FL_echo_us;
            last_SRF_center_echo = &last_srf_L_echo_us;
        }
        else if (&srf_center == &srf_R){
            turn_power = is_driving_backwards ? -TURN_AMOUNT : TURN_AMOUNT;
            t2_offset = T2_OFFSET_R;
            last_SRF_front_echo = &last_srf_FR_echo_us;
            last_SRF_center_echo = &last_srf_R_echo_us;
        }
        //else: not a valid srf???
        
        //take readings
        last_srf_trigger_ms = time_now;
        *last_SRF_center_echo = srf_center.ping();
        unsigned int t2 = *last_SRF_center_echo + t2_offset;
        do{
            time_now = millis();
        } while(time_now - last_srf_trigger_ms < 50);
        last_srf_trigger_ms = time_now;
        *last_SRF_front_echo = srf_front.ping();
        unsigned int& t1 = *last_SRF_front_echo;
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
        return true;
    }//end if(time_now - last_srf_trigger_ms >= 50)
    else
        return false;
}

void find_opening(NewPing& srf, int drive_speed){
    Serial.println("find_opening");
    unsigned long timeNow;
    unsigned long srf_reading;
    gyro_PID_setpoint = angle;
    
    //wait until opening
    Serial.println("Looking for opening...");
    ST.drive(drive_speed);
    do {
        timeNow = millis();
        if(timeNow - last_srf_trigger_ms >= 50){
            last_srf_trigger_ms = timeNow;
            srf_reading = srf.ping_cm();
            //Serial.println(srf_reading);
        }
        follow_gyro();
    } while(srf_reading <= 36);
    
    //save encoder value for opening
    Serial.print("L encoder position: ");
    int32_t encoder_opening = motor_L_encoder.read();
    Serial.println(encoder_opening);
    
    int32_t last_encoder_reading;
    Serial.println("Advancing 1/2 turn...");
    do {
        last_encoder_reading = motor_L_encoder.read();
        follow_gyro();
    } while(((drive_speed < 0) ? (last_encoder_reading - encoder_opening) //if going backwards
             : (encoder_opening - last_encoder_reading) //if going forward
             ) < (MOTOR_COUNTS_PER_REVOLUTION / 2));
    
    //wait until wall
    Serial.println("Looking for wall...");
    do {
        timeNow = millis();
        if(timeNow - last_srf_trigger_ms >= 50){
            last_srf_trigger_ms = timeNow;
            srf_reading = srf.ping_cm();
            //Serial.println(srf_reading);
        }
        follow_gyro();
    } while (srf_reading >= 25);
    
    //save encoder value for wall
    Serial.print("Wall encoder position: ");
    int32_t encoder_wall = motor_L_encoder.read();
    Serial.println(encoder_wall);
    
    
    //reverse to middle of opening
    Serial.println("Going to opening...");
    ST.drive(-drive_speed);
    do {
        last_encoder_reading = motor_L_encoder.read();
        follow_gyro();
    } while((drive_speed < 0) ? last_encoder_reading > ((encoder_opening + encoder_wall)/ 2)
            : last_encoder_reading < ((encoder_opening + encoder_wall)/ 2));
    
    ST.stop();
}


