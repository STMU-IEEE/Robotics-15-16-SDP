//Uncomment which sensors to test
//#define TEST_L_SRF
//#define TEST_R_SRF
//#define TEST_F_SRF
//#define TEST_FR_SRF
//#define TEST_FL_SRF

void srfTest() {
    //static int8_t nextSensor = 0;
    unsigned long timeNow;
    
#ifdef TEST_L_SRF
    do
        timeNow = millis();
    while(timeNow - last_SRF_trigger < 50);
    last_SRF_trigger = timeNow;
    Serial.print("Left ping: ");
    Serial.print(srf_L.ping_cm());
    Serial.println("cm");
#endif
    
#ifdef TEST_R_SRF
    do
        timeNow = millis();
    while(timeNow - last_SRF_trigger < 50);
    last_SRF_trigger = timeNow;
    Serial.print("Right ping: ");
    Serial.print(srf_R.ping_cm());
    Serial.println(" cm");
#endif
    
#ifdef TEST_F_SRF
    do
        timeNow = millis();
    while(timeNow - last_SRF_trigger < 50);
    last_SRF_trigger = timeNow;
    Serial.print("Front ping: ");
    Serial.print(srf_F.ping_cm());
    Serial.println(" cm");
#endif
    
#ifdef TEST_FR_SRF
    do
        timeNow = millis();
    while(timeNow - last_SRF_trigger < 50);
    last_SRF_trigger = timeNow;
    Serial.print("Front right ping: ");
    Serial.print(srf_FR.ping_cm());
    Serial.println(" cm");
#endif
    
#ifdef TEST_FL_SRF
    do
        timeNow = millis();
    while(timeNow - last_SRF_trigger < 50);
    last_SRF_trigger = timeNow;
    Serial.print("Front left ping: ");
    Serial.print(srf_FL.ping_cm());
    Serial.println(" cm");
#endif
    
}

int srfOffset(){
    unsigned long timeNow;
    do
        timeNow = millis();
    while(timeNow - last_SRF_trigger < 50);
    last_SRF_trigger = timeNow;
    unsigned int uS_R = srf_R.ping();
    
    do
        timeNow = millis();
    while(timeNow - last_SRF_trigger < 50);
    last_SRF_trigger = timeNow;
    unsigned int uS_FR = srf_FR.ping();
    Serial.print("Offset: ");
    int result = (int)(uS_FR - uS_R);
    Serial.print(result);
    Serial.println(" uS");
    return result;
}

//follows wall for 10s forwards and backwards
void wallFollower(NewPing& srf_front, NewPing& srf_center){
    const int8_t drive_power = 35;
    const unsigned int test_distance = 13;
    ST.drive(drive_power);
    unsigned long start = millis();
    while(millis() - start < 10000)
        followSRFs(srf_front,srf_center,false,test_distance);
    ST.drive(-drive_power);
    start = millis();
    while(millis() - start < 10000)
        followSRFs(srf_front,srf_center,true,test_distance);
    ST.stop();
}


/*
 Put the robot centered with opening facing toward it.
 The robot will point turn left, and then measure the forward encoder count to the wall,
 the reverse to find the wall on other side of opening.
 The robot will report the encoder counts for both, and the average
 (which is the offset to use in findOpening).
 
 03/10/16: typical value for 12 inch opening is -582
 
 */
int32_t find_opening_offset_helper() {
    Serial.println("find opening offset helper");
    Serial.println("turning left...");
    angle = 0;
    ST.drive(0);
    ST.turn(-10);
    gyroAngle(-90);
    ST.stop();
    Serial.println("zeroing encoder...");
    motor_R_encoder.write(0);
    gyro_PID_setpoint = -90;
    Serial.print("front wall count: ");
    ST.drive(10);
    do {
        unsigned long timeNow = millis();
        if(timeNow - last_SRF_trigger >= 50){
            last_SRF_trigger = timeNow;
            last_SRF_R_echo = srf_R.ping();
        }
        followGyro();
    } while(srf_R.convert_cm(last_SRF_R_echo) > 30);
    ST.stop();
    int32_t front_wall_count = motor_R_encoder.read();
    Serial.println(front_wall_count);
    Serial.print("rear wall count: ");
    ST.drive(-10);
    do {
        unsigned long timeNow = millis();
        if(timeNow - last_SRF_trigger >= 50){
            last_SRF_trigger = timeNow;
            last_SRF_R_echo = srf_R.ping();
        }
        followGyro();
    } while((motor_R_encoder.read() > 0)
            || (srf_R.convert_cm(last_SRF_R_echo) > 30));
    ST.stop();
    int32_t rear_wall_count = motor_R_encoder.read();
    Serial.println(rear_wall_count);
    Serial.print("average count: ");
    int32_t average_count = (front_wall_count + rear_wall_count) / 2;
    Serial.println(average_count);
    return average_count;
}