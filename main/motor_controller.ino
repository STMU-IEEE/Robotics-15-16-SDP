void find_actual_baud(){
    long options[] = {2400,9600,19200,38400};
    int32_t starting_position = motor_L_encoder.read();
    int32_t new_position;
    int i;
    for(i = 0; i < 4; i++){
        ST_Serial.flush();
        ST_Serial.begin(options[i]);
        ST.drive(20);
        ST.turn(0);
        delay(200);
        ST.stop();
        new_position = motor_L_encoder.read();
        if(abs(new_position - starting_position) > 50){
            Serial.print("Actual baud is ");
            Serial.println(options[i]);
            return;
        }
    }
    Serial.println("Did not move");
}

void test_ST()
{
    ST.drive(25);
    delay(1000);
    ST.drive(-25);
    delay(1000);
    ST.drive(0);
    ST.turn(25);
    delay(1000);
    ST.turn(-25);
    delay(1000);
    ST.stop();
}

void test_swing(){
    ST.drive(16);
    ST.turn(16);
    delay(2000);
    ST.stop();
}

