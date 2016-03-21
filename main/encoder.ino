//Continuously print encoder position--move wheels manually to change readings
void encoder_read_test() {
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
void encoder_distance_test() {
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
        if(follow_gyro())
            Serial.println(gyro_PID_output);
    
    //stop
    ST.stop();
    
    //stop PID
    gyro_PID.SetMode(MANUAL);
    
    Serial.println("Final positions:");
    Serial.print("L\t");
    Serial.println(motor_L_encoder.read());
    Serial.print("R\t");
    Serial.println(motor_R_encoder.read());
}

/* Encoder-assisted compensation
 * e.g. for wall following: attempt to track average encoder difference
 * restore it once opening is found to prevent crashing into wall
 * **Assumes exclusive use of encoders**
 * Use regularly-timed samples: e.g. when wall-following, _don't_ use
 
 do {
 followSRFs(...);
 encoder_compensate_sample(); //will sample too often--unusable result
 } while(...);
 
 * Replace with:
 
 do {
 while(!followSRFs(...)); //wait for regularly-timed update before proceeding
 encoder_compensate_sample();
 } while(...);
 
 */

//setup global variables for use with compensation
void encoder_compensate_initialize() {
    encoder_compensate_sum = 0;
    encoder_compensate_n = 0;
    motor_R_encoder.write(0);
    motor_L_encoder.write(0);
}

//sample the encoder difference
void encoder_compensate_sample() {
    int32_t sample_difference = motor_L_encoder.read() + motor_R_encoder.read(); //add, because the encoders count opposite from each other
    encoder_compensate_n++;
    Serial.print("Difference ");
    Serial.print(encoder_compensate_n);
    Serial.print(" : ");
    Serial.println(sample_difference);
    encoder_compensate_sum += sample_difference;
}

/* apply the average encoder difference
 * use_motor_R: if true, swing turns using right motor
 *              if false, swing turns using left motor */
void encoder_compensate_apply(bool use_motor_R) {
    Serial.print("Average difference: ");
    int32_t average_difference = encoder_compensate_sum / encoder_compensate_n;
    Serial.println(average_difference);
    //swing turn about motor to correct difference
    if(use_motor_R) {
        motor_R_encoder.write(average_difference);
        if(average_difference > 0){
            ST.drive(-10);
            ST.turn(10);
            while(motor_R_encoder.read() > 0);
        }
        else {
            ST.drive(10);
            ST.turn(-10);
            while(motor_R_encoder.read() <= 0);
        }
    }
    else {
        motor_L_encoder.write(average_difference);
        if(average_difference > 0){
            ST.drive(10);
            ST.turn(10);
            while(motor_L_encoder.read() > 0);
        }
        else {
            ST.drive(-10);
            ST.turn(-10);
            while(motor_L_encoder.read() <= 0);
        }
    }
    ST.stop();
}