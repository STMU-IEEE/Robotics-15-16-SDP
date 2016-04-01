//WARNING: not constrained to safe range
void servo_test(Servo &servo){
    Serial.print("Enter 3 digit angle: ");
    while(Serial.available() < 3);
    int16_t servo_angle = Serial.parseInt();
    Serial.println(servo_angle);
    //servo_angle = constrain(servo_angle);
    servo.write(servo_angle);
}

void approach_victim(){
    //lower arm
    arm_servo.write(ARM_DOWN);
    //open grabber
    grabber_servo.write(GRABBER_OPEN);
}

void pick_up_victim(){
    /* unused, grabber flipped upside down
    //lift arm slightly before closing
    arm_servo.write(ARM_ALMOST_DOWN);
    delay(500);
    //close grabber mostly
    grabber_servo.write(GRABBER_ALMOST_CLOSE);
    delay(500);
    //lower grabber slightly to straighten victim
    arm_servo.write(ARM_DOWN);
    delay(500);
    */
    //close grabber tightly
    grabber_servo.write(GRABBER_MIN);
    //wait for grabber to close
    delay(500);
    //raise arm
    arm_servo.write(ARM_UP);
    delay(500);
    //close grabber normally
    grabber_servo.write(GRABBER_CLOSE);
}

void drop_victim() {
    arm_servo.write(ARM_DOWN);
    delay(300);
    grabber_servo.write(GRABBER_OPEN);
    delay(300);
    arm_servo.write(ARM_UP);
    delay(300);
    grabber_servo.write(GRABBER_MIN);
}