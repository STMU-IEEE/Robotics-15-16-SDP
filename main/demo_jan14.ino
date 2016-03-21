/*demo:
 * drop grabber
 * go forward until victim detected
 * pick up and report color
 * turn around and drop victim
 */

void demo_jan14() {
    
    byte straight_speed = 20;
    byte turn_speed = 16; //slow to minimize error
    
    //lower arm
    arm_servo.write(ARM_DOWN);
    //open grabber
    grabber_servo.write(GRABBER_OPEN);
    delay(500);
    //go forward until photo gate triggered
    ST.drive(straight_speed);
    while(photogate_average() > PHOTOGATE_LOW);
    while(photogate_average() < PHOTOGATE_HIGH);
    //stop
    ST.stop();
    
    //close grabber
    grabber_servo.write(GRABBER_CLOSE);
    //wait for grabber to close
    delay(500);
    //raise arm
    arm_servo.write(ARM_UP);
    digitalWrite(COLOR_LED_PIN, HIGH);
    delay(1000);
    //print hue
    Serial.println(read_hue());
    
    ST.drive(-straight_speed);
    delay(1000);
    ST.stop();
    //spin right 90 degrees
    ST.turn(turn_speed);
    //make 1 full right turn
    gyro_angle(360);
    ST.stop();
    
    //drop victim
    delay(2000);
    arm_servo.write(ARM_DOWN);
    delay(500);
    grabber_servo.write(GRABBER_OPEN);
    delay(500);
    arm_servo.write(ARM_UP);
    
}

