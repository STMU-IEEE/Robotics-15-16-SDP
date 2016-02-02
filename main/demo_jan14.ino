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
  mcWrite(MC_FORWARD,straight_speed);
  while(photogateAverage() > PHOTOGATE_LOW);
  while(photogateAverage() < PHOTOGATE_HIGH);
  //stop
  mcWrite(MC_FORWARD, 0);
  mcWrite(MC_LEFT,0);
  
  //close grabber
  grabber_servo.write(GRABBER_CLOSE);
  //wait for grabber to close
  delay(500);
  //raise arm
  arm_servo.write(ARM_UP);
  digitalWrite(COLOR_LED_PIN, HIGH);
  delay(1000);
  //print hue
  Serial.println(readHue());

  mcWrite(MC_BACKWARDS,straight_speed);
  delay(1000);
  mcWrite(MC_FORWARD, 0); //stop turning
  mcWrite(MC_LEFT,0);
  //spin right 90 degrees
  mcWrite(MC_RIGHT, turn_speed);
  //make 1 full right turn
  gyroAngle(360);
  mcWrite(MC_FORWARD, 0); //stop turning
  mcWrite(MC_LEFT,0);

  //drop victim
  delay(2000);
  arm_servo.write(ARM_DOWN);
  delay(500);
  grabber_servo.write(GRABBER_OPEN);
  delay(500);
  arm_servo.write(ARM_UP);
  
}

