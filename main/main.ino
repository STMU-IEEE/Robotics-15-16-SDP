#include "includes.h"

//servos
Servo grabber_servo, arm_servo;

//Gyro and PID global variables
L3G gyro;

//for gyro calibration routine
const int sampleNum = 1000;
int16_t dc_offset = 0;
float noise = 0;

//const float sampleRate = 189.4F; //gyro update rate in Hz from datasheet
const float SAMPLE_RATE = 183.3F; //measured gyro rate
//float sampleRate = 183.3F; //may be able to measure during calibration

//const float ADJUSTED_SENSITIVITY = 0.009388F; //empirically corrected sensitivity (for turn speed 16)
//const float ADJUSTED_SENSITIVITY = 0.0097F; //compensate for measured gyro
//const float ADJUSTED_SENSITIVITY = 0.0099F; //compensate for drift
const float ADJUSTED_SENSITIVITY = 0.008956528F; //compensated 02/04/16
int16_t& gyro_robot_z = gyro.g.y; //robot's -z axis corresponds to gyro's +y (data is negated)

double rate = 0;
double prev_rate = 0;

double gyro_PID_output = 64; //initialize to 64 = stop
double angle = 0;
double& gyro_PID_input = angle; //angle is input to PID controller
double gyro_PID_setpoint = 0;
double gyro_PID_Kp = 0.5;
double gyro_PID_Ki = 0;
double gyro_PID_Kd = 0;

//bits for gyro registers (cf. datasheet):
const byte H_Lactive       =     1 << 5;    //CTRL3(H_Lactive)--does not affect DRDY (cf. application note AN4506 p. 22)
const byte INT2_DRDY       =     1 << 3;    //CTRL3(INT2_DRDY)
const byte INT2_Empty      =     1 << 0;    //CTRL3(INT2_Empty)
const byte FIFO_EN         =     1 << 6;    //CTRL5(FIFO_EN)
const byte FM_BYPASS_MODE  = 0b000 << 5;    //FIFO_CTRL(FM2:0) for bypass mode (FIFO off/reset)
const byte FM_STREAM_MODE  = 0b010 << 5;    //FIFO_CTRL(FM2:0) for stream mode
const byte ZYXDA           =     1 << 3;    //STATUS(ZYXDA), aka XYZDA; data ready flag

//Gyro PID controller
PID gyroPID(&gyro_PID_input, &gyro_PID_output, &gyro_PID_setpoint,
            gyro_PID_Kp, gyro_PID_Ki, gyro_PID_Kd,
            DIRECT); // change in output corresponds to same-sign change in input

//color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_4X);

//use separate serial port for Sabertooth (RX only)
HardwareSerial& mcSerial = Serial2;

//ultrasonic range finders
NewPing srf_L = NewPing(SRF_L_TRIGGER, SRF_L_ECHO);
NewPing srf_R = NewPing(SRF_R_TRIGGER, SRF_R_ECHO);
NewPing srf_F = NewPing(SRF_F_TRIGGER, SRF_F_ECHO);

//motor quadrature encoders
//positive counting == clockwise rotation
Encoder motor_L_encoder(MOTOR_L_ENCODER_A, MOTOR_L_ENCODER_B);
Encoder motor_R_encoder(MOTOR_R_ENCODER_A, MOTOR_R_ENCODER_B);

void setup() {
  // put your setup code here, to run once:

  //shutoff Sabertooth motors until stop commands are sent
  //(this still takes about 2 seconds from reset to happen)
  pinMode(MC_SHUTOFF_PIN,OUTPUT);
  digitalWrite(MC_SHUTOFF_PIN,LOW); //shutoff is active low
  
  //set serial baud
  Serial.begin(115200);
  mcSerial.begin(2400);
  
  //wait 2s for Sabertooth to power up (p. 16)
  Serial.println("\nWaiting for Sabertooth to power up...");
  delay(2000);
  //initialize motor controller baud rate
  Serial.print("Initializing Sabertooth...");
  mcInit();
  //stop
  Serial.print("\nStopping motors...");
  mcWrite(MC_FORWARD,0);
  mcWrite(MC_LEFT,0);

  //Sabertooth can be re-enabled
  digitalWrite(MC_SHUTOFF_PIN,HIGH); 
  
  //Servos
  Serial.println("Attaching servos...");
  grabber_servo.attach(GRABBER_PIN);
  grabber_servo.write(GRABBER_MIN);
  arm_servo.attach(ARM_PIN);
  arm_servo.write(ARM_UP);
  
  //color sensor LED
  pinMode(COLOR_LED_PIN,OUTPUT);

  //initialize color sensor
  //based on Adafruit TCS3725 example code
  Serial.print("TCS34725 I2C color sensor");
  if(!tcs.begin())         //Adafruit_TCS34725.cpp v1.0.1 prints "44\r\n"
    Serial.print(" not");
  Serial.println(" found");
  
  //initialize gyro sensor
  //based on Adafruit L3GD20 example code:
  Serial.print("L3GD20H I2C gyro sensor");
  if (!gyro.init())
    Serial.print(" not");
  Serial.println(" found");
  gyro.enableDefault();

  gyroCalibrate();

  //set PID limits based on 0 = full left, 127 = full right, 64 = stop
  //gyroPID.SetOutputLimits(0, 127);

  //constrain to safer values:
  byte turn_range = 16;
  gyroPID.SetOutputLimits(64 - (turn_range/2), 64 + (turn_range/2));
  
  //assume PID is computed for every gyro reading
  gyroPID.SetSampleTime((int)(1000/SAMPLE_RATE)); //in ms

}

void loop() {

  Serial.println("Press g to continue");
  while(Serial.read() != 'g')
    ledBlink(2000);
  
  leaveStartingArea();
  //testGyroTurn();
  //testSwing();
  
} //end loop()

void testMC()
{
      mcWrite(MC_FORWARD,25);
      delay(1000);
      mcWrite(MC_BACKWARDS,25);
      delay(1000);
      mcWrite(MC_FORWARD,0);
      mcWrite(MC_RIGHT, 25);
      delay(1000);
      mcWrite(MC_LEFT, 25);
      delay(1000);
      mcWrite(MC_LEFT, 0);
}

void testSwing(){
  mcWrite(MC_FORWARD,16);
  mcWrite(MC_RIGHT, 16);
  delay(2000);
  mcWrite(MC_FORWARD,0);
  mcWrite(MC_RIGHT, 0);
}

//blink color sensor LED once
void ledBlink(unsigned long delay_ms) {
  digitalWrite(COLOR_LED_PIN, HIGH);
  delay(delay_ms/2);
  digitalWrite(COLOR_LED_PIN, LOW);
  delay(delay_ms/2);
}

void mcInit() {
  //wait until buffer empty
  //mcSerial.flush();
  //send initialization byte (170)
  mcSerial.write(MC_INIT_BYTE);
  //wait until buffer empty
  //mcSerial.flush();
}

void mcWrite(byte cmd, byte data) {
  //compute checksum and place in byte array for transferring
  byte mc_cmd[4] = { MC_ADDR, cmd, data,
       (byte)((MC_ADDR + cmd + data) & 0b01111111) }; //checksum; use explicit cast to ignore -Wnarrowing
  //wait until buffer empty
  //mcSerial.flush();
  //write command to motor controller
  mcSerial.write(mc_cmd,4);
  //wait until buffer empty
  //mcSerial.flush();
}

void leaveStartingArea() {
  Serial.println("Zeroing encoders...");
  motor_L_encoder.write(0);
  motor_R_encoder.write(0);

  //initialize PID
  angle = 0;             //start with angle 0
  gyro_PID_setpoint = 0; //keep angle at 0
  gyro_PID_output = 64; //start without turning
  
  //wait 50ms between readings
  unsigned long lastSRF = 0;

  //for debugging
  unsigned long srf_reading;
  
  //go forward
  mcWrite(MC_FORWARD, 25);
  mcWrite(MC_TURN_7BIT, 64);

  //start PID
  gyroPID.SetMode(AUTOMATIC);
  
  //wait until opening to lane 2 on left
  while(true){
    if(millis() - lastSRF > 50){
      lastSRF = millis();
      srf_reading = srf_L.ping_cm();
      Serial.println(srf_reading);
    }
    if(srf_reading > 36)
      break;
    followGyro();
  }

  //save encoder value for opening
  int32_t encoder_opening = motor_L_encoder.read();
  Serial.println("-----   -----");

  while(motor_L_encoder.read() > (encoder_opening - (MOTOR_COUNTS_PER_REVOLUTION / 2)))
    followGyro();
  
  //wait until opening to lane 2 on left 
  while(true){
    if(millis() - lastSRF > 50){
      lastSRF = millis();
      srf_reading = srf_L.ping_cm();
      Serial.println(srf_reading);
    }
    if(srf_reading < 25)
      break;
    followGyro();
  }
  
  //save encoder value for wall
  int32_t encoder_wall = motor_L_encoder.read();
  
  Serial.print("Opening:\t");
  Serial.println(encoder_opening);
  Serial.print("Wall:\t");
  Serial.println(encoder_wall);

  //reverse to middle of opening
  mcWrite(MC_BACKWARDS,25);
  while(motor_L_encoder.read() < ((encoder_opening + encoder_wall)/ 2))
    followGyro();

  Serial.print("Stop:\t");
  Serial.println(motor_L_encoder.read());
    
  //turn left 90 degrees
  mcWrite(MC_FORWARD,0);
  mcWrite(MC_LEFT,16);
  gyroAngle(-90);

  
  //go forward toward wall
  gyro_PID_setpoint = -90;
  mcWrite(MC_FORWARD, 20);
   while(true){
    if(millis() - lastSRF > 50){
      lastSRF = millis();
      srf_reading = srf_F.ping_cm();
      Serial.println(srf_reading);
    }
    if(srf_reading < 3)
      break;
    followGyro();
  }
  
  //turn right 45 degrees in place
  mcWrite(MC_FORWARD,0);
  mcWrite(MC_RIGHT,16);
  gyroAngle(-45);
  
  mcWrite(MC_FORWARD, 10);
  mcWrite(MC_RIGHT,10);
  gyroAngle(0);

  //now facing E city victim

  //go forward until wall on right
  gyro_PID_setpoint = 0;
  mcWrite(MC_FORWARD, 20);
  arm_servo.write(ARM_DOWN);
  grabber_servo.write(GRABBER_OPEN);
  while(photogateAverage() > PHOTOGATE_LOW){
    followGyro();
  }
  Serial.println("Approaching victim...");
  while(photogateAverage() < PHOTOGATE_HIGH){
    followGyro();
  }
  //stop
  mcWrite(MC_FORWARD,0);
  mcWrite(MC_LEFT,0);
  
  Serial.println("Grabbing victim...");
  grabber_servo.write(GRABBER_CLOSE);
  delay(500);
  arm_servo.write(ARM_UP);
  //stop PID
  gyroPID.SetMode(MANUAL);

  
}

