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

double gyro_PID_output = 0; //turning power: initialize to 0 = stop
double angle = 0;
double& gyro_PID_input = angle; //angle is input to PID controller
double gyro_PID_setpoint = 0;   //angle to keep
//tuning parameters
double gyro_PID_Kp = 1.0;
double gyro_PID_Ki = 0.0;
double gyro_PID_Kd = 0.0;

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
HardwareSerial& STSerial = Serial2;
//Sabertooth 2x25 v1.02 motor controller
Sabertooth ST(ST_ADDR, STSerial);

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
  pinMode(ST_SHUTOFF_PIN,OUTPUT);
  digitalWrite(ST_SHUTOFF_PIN,LOW); //shutoff is active low
  
  //set serial baud
  Serial.begin(115200);
  STSerial.begin(2400); //problems communicating at >2400bps?
  
  //wait 2s for Sabertooth to power up (p. 16)
  Serial.println("\nWaiting for Sabertooth to power up...");
  delay(2000);
  //initialize motor controller baud rate--already waited for startup
  Serial.print("Initializing Sabertooth...");
  ST.autobaud(false);
  
  //stop
  Serial.print("\nStopping motors...");
  ST.stop();

  //Sabertooth can be re-enabled
  digitalWrite(ST_SHUTOFF_PIN,HIGH); 
  
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

  //constrain turning power to safer values:
  byte turn_range = 16;
  gyroPID.SetOutputLimits(turn_range/2, turn_range/2);
  
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

//blink color sensor LED once
void ledBlink(unsigned long delay_ms) {
  digitalWrite(COLOR_LED_PIN, HIGH);
  delay(delay_ms/2);
  digitalWrite(COLOR_LED_PIN, LOW);
  delay(delay_ms/2);
}

void leaveStartingArea() {
  Serial.println("Zeroing encoders...");
  motor_L_encoder.write(0);
  motor_R_encoder.write(0);

  //initialize PID
  angle = 0;             //start with angle 0
  gyro_PID_setpoint = 0; //keep angle at 0
  gyro_PID_output = 0; //start without turning
  
  //wait 50ms between readings
  unsigned long lastSRF = 0;

  //for debugging
  unsigned long srf_reading;
  
  //go forward
  ST.drive(25);
  ST.turn(0);

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
  ST.drive(-25);
  while(motor_L_encoder.read() < ((encoder_opening + encoder_wall)/ 2))
    followGyro();

  Serial.print("Stop:\t");
  Serial.println(motor_L_encoder.read());
    
  //turn left 90 degrees
  ST.drive(0);
  ST.turn(-16);
  gyroAngle(-90);

  
  //go forward toward wall
  gyro_PID_setpoint = -90;
  ST.drive(20);
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
  ST.drive(0);
  ST.turn(16);
  gyroAngle(-45);
  
  ST.drive(10);
  ST.turn(10);
  gyroAngle(0);

  //now facing E city victim

  //go forward until wall on right
  gyro_PID_setpoint = 0;
  ST.turn(0);
  ST.drive(20);
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
  ST.stop();
  
  Serial.println("Grabbing victim...");
  grabber_servo.write(GRABBER_CLOSE);
  delay(500);
  arm_servo.write(ARM_UP);
  //stop PID
  gyroPID.SetMode(MANUAL);

  
}

