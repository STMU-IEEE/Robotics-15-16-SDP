#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"  //color sensor
//#include "Adafruit_L3GD20.h"    //gyro sensor; may also try L3G by Pololu
//Adafruit library outputs as scaled floats
//Use Pololu library which outputs unscaled int16_t instead:
#include "L3G.h"                //gyro sensor; import from L3G folder at https://github.com/pololu/l3g-arduino
#include "FastLED.h"            //for rgb2hsv_approximate()
#include "NewPing.h"            //for ultrasonic range finders; import from NewPing_v1.7.zip
#include "PID_v1.h"             //import from https://github.com/br3ttb/Arduino-PID-Library/

//servos
#define GRABBER_PIN     5
#define ARM_PIN         6
Servo grabber_servo, arm_servo;

/*********servo angles from testing*********/
#define ARM_UP          28
#define ARM_DOWN        97
#define GRABBER_OPEN    140
#define GRABBER_CLOSE   75

//Gyro and PID global variables
#define GYRO_DRDY_PIN   12                  //INT2/data ready pin on L3GD20H (level shifted using Alamode by connecting to RPi GPIO MISO/header pin 21)
#define GYRO_DRDY_INT_PIN 2                 //should be manually jumpered to drive interrupt (if need to reclaim pin 11, use external level shifter)
L3G gyro;

unsigned long lastDebugTime = 0; //debug

//uncomment to enable noise rejection
//#define GYRO_NOISE_THRESHOLD

//for gyro calibration routine
const int sampleNum = 1000;
int16_t dc_offset = 0;
float noise = 0;

const float sampleRate = 189.4F; //gyro update rate in Hz
const float ADJUSTED_SENSITIVITY = 0.009388F; //empirically corrected sensitivity (for turn speed 16)
int16_t& gyro_robot_z = gyro.g.y; //robot's -z axis corresponds to gyro's +y (data is negated)
double rate = 0 ;
double prev_rate = 0;

double gyro_PID_output = 64; //initialize to 64 = stop
double angle = 0;
double& gyro_PID_input = angle; //angle is input to PID controller
double gyro_PID_setpoint = 0;
double gyro_PID_Kp = 1;
double gyro_PID_Ki = 1;
double gyro_PID_Kd = 1;

//bits for gyro registers (cf. datasheet):
const byte H_Lactive       =     1 << 5;    //CTRL3(H_Lactive)--does not affect DRDY (cf. application note AN4506 p. 22)
const byte INT2_DRDY       =     1 << 3;    //CTRL3(INT2_DRDY)
const byte INT2_Empty      =     1 << 0;    //CTRL3(INT2_Empty)
const byte FIFO_EN         =     1 << 6;    //CTRL5(FIFO_EN)
const byte FM_BYPASS_MODE  = 0b000 << 5;    //FIFO_CTRL(FM2:0) for bypass mode (FIFO off/reset)
const byte FM_STREAM_MODE  = 0b010 << 5;    //FIFO_CTRL(FM2:0) for stream mode

//Gyro PID controller
PID gyroPID(&gyro_PID_input, &gyro_PID_output, &gyro_PID_setpoint,
            gyro_PID_Kp, gyro_PID_Ki, gyro_PID_Kd,
            DIRECT); // change in output corresponds to same-sign change in input

//color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_4X);

//digital output for gating serial TX to motor controller
#define MC_GATE_PIN     14

//force Sabertooth stop using emergency shutoff (S2)
//note: shutoff is active low
#define MC_SHUTOFF_PIN  8

//enable value for motor controller (for NAND gate)
#define MC_ON           HIGH
#define MC_OFF          LOW

//Sabertooth address from DIP switches
#define MC_ADDR         128

//Sabertooth initialization value
#define MC_INIT_BYTE    170

//Sabertooth commands (mixed mode)
#define MC_FORWARD      8
#define MC_BACKWARDS    9
#define MC_RIGHT        10
#define MC_LEFT         11
#define MC_TURN_7BIT    13

//color sensor LED (turn off to avoid blinding people)
#define COLOR_LED_PIN   13
#define COLOR_LED_ON    HIGH
#define COLOR_LED_OFF   LOW

//pins for ultrasonic range finders
/* If we need to reduce pin usage, check if possible
 * to either share echo pins between sensors (need external OR), 
 * or use shared single trigger/echo pin mode per sensor*/
#define SRF_L_ECHO      3
#define SRF_L_TRIGGER   11
#define SRF_R_ECHO      9
#define SRF_R_TRIGGER   10
NewPing srf_L = NewPing(SRF_L_TRIGGER, SRF_L_ECHO);
NewPing srf_R = NewPing(SRF_R_TRIGGER, SRF_R_ECHO);

//pin for photogate (analog)
#define PHOTOGATE_PIN   3 //should this be A3 for clarity?

//15-bit thresholds with hysteresis
//(readings are roughly 2200 to 20000)
#define PHOTOGATE_LOW   6000
#define PHOTOGATE_HIGH  12000

void setup() {
  // put your setup code here, to run once:

  //shutoff Sabertooth motors until stop commands are sent
  //(this still takes about 2 seconds from reset to happen)
  pinMode(MC_SHUTOFF_PIN,OUTPUT);
  digitalWrite(MC_SHUTOFF_PIN,LOW); //shutoff is active low
  
  //set pin to gate serial TX to motor controller as output
  pinMode(MC_GATE_PIN,OUTPUT);
  //disable motor controller listening to serial TX
  digitalWrite(MC_GATE_PIN,MC_OFF);
  
  //set serial baud
  Serial.begin(38400);

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
  grabber_servo.write(GRABBER_OPEN);
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

  //data ready pin as input
  pinMode(GYRO_DRDY_PIN,INPUT);
  /* enable data ready DRDY line
   * as active-low interrupt pin
   * (cf. application note AN4506)*/
  gyro.writeReg(L3G::CTRL3, INT2_DRDY);  

  //set PID limits based on 0 = full left, 127 = full right, 64 = stop
  gyroPID.SetOutputLimits(0, 127);

  //assume PID is computed for every gyro reading -> 189.4Hz
  gyroPID.SetSampleTime((int)(1000/sampleRate)); //in ms

}

void loop() {

  Serial.println("Press g to continue");
  while(Serial.read() != 'g')
    ledBlink(2000);
  
  //perform robot behaviors
  robotMain();

  //turn off servos
  //arm_servo.detach();
  //grabber_servo.detach();
  
  //print photogate output to help identify threshold
  //Serial.println(photogateAverage());
  ledBlink(1000);
} //end loop()



void robotMain(){
  //place robot behaviors here
  ledBlink(500);
  ledBlink(500);

  /*demo:
   * drop grabber
   * go forward until victim detected
   * pick up and report color
   * turn around and drop victim
   */

   /*
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
  digitalWrite(COLOR_LED_PIN,COLOR_LED_ON);
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
  */
/*
  Serial.print("Enter power: ");
  while(Serial.available() < 2)
    ledBlink(1000);
  byte speed = Serial.parseInt();

  gyro_PID_setPoint = 64;
  //enable PID
  gyroPID.SetMode(AUTOMATIC);
*/
  //debug gyro sample rate
  lastDebugTime = millis();
  //enable gyro angle and PID updating when DRDY low
  attachInterrupt(digitalPinToInterrupt(GYRO_DRDY_INT_PIN), isr_DRDY, LOW);
  while(!Serial.available());
  detachInterrupt(digitalPinToInterrupt(GYRO_DRDY_INT_PIN));
/*
  unsigned long lastMillis = millis();
  byte newTurn = 64;
  while(true){ //infinite loop
  //go forward for 1s
  mcWrite(MC_FORWARD,speed);
  while((millis() - lastMillis()) < 1000))
    if(newTurn != (byte)gyro_PID_output){
      newTurn = (byte)gyro_PID_output;
      Serial.println(gyro_PID_output);
      mcWrite(MC_TURN_7BIT,newTurn);
    }

  //stop
  mcWrite(MC_FORWARD,0);
  mcWrite(MC_TURN_7BIT,64);
  delay(1000);

  //go forward for 1s
  mcWrite(MC_BACKWARDS,speed);
  while((millis() - lastMillis()) < 1000))
    if(newTurn != (byte)gyro_PID_output){
      newTurn = (byte)gyro_PID_output;
      Serial.println(gyro_PID_output);
      mcWrite(MC_TURN_7BIT,newTurn);
    }

  //stop
  mcWrite(MC_FORWARD,0);
  mcWrite(MC_TURN_7BIT,64);
  delay(1000);
  }
*/
}

//blink color sensor LED once
void ledBlink(unsigned long delay_ms) {
  digitalWrite(COLOR_LED_PIN,COLOR_LED_ON);
  delay(delay_ms/2);
  digitalWrite(COLOR_LED_PIN,COLOR_LED_OFF);
  delay(delay_ms/2);
}

/* not enough PWM outputs for this
//based on Fade LED example
void ledFade() {
  int brightness = 0;
  while(brightness < 255) {
    // set the brightness of pin 9:
    analogWrite(COLOR_LED_PIN, ++brightness);
    delay(6);
  }
  while(brightness > 0) {
    analogWrite(COLOR_LED_PIN, --brightness);
    delay(6);
  }
} */

// For motor controller, may consider SoftwareSerial instead of hardware TX + gate

void mcInit() {
  //wait for TX to finish before enabling output
  Serial.flush();
  //enable output to motor controller
  digitalWrite(MC_GATE_PIN,MC_ON);
  //send initialization byte (170)
  Serial.write(MC_INIT_BYTE);
  //wait for TX to finish before disabling output
  Serial.flush();
  //disable output to motor controller
  digitalWrite(MC_GATE_PIN,MC_OFF);
}

void mcWrite(byte cmd, byte data) {
  //compute checksum and place in byte array for transferring
  byte mc_cmd[4] = { MC_ADDR, cmd, data,
       (byte)((MC_ADDR + cmd + data) & 0b01111111) }; //checksum; use explicit cast to ignore -Wnarrowing
  //wait for TX to finish before enabling output
  Serial.flush();
  //enable output to motor controller
  digitalWrite(MC_GATE_PIN,MC_ON);
  //write command to motor controller
  Serial.write(mc_cmd,4);
  //wait for TX to finish before disabling output
  Serial.flush();
  //disable output to motor controller
  digitalWrite(MC_GATE_PIN,MC_OFF);
}

//Calculate the hue (color) detected
//Hypothesis is that this is more immune to changes in lighting than e.g. simply using red/green values.
//As written, there is room for optimization and improved accuracy--test first.
uint8_t readHue() {
  uint16_t tcs_r, tcs_g, tcs_b, tcs_c;  //red, green, blue, clear
  tcs.getRawData(&tcs_r,&tcs_g,&tcs_b,&tcs_c);
  CRGB tcs_rgb; //object from FastLED
  /*
  tcs_rgb.red = highByte(tcs_r);
  tcs_rgb.green = highByte(tcs_g);
  tcs_rgb.blue = highByte(tcs_b);*/
  //scale to 8-bit (only need relative precision for hue;
  // ignore saturation and value)
  Serial.println("Color sensor readings:");
  Serial.print("R:\t");
  Serial.println(tcs_r);
  Serial.print("G:\t");
  Serial.println(tcs_g);
  Serial.print("B:\t");
  Serial.println(tcs_b);
  while(max(max(tcs_r,tcs_g),tcs_b) > 255) {
    tcs_r >>= 1;
    tcs_g >>= 1;
    tcs_b >>= 1;
  }
  Serial.println("Color sensor (scaled to 8 bit):");
  Serial.print("R:\t");
  Serial.println(tcs_rgb.r = tcs_r);
  Serial.print("G:\t");
  Serial.println(tcs_rgb.g = tcs_g);
  Serial.print("B:\t");
  Serial.println(tcs_rgb.b = tcs_b);
  CHSV tcs_hsv = rgb2hsv_approximate(tcs_rgb); //convert CRGB object to CHSV
  Serial.print("Hue (8-bit):\t");
  return tcs_hsv.hue;
}

//Take sum of 32 readings (interpret as 15-bit average instead of 10-bit)
//(Why 32? maximimum readings that fit inside signed 16-bit, and still fast)
int photogateAverage() {
  int average = 0;
  for(int count = 0; count < 32; count++)
    average += analogRead(PHOTOGATE_PIN);
  return average;
}

/* Gyro calibration
   * based on GyroTest.ino example by G. C. Hill (2013, EE444 at CSULB) (must inquire terms of reuse)
   * from lab 5, p. 6: "8  Calibrate the Gyro"
   * 
   * Note: the time required for this initialization
   * might be cutting into time for our robot to start moving
   * Check if rules allow calibration before pressing "go" button
   * 
   * gyro -y axis corresponds to robot's +z axis
   */
void gyroCalibrate() {
  //"8.1  Measure Gyro Offset at Rest (Zero-rate Level)"
  Serial.print("Gyro DC Offset: ");
  int32_t dc_offset_sum = 0; //original type "int" overflows!
  for(int n = 0; n < sampleNum; n++){
    gyro.read();
    dc_offset_sum += gyro_robot_z;
    //Serial.println(dc_offset_sum);
  }
  dc_offset = dc_offset_sum / sampleNum;
  Serial.println(dc_offset);
  
  Serial.print("Gyro Noise Level: ");
  for(int n = 0; n < sampleNum; n++)
  {
    gyro.read();
    if((gyro_robot_z - dc_offset) > noise)
      noise = gyro_robot_z - dc_offset;
    else if((gyro_robot_z - dc_offset) < -noise)
      noise = -gyro_robot_z - dc_offset;
  }
  noise /= 100; //"gyro returns hundredths of degrees/sec"
  Serial.println(noise,4); //prints 4 decimal places
}

//wait until past target relative angle
//if turning clockwise, use false for is_counter_clockwise
//Based on "9  Measure Rotational Velocity" and "10  Measure Angle" (Hill 2013, p. 8)
void gyroAngle(float target) {
  //const int sampleTime = 10; //in ms
  //unsigned long time1 = millis(),time2; //same type as millis()
  bool is_counter_clockwise = (target > 0);
  Serial.println("gyroAngle");//debug
  //Wait for angle to cross target
  while((is_counter_clockwise && (angle < target)) ||     //increasing angle
         (!is_counter_clockwise && (angle > target)));    //decreasing angle
} //end gyroAngle

/* The following snippet might not work due to
 * incorrect order of operations:
 *   - dc_offset is scaled by (sampleNum - 1) then added 
 *      but never unscaled by (1 / sampleNum)
 *   - does (1 / sampleNum) == 0 because of integer division?
 *      (sampleNum is type int)
 *      if so: then (1 / sampleNum) * gyro.g.z == 0

void gyroRecalibrate() {
  //"11  Design Considerations" (Hill 2013, p. 10)
  //"Code to correct for gyro drift when rover is motionless"
  dc_offset = (sampleNum - 1) * dc_offset + (1 / sampleNum) * gyro_robot_z;
}
*/

//interrupt service handler for DRDY line from gyro
//to continuously update angle and PID
void isr_DRDY(){
  Serial.print("Reading...");
  gyro.read();
  Serial.print("done");

  //debug: see how often the gyro is updating
  unsigned long this_delay = millis() - lastDebugTime;
  lastDebugTime = millis();
  Serial.println((double)(1000/this_delay)); //in Hz
  
  /*
  rate = (float)(gyro_robot_z - dc_offset) * ADJUSTED_SENSITIVITY;
#ifdef  GYRO_NOISE_THRESHOLD
  //"11  Design Considerations" (p. 10)
  //"Ignore the gyro if our angular velocity does not meet our threshold"
  if(rate >= noise || rate <= -noise) 
    //will make angle += ... conditional
#endif
  //angle += ((prev_rate + rate) * ((float)sampleTime / 1000)) / 2; //as-is from p. 9: numerical integration using trapezoidal average of rates
  angle += ((prev_rate + rate) / sampleRate) / 2;                   //using output data rate specified by L3GD20H

  //"remember the current speed for the next loop rate integration."
  prev_rate = rate;
  
  gyroPID.Compute();
*/
}

