#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"  //color sensor
#include "Adafruit_L3GD20.h"    //gyro sensor
#include "FastLED.h"            //for rgb2hsv_approximate()
#include "NewPing.h"            //for ultrasonic range finders; import from NewPing_v1.7.zip
//servos
#define GRABBER_PIN   5
#define ARM_PIN       6
Servo grabber_servo, arm_servo;

/*********servo angles from testing*********/
#define ARM_UP        22
#define ARM_DOWN      94
#define GRABBER_OPEN  140
#define GRABBER_CLOSE 72

Adafruit_L3GD20 gyro;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//digital output for gating serial TX to motor controller
#define MC_GATE       2
//enable value for motor controller (for NAND gate)
#define MC_ON         HIGH
#define MC_OFF        LOW

//Sabertooth address from DIP switches
#define MC_ADDR       128

//Sabertooth initialization value
#define MC_INIT_BYTE  170

//Sabertooth commands
#define MC_FORWARD    8
#define MC_BACKWARDS  9
#define MC_RIGHT      10
#define MC_LEFT       11

//color sensor LED
#define COLOR_LED_PIN 14
#define COLOR_LED_ON  HIGH
#define COLOR_LED_OFF LOW

//pins for ultrasonic range finders
#define SRF_L_ECHO    3
#define SRF_L_TRIGGER 11
#define SRF_R_ECHO    9
#define SRF_R_TRIGGER 10
NewPing srf_L = NewPing(SRF_L_TRIGGER, SRF_L_ECHO);
NewPing srf_R = NewPing(SRF_R_TRIGGER, SRF_R_ECHO);

//pin for photogate (analog)
#define PHOTOGATE_PIN 3

void setup() {
  // put your setup code here, to run once:
  
  //set pin to gate serial TX to motor controller as output
  pinMode(MC_GATE,OUTPUT);
  //disable motor controller listening to serial TX
  digitalWrite(MC_GATE,MC_OFF);
  
  //set serial baud
  Serial.begin(9600);

  //initialize motor controller baud rate
  mcInit();
  
  //TODO: see libraries for how to initialize ultrasonic range finders

  //Servos
  grabber_servo.attach(GRABBER_PIN);
  arm_servo.attach(ARM_PIN);
  
  //debug LED
  pinMode(COLOR_LED_PIN,OUTPUT);

  //perform robot behaviors
  robotMain();
}

void loop() {
  //robot is done, slow blink color sensor LED
  //ledFade();
  //print photogate output to help identify threshold
  Serial.println(analogRead(PHOTOGATE_PIN));
  //ledBlink(1000);
} //end loop()



void robotMain(){
  //place robot behaviors here
  ledBlink(500);
  ledBlink(500);
  //need to find threshold for photogate
  //lower arm
  arm_servo.write(ARM_DOWN);
  //open grabber
  grabber_servo.write(GRABBER_OPEN);
/*  //go forward until photo gate triggered
  while(analogRead(
  //close grabber
  grabber_servo.write(GRABBER_OPEN);
  //raise arm
  arm_servo.write(ARM_UP);
  //print hue
  Serial.print(readHue());
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

void mcInit() {
  //enable output to motor controller
  digitalWrite(MC_GATE,MC_ON);
  //send initialization byte (170)
  Serial.write(MC_INIT_BYTE);
  //wait for TX to finish before disabling output
  Serial.flush();
  //disable output to motor controller
  digitalWrite(MC_GATE,MC_OFF);
}

void mcWrite(byte cmd, byte data) {
  //compute checksum and place in byte array for transferring
  byte mc_cmd[4] = { MC_ADDR, cmd, data,
       (byte)((MC_ADDR + cmd + data) & 0b01111111) }; //checksum; use explicit cast to ignore -Wnarrowing
  //enable output to motor controller
  digitalWrite(MC_GATE,MC_ON);
  //write command to motor controller
  Serial.write(mc_cmd,4);
  //wait for TX to finish before disabling output
  Serial.flush();
  //disable output to motor controller
  digitalWrite(MC_GATE,MC_OFF);
}

//Calculate the hue (color) detected
//Hypothesis is that this is more immune to changes in lighting than e.g. simply using red/green values.
//As written, there is room for optimization and improved accuracy--test first.
uint8_t readHue() {
  uint16_t tcs_r, tcs_g, tcs_b, tcs_c;  //red, green, blue, clear
  tcs.getRawData(&tcs_r,&tcs_b,&tcs_g,&tcs_c);
  CRGB tcs_rgb;
  tcs_rgb.red = highByte(tcs_r);
  tcs_rgb.green = highByte(tcs_g);
  tcs_rgb.blue = highByte(tcs_b);
  CHSV tcs_hsv = rgb2hsv_approximate(tcs_rgb);
  return tcs_hsv.hue;
}

