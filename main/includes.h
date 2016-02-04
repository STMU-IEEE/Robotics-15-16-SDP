/********************     Libraries      *******************/
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
#include "Encoder.h"            //for quadrature encoders on 

/*********************     Pin assignments for Arduino Mega     ************************/
//0 reserved for Serial RX --> USB
//1 reserved for Serial TX --> USB
#define MOTOR_L_ENCODER_A   2
#define MOTOR_L_ENCODER_B   3
//4-5 open
#define SRF_F_ECHO      6
#define SRF_F_TRIGGER   7
//8 open
#define GRABBER_PIN     9    //servo 1 on Adafruit motor shield
#define ARM_PIN         10    //servo 2 on Adafruit motor shield
#define SRF_L_ECHO      11
#define SRF_L_TRIGGER   11
#define SRF_R_ECHO      12
#define SRF_R_TRIGGER   12
#define COLOR_LED_PIN   13    //to avoid blinding people; same as onboard LED
//14-15 open
//16 reserved for Serial2 TX --> Sabertooth S1
//17 reserved for Serial2 RX
#define MOTOR_R_ENCODER_A   18
#define MOTOR_R_ENCODER_B   19
//20 reserved for SDA
//21 reserved for SCL
#define MC_SHUTOFF_PIN  22    //active low Sabertooth shutoff (S2)
//23-53 open 
//pins 54 to 69 correspond to pins A0 to A15 on Arduino Mega
#define PHOTOGATE_PIN   A3    //pin for photogate (analog)
//A4 shorted to SDA by Adafruit motor shield
//A5 shorted to SCL by Adafruit motor shield

/************************     Servos      ************************/
//servo angles from testing
#define ARM_UP          28
#define ARM_DOWN        97
#define GRABBER_OPEN    140
#define GRABBER_CLOSE   75
 
/*************************   Photogate     *********************/
//15-bit thresholds with hysteresis
//(readings are roughly 2200 to 20000)
#define PHOTOGATE_LOW   6000
#define PHOTOGATE_HIGH  12000


/*************************   Gyro sensor     *********************/
//uncomment to enable noise rejection
//#define GYRO_NOISE_THRESHOLD

/********************    Ultrasonic sensors   *******************/


/*******************     Color sensor    ************************/
//8-bit known hues measured for each victim
#define YELLOW_HUE 55
#define RED_HUE 0

//symbolic type for each victim
enum victim_color: int8_t {
  red,
  yellow
};


/*************************   Sabertooth     *********************/
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

/*********************** Motor encoders ***************************/
//encoder counts per revolution for 50:1 motor (Pololu #2824)
// 50 * 64 = 3200
#define MOTOR_COUNTS_PER_REVOLUTION 3200

