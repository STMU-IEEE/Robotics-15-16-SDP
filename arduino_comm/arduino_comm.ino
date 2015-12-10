#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_L3GD20.h"

Adafruit_L3GD20 gyro;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//digital output for gating serial TX to motor controller
#define MC_GATE 2
//enable value for motor controller (for NAND gate)
#define MC_ON HIGH
#define MC_OFF LOW

//Sabertooth initialization value
#define MC_INIT_BYTE 170

//pins for ultrasonic range finders
//#define SRF_L 

//commands
#define MC_ECHO_INIT 170
#define MC_ECHO_COMM 171
#define SERVO_ATTACH  14
#define SERVO_DETACH  15
#define SERVO_WRITE   16
#define READ_COLOR    20
#define READ_GYRO     30

//servos
Servo my_servos[2];
#define GRABBER_PIN   9
#define ARM_PIN       10
int servo_pins[2] = {GRABBER_PIN, ARM_PIN};

//debug LED
#define DEBUG_LED 14
#define DEBUG_ON  HIGH
#define DEBUG_OFF LOW

void setup() {
  // put your setup code here, to run once:
  
  //set pin to gate serial TX to motor controller as output
  pinMode(MC_GATE,OUTPUT);
  //disable motor controller listening to serial TX
  digitalWrite(MC_GATE,MC_OFF);
  
  //set serial baud
  Serial.begin(9600);

  //TODO: see libraries for how to initialize ultrasonic range finders

  //debug LED
  pinMode(DEBUG_LED,OUTPUT);
}


/* possible to use if() instead of while(),
 *  or serialEvent() instead of loop()?
 */

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available() > 0){
    digitalWrite(DEBUG_LED,DEBUG_OFF);
    switch(Serial.read()){
      //New scopes {} used where local variables are declared
      case MC_ECHO_INIT:
        //enable output to motor controller
        digitalWrite(MC_GATE,MC_ON);
        //send initialization byte (170)
        Serial.write(MC_INIT_BYTE);
        //wait for TX to finish
        Serial.flush();
        //disable output to motor controller
        digitalWrite(MC_GATE,MC_OFF);
        break;
      case MC_ECHO_COMM:
      {
        /* possible optimization: echo individual bytes
         *  without waiting for whole command
         */
        
        //buffer for address, command, data, and checksum 
        byte mc_comm[4];
        //read address, command, and data
        while(Serial.available() < 3);
        Serial.readBytes(mc_comm,3);
        //compute checksum
        mc_comm[3] = (mc_comm[0]+mc_comm[1]+mc_comm[2]) & 0b01111111;
        //enable output to motor controller
        digitalWrite(MC_GATE,MC_ON);
        //write command to motor controller
        Serial.write(mc_comm,4);
        //wait for TX to finish before disabling output
        Serial.flush();
        //disable output to motor controller
        digitalWrite(MC_GATE,MC_OFF);
        break;
      }
      case SERVO_ATTACH:
      {
        while(Serial.available() < 1);
        int idx = Serial.read();
        my_servos[idx].attach(servo_pins[idx]);
        break;
      }
      case SERVO_DETACH:
        while(Serial.available() < 1);
        my_servos[Serial.read()].detach();
        break;
      case SERVO_WRITE:
      {
        while(Serial.available() < 2);
        int idx = Serial.read();
        int angle = Serial.read();
        my_servos[idx].write(angle);
        break;
      }
      case READ_COLOR:
      {
        uint16_t tcs_values[4]; //array for red, green, blue, clear
        tcs.getRawData(&tcs_values[0], &tcs_values[1], &tcs_values[2], &tcs_values[3]);
        //serialize array of values by accessing as byte array (big endian)
        Serial.write((byte*)tcs_values,4*sizeof(uint16_t));
        break;
      }
      case READ_GYRO:
      {
        gyro.read();
        float gyro_values[3] = {gyro.data.x, gyro.data.y, gyro.data.z}; //array for x, y, z
        Serial.write((byte*)gyro_values,3*sizeof(float));
        break;
      }
      default:
        digitalWrite(DEBUG_LED,DEBUG_ON);

    } //end switch(ser_comm)
  } //end while(Serial.available() > 0)
} //end loop()
