#include <Servo.h>

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
#define SERVO_ANGLE   16
//#define READ_COLOR   20

//servos
Servo my_servos[2];
#define GRABBER_PIN   9
#define ARM_PIN       10
int servo_pins[2] = {GRABBER_PIN, ARM_PIN};

void setup() {
  // put your setup code here, to run once:
  
  //set pin to gate serial TX to motor controller as output
  pinMode(MC_GATE,OUTPUT);
  //disable motor controller listening to serial TX
  digitalWrite(MC_GATE,MC_OFF);
  
  //set serial baud
  Serial.begin(9600);

  //TODO: see libraries for how to initialize ultrasonic range finders
  
  my_servos[0].attach(servo_pins[0]);
  my_servos[1].attach(servo_pins[1]);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available() > 0){
    switch(Serial.read()){
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
        //buffer for address, command, data, and checksum 
        byte mc_comm[4];
        //read address, command, and data
        Serial.readBytes(mc_comm,3);
        //compute checksum
        mc_comm[3] = (mc_comm[0]+mc_comm[1]+mc_comm[2]) & 0b01111111;
        //enable output to motor controller
        digitalWrite(MC_GATE,MC_ON);
        //write command to motor controller
        Serial.write(mc_comm,4);
        //wait for TX to finish
        Serial.flush();
        //disable output to motor controller
        digitalWrite(MC_GATE,MC_OFF);
        break;
      }
      case SERVO_ATTACH:
      {
        int idx = Serial.read();
        my_servos[idx].attach(servo_pins[idx]);
        break;
      }
      case SERVO_DETACH:
        my_servos[Serial.read()].detach();
        break;
      case SERVO_ANGLE:
      {
        int idx = Serial.read();
        my_servos[idx].write(Serial.read());
        break;
      }
    } //end switch(ser_comm)
  } //end while(Serial.available() > 0)
} //end loop()
