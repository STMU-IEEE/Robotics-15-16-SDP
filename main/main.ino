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
NewPing srf_FR = NewPing(SRF_FR_TRIGGER, SRF_FR_ECHO);
NewPing srf_FL = NewPing(SRF_FL_TRIGGER, SRF_FL_ECHO);

//use to wait 50ms between readings; update using millis()
unsigned long last_SRF_trigger = 0;

//keep last reading (in microseconds) available for use globally
unsigned int last_SRF_L_echo;
unsigned int last_SRF_R_echo;
unsigned int last_SRF_F_echo;
unsigned int last_SRF_FL_echo;
unsigned int last_SRF_FR_echo;


//motor quadrature encoders
//positive counting == clockwise rotation
Encoder motor_L_encoder(MOTOR_L_ENCODER_A, MOTOR_L_ENCODER_B);
Encoder motor_R_encoder(MOTOR_R_ENCODER_A, MOTOR_R_ENCODER_B);

//tables for interpolation
const int IR_TABLE_SIZE = 14;
int ir_distances_cm[IR_TABLE_SIZE] =	{150,140,130,120,110,100, 90, 80, 70, 60, 50, 40, 30, 20}; 


void setup() {
  // put your setup code here, to run once:

  //shutoff Sabertooth motors until stop commands are sent
  //(this still takes about 2 seconds from reset to happen)
  pinMode(ST_SHUTOFF_PIN,OUTPUT);
  digitalWrite(ST_SHUTOFF_PIN,LOW); //shutoff is active low

  //configure stop pin
  pinMode(STOP_PIN, INPUT_PULLUP);
  enableInterrupt(STOP_PIN, ISR_STOP, FALLING);
  //configure go pin
  pinMode(GO_PIN, INPUT_PULLUP);
 
   
  //set serial baud
  Serial.begin(115200);
  STSerial.begin(38400); //problems communicating regardless of baud rate?
  
  //initialize motor controller baud rate
  Serial.println("Initializing Sabertooth...");
  ST.autobaud(true);
  
  //stop
  Serial.println("Stopping motors...");
  ST.stop();

  //Sabertooth can be re-enabled
  digitalWrite(ST_SHUTOFF_PIN,HIGH); 
  
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

  
	//data ready pin as input
	pinMode(GYRO_DRDY_PIN,INPUT);

  //constrain turning power to safer values:
  int turn_range = 16;
  gyroPID.SetOutputLimits(-turn_range/2, turn_range/2);
  
  //assume PID is computed for every gyro reading
  gyroPID.SetSampleTime((int)(1000/SAMPLE_RATE)); //in ms
}

void loop() {

	robot_game();
	//srfTest();
	
	//wallFollower(srf_FL,srf_L);
	//testMC();
	/*
	int i, sum = 0;
	for(i = 0; i < 100; i++)
		sum += srfOffset();
	int averageOffset = sum / i;
	Serial.print("Average offset: ");
	Serial.print(averageOffset);
	Serial.println(" uS");
	*/
	//while(digitalRead(STOP_PIN) != LOW)
	//while(true)
		//srfTest();
	//depart_from_Y();
}

void robot_game() {
	Serial.println("Press GO to continue");
	while(digitalRead(GO_PIN) != LOW);
	
	robot_setup();
	
  	leaveStartingArea();
  	L1_to_L2();
  	
	victim_color E_city = get_E_city();
	if(E_city == yellow){
		dropoff_E_city_Y();
		depart_from_Y_1();
	}
	else{ //red
		L2_to_L1();
		dropoff_R(); // will be part of usual dropoff red victim case
		L1_to_L2();
		L2_E_to_L2_S_B();
			//should now be facing wall between lane 1 and 2 by W opening to lane 3 
      
	}
	//victim_color W_city = get_W_city();
	get_W_city();
	if(E_city == yellow){//i.e. W_city == red
		L2_W_to_L2_S();
		L2_to_L1();
		dropoff_R();
		L1_to_L2();
		L2_E_to_L2_N();
	}
	else{
		dropoff_Y();
		depart_from_Y_2();
	}
	//get to east L2-L3 opening
	//depart_from_Y_2();
	//L2_E_to_L2_N();
	get_NE_victim();
	
} //end robot_game()


void robot_setup(){
	gyroCalibrate();
	find_actual_baud();
	//Servos
	Serial.println("Attaching servos...");
	grabber_servo.attach(GRABBER_PIN);
	grabber_servo.write(GRABBER_MIN);
	arm_servo.attach(ARM_PIN);
	arm_servo.write(ARM_UP);
}

//Interrupt functions for STOP buttons 
void ISR_STOP() {
	//disable Sabertooth
	digitalWrite(ST_SHUTOFF_PIN, LOW);
	//detach servos
	grabber_servo.detach();
	arm_servo.detach();
	//wait for GO to be pressed
	while(digitalRead(GO_PIN) != LOW);
	//enable Sabertooth
	digitalWrite(ST_SHUTOFF_PIN, HIGH);
	//reattach servos
	grabber_servo.attach(GRABBER_PIN);
	arm_servo.attach(ARM_PIN);
}



