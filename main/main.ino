#include "includes.h"

//servos
Servo grabber_servo, arm_servo;

//Gyro and PID global variables
L3G gyro;

//for gyro calibration routine
const int sample_num = 1000;
int16_t dc_offset = 0;
float noise = 0;

//const float SAMPLE_RATE = 189.4F; //gyro update rate in Hz from datasheet
const float SAMPLE_RATE = 183.3F; //measured gyro rate--may be able to measure during calibration

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

//bits for gyro registers (cf. datasheet)
const byte INT2_DRDY       =     1 << 3;    //CTRL3(INT2_DRDY)
const byte ZYXDA           =     1 << 3;    //STATUS(ZYXDA), aka XYZDA; data ready flag

//Gyro PID controller
PID gyro_PID(&gyro_PID_input, &gyro_PID_output, &gyro_PID_setpoint,
            gyro_PID_Kp, gyro_PID_Ki, gyro_PID_Kd,
            DIRECT); // change in output corresponds to same-sign change in input

//color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_4X);

//use separate serial port for Sabertooth (TX only; RX unused)
HardwareSerial& ST_Serial = Serial2;
//Sabertooth 2x25 v1.02 motor controller
Sabertooth ST(ST_ADDR, ST_Serial);

//ultrasonic range finders
NewPing srf_L = NewPing(SRF_L_TRIGGER, SRF_L_ECHO);
NewPing srf_R = NewPing(SRF_R_TRIGGER, SRF_R_ECHO);
NewPing srf_F = NewPing(SRF_F_TRIGGER, SRF_F_ECHO);
NewPing srf_FR = NewPing(SRF_FR_TRIGGER, SRF_FR_ECHO);
NewPing srf_FL = NewPing(SRF_FL_TRIGGER, SRF_FL_ECHO);

//use to wait 50ms between readings; update using millis()
unsigned long last_srf_trigger_ms = 0;

//keep last reading (in microseconds) available for use globally
unsigned int last_srf_L_echo_us;
unsigned int last_srf_R_echo_us;
unsigned int last_srf_F_echo_us;
unsigned int last_srf_FL_echo_us;
unsigned int last_srf_FR_echo_us;


//motor quadrature encoders
//positive counting == clockwise rotation
Encoder motor_L_encoder(MOTOR_L_ENCODER_A, MOTOR_L_ENCODER_B);
Encoder motor_R_encoder(MOTOR_R_ENCODER_A, MOTOR_R_ENCODER_B);

//encoder-assisted compensation: used to calculate average encoder difference
int32_t encoder_compensate_sum; //sum of samples
int encoder_compensate_n; //number of samples

//tables for interpolation
const int IR_TABLE_SIZE = 14;
int ir_left_raw_table[IR_TABLE_SIZE] =	{ 92,102,108,116,128,140,153,173,193,223,265,331,419,539};
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
    ST_Serial.begin(38400); //problems communicating regardless of baud rate?
    
    //initialize motor controller baud rate
    Serial.println("Initializing Sabertooth...");
    ST.autobaud(true);
    
    //stop regardless of actual baud rate
    Serial.println("Stopping motors...");
    long options[] = {2400,9600,19200,38400};
    for (int i = 0; i < 4; i++){
        ST_Serial.begin(options[i]);
        ST.stop();
        ST_Serial.flush();
    }
    
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
    gyro_PID.SetOutputLimits(-turn_range/2, turn_range/2);
    
    //assume PID is computed for every gyro reading
    gyro_PID.SetSampleTime((int)(1000/SAMPLE_RATE)); //in ms
}

void loop() {
    robot_game();
}

void robot_game() {
    Serial.println("Press GO to continue");
    while(digitalRead(GO_PIN) != LOW);
    
    robot_setup();
    
    leave_starting_area();
    
    //face E city victim
    L1_to_L2();
    
    //retrieve and get color of E city victim
    victim_color E_city = get_E_city();
    
    victim_color W_city; //will determine using color of E city victim
    
    //drop off E city victim and get to common point for W city victim
    if(E_city == yellow){
        W_city = red;
        dropoff_E_city_Y();
        depart_from_Y_1();
    }
    else{ //E_city == red
        W_city = yellow;
        back_into_Y_then_face_L1();
        L2_to_L1();
        dropoff_R(); // will be part of usual dropoff red victim case
        depart_from_R_dropoff();
        L1_to_L2();
        L2_E_to_L2_S_B();
        //should now be facing wall between lane 1 and 2 by W opening to lane 3
        
    }
    //victim_color W_city = get_W_city();
    get_W_city();
    
    L3_to_L2();
    //drop off W city victim--get to common point later
    if(W_city == red){
        L2_W_to_L1_E();
        dropoff_R();
        depart_from_R_dropoff();
        L1_to_L2();
    }
    else{//W_city == yellow
        dropoff_Y();
    }
    
    //get to common point for E offroad victim
    if(W_city == red){
        L2_E_to_L2_N();
    }
    else{//W_city == yellow
        depart_from_Y_2();
    }
    
    victim_color E_offroad = get_E_offroad();
    return_offroad();
    L3_to_L2();
    
    victim_color W_offroad; //will determine using color of E offroad victim
    
    //drop off E offroad victim--get to common point for W offroad victim later
    if(E_offroad == yellow){
        W_offroad = red;
        dropoff_Y();
    }
    else{ //E_offroad == red
        W_offroad = yellow;
        L2_W_to_L1_E();
        dropoff_R();
        depart_from_R_dropoff();
        L1_to_L2();
    }
    
    //get to common point for W offroad victim
    if(E_offroad == red){
        L2_E_to_L2_S_B();
    }
    else{//E_offroad == yellow
        depart_from_Y_1();
    }
    
    get_W_offroad();
    return_offroad();
    L3_to_L2();
    if(W_offroad == yellow) {
        dropoff_Y();
        Y_dropoff_to_start();
    }
    else {//W_offroad == red
        L2_W_to_L1_E();
        dropoff_R();
        R_dropoff_to_start();
    }
} //end robot_game()


void robot_setup(){
    gyro_calibrate();
    find_actual_baud();
    
    //Servos
    Serial.println("Attaching servos...");
    grabber_servo.attach(GRABBER_PIN);
    grabber_servo.write(GRABBER_MIN);
    arm_servo.attach(ARM_PIN);
    arm_servo.write(ARM_UP);
    
    //initialize PID
    angle = 0;             //start with angle 0
    gyro_PID_setpoint = 0; //keep angle at 0
    gyro_PID_output = 0; //start without turning
    
    //start PID
    gyro_PID.SetMode(AUTOMATIC);
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



