//Take sum of 32 readings (interpret as 15-bit average instead of 10-bit)
//(Why 32? maximimum readings that fit inside signed 16-bit, and still fast)
int analog_average(int analog_pin) {
    int average = 0;
    for(int count = 0; count < 32; count++)
        average += analogRead(analog_pin);
    return average;
}

/*
 * for photogate built using 2 proximity sensors
 * use one as emitter; block emitter of receiver to only measure transmitted light
 */
int photogate_average() {
    return analog_average(PHOTOGATE_PIN);
}

int rear_average() {
	return analog_average(IR_REAR_PIN);
}

//place robot with back against wall
int find_rear_threshold() {
	int max_value = INT_MIN;
	int min_value = INT_MAX;
	ST.drive(20);
	ST.turn(0);
	motor_R_encoder.write(0);
	digitalWrite(COLOR_LED_PIN,HIGH); // debug encoder write
	while(motor_R_encoder.read() < (MOTOR_COUNTS_PER_REVOLUTION / 2)){
		digitalWrite(COLOR_LED_PIN,LOW); // debug encoder write
		int rear_sample = rear_average();
		max_value = max(max_value, rear_sample);
		min_value = min(min_value, rear_sample);
	}
	ST.stop();
	return 3 * (max_value / 5) + 2 * (min_value / 5); //return 60% threshold
}