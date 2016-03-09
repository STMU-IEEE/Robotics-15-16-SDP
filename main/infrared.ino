//Based on sketch by Noah Stahl (2011) for Sharp GP2Y0A02YK0F
float read_ir_long_range_cm(int sensor_pin){
	int32_t avg_sensor_value = 0;
	int i;
	const int N = 160;
	for (i = 0; i < N; i++){
		avg_sensor_value += analogRead(sensor_pin);
		//delay(48);
	}
	//inches = 4192.936 * pow(((float)avg_sensor_value)/N,-0.935) - 3.937;
	float cm = 10650.08 * pow(((float)avg_sensor_value)/N,-0.935) - 10;
	return cm;
}

void ir_calibration_helper(int sensor_pin){
	const int N = 1024;
	for(int i = 0; i < IR_TABLE_SIZE; i++){
		Serial.print("Move to ");
		Serial.print(ir_distances_cm[i]);
		Serial.println(" and press GO");
		while(digitalRead(GO_PIN));
		Serial.read();
		int32_t sum = 0;
		for(int j = 0; j < N; j++)
			sum += analogRead(sensor_pin);
		Serial.print("Average: ");
		Serial.println(sum / N);
	}
}