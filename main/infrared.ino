//Based on sketch by Noah Stahl (2011) for Sharp GP2Y0A02YK0F
float read_ir_long_range_cm(){
	float sensorValue = analogRead(sensorIR);
	//inches = 4192.936 * pow(sensorValue,-0.935) - 3.937;
	float cm = 10650.08 * pow(sensorValue,-0.935) - 10;
	return cm;
}