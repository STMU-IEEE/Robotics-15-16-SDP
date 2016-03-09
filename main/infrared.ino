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
	return multiMap((avg_sensor_value / N), ir_left_raw_table, ir_distances_cm, IR_TABLE_SIZE);
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

//multiMap from Arduino Playground by rob.tillaart@removethisgmail.com
//http://playground.arduino.cc/Main/MultiMap

// note: the _in array should have increasing values
int multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}