//Take sum of 32 readings (interpret as 15-bit average instead of 10-bit)
//(Why 32? maximimum readings that fit inside signed 16-bit, and still fast)
int analogAverage(int analog_pin) {
  int average = 0;
  for(int count = 0; count < 32; count++)
    average += analogRead(analog_pin);
  return average;
}

/*
 * for photogate built using 2 proximity sensors
 * use one as emitter; block emitter of receiver to only measure transmitted light
 */
int photogateAverage() {
	return(PHOTOGATE_PIN);
}