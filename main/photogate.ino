//Take sum of 32 readings (interpret as 15-bit average instead of 10-bit)
//(Why 32? maximimum readings that fit inside signed 16-bit, and still fast)
int photogateAverage() {
  int average = 0;
  for(int count = 0; count < 32; count++)
    average += analogRead(PHOTOGATE_PIN);
  return average;
}

