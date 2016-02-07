void testMC()
{
      ST.drive(25);
      delay(1000);
      ST.drive(-25);
      delay(1000);
      ST.drive(0);
      ST.turn(25);
      delay(1000);
      ST.turn(-25);
      delay(1000);
      ST.stop();
}

void testSwing(){
  ST.drive(16);
  ST.turn(16);
  delay(2000);
  ST.stop();
}

