


void zero200gAxis() { 
  
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mma.getEvent(&accel, &gyro, &temp);

  long x2 = 0;
  long y2 = 0;
  long z2 = 0;

 
  for (int i = 0; i < 5; i++) {
    int x1 = accel.acceleration.x;
    x2 = x1 + x2;
    //delayMicroseconds(200);
    int y1 = accel.acceleration.y;
    y2 = y1 + y2;
   // delayMicroseconds(200);
    int z1 = accel.acceleration.z;
    z2 = z1 + z2;
    //delayMicroseconds(200);
  }
  

}



long read200G(int xNeutral, int yNeutral, int zNeutral) {
  // x is rotating around egg center (left/right)
  // y is rotating around egg middlle (top/down)
  // z is rotating around egg center (tumble)
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mma.getEvent(&accel, &gyro, &temp);
  int x1 = accel.acceleration.x;
  int y1 = accel.acceleration.y;
  int z1 = accel.acceleration.z;
  delayMicroseconds(200);

  //accel is m/s^2

  if (x1 >= xNeutral) 
  {
    x1 = x1 - xNeutral;
  } else {
    x1 = xNeutral - x1;
  }


  if (y1 >= yNeutral) {
    y1 = y1 - yNeutral;
  } else {
    y1 = yNeutral - y1;
  }
  if (z1 >= zNeutral) {
    z1 = z1 - zNeutral;
  } else {
    z1 = zNeutral - z1;
  }

  float totAccTemp = x1+ y1 + z1;
  // totAcc2 = totAccTemp;
  // Serial.print("200G reading: ");
  // Serial.println(totAcc2);
  return totAccTemp;

}