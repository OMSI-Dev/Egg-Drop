byte findAverageRolling() {
  int highValue = 0;
  int numberOf = 0;
  int byteToSend = 0;

  for (int i = 0; i < sizeRolling; i++) {
    if (rollingAcc[i] > 500) {
      numberOf++;
      highValue = highValue + rollingAcc[i];
    }
  }
  if (numberOf > 0) {
    byteToSend = highValue / numberOf;
    return (byte)byteToSend;
  } else {
    byteToSend = 0;
    return (byte)byteToSend;
  }
}


byte findFirstByte(int freefall) {

  uint8_t firstByte = 0;
  //first 8 bits
  for (int i = 7; i > 0; i--)
  {
    byte x = bitRead(freefall, i);
    if (x)
    {
      bitSet(firstByte, i);
    }
  }
  byte x = bitRead(freefall, 0);
  if (x)
  {
    bitSet(firstByte, 0);
  }

  return firstByte;
}

byte findSecondByte(int freefall) {

  byte secondByte = 0;
  //bits 8 to 15
  int j = 7;
  for (int i = 15; i > 7; i--) {
    byte x = bitRead(freefall, i);
    if (x) {
      bitSet(secondByte, j);
    }
    j--;

  }


  return secondByte;
}






int findHighestDecel() {
  int highValue = 0;
  int impactIncrement = 0;

  // gforce = deceleration / 9.8m/s^2

  
  for (int i = 0; i < incrementEnd; i++) {
    // Serial.print(eggDecel[i]);
    // Serial.print(",");
    if (eggDecel[i] > highValue) {
      highValue = eggDecel[i];
      impactIncrement = i;
    }
  }
  
  Serial.println();
  Serial.print("highest Accel: ");
  Serial.println(highValue);
  Serial.print("Geforce: ");
  Serial.println(highValue/9.8);

  memset(eggDecel, 0, sizeof(eggDecel));

  // x is rotating around egg center (left/right)
  // y is rotating around egg middlle (top/down)
  // z is rotating around egg center (tumble)
  
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mma.getEvent(&accel, &gyro, &temp);
  impactAngle = atan2(gyro.gyro.y,gyro.gyro.x);
  impactAngle = abs(impactAngle * (180.0 / PI));
  
  if(impactAngle >= 90)
  {
    orientation = 1;
  }
  if(impactAngle <= 20)
  {
    orientation = 0;
  }

  return highValue/9.8;
}

void calibrate()
{
  Serial.println("Calibrating...");
  float x1=0,y1=0,z1=0,x2=0,y2=0,z2=0;

  mma.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  mma.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);

  for (int i = 0; i < calibrationRate; i++) 
  {

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    mma.getEvent(&accel, &gyro, &temp);

    Serial.print("Calibration Step: ");
    Serial.println(i);

    x1 = accel.acceleration.x;
    x2 = x1 + x2;
    y1 = accel.acceleration.y;
    y2 = y1 + y2;
    z1 = accel.acceleration.z ;
    z2 = z1 + z2;
  }

  xNeutral = x2 / calibrationRate;
  Serial.print("x Base:");
  Serial.println(xNeutral);

  yNeutral = y2 / calibrationRate;
  Serial.print("y Base:");
  Serial.println(yNeutral);

  zNeutral = z2 / calibrationRate;
  Serial.print("z Base:");
  Serial.println(zNeutral);

}

int findHeight(long currtotalFreefallTime)
{
  #define gravity 9.806
  #define initialVel 0
  #define totalFalltime currtotalFreefallTime/1000
  
  float freeSpeed = initialVel + gravity*totalFalltime;
  float height = 0.5 * gravity * totalFalltime;

  Serial.print("Free fall speed before impact in millis: ");
  Serial.println(currtotalFreefallTime);
  Serial.print("Free fall speed before impact in millis: ");
  Serial.println(totalFalltime);    
  Serial.print("Free fall speed before impact in m/2: ");
  Serial.println(freeSpeed);
  Serial.print("Height at impact");
  Serial.print(height);
  Serial.println(" M");
}