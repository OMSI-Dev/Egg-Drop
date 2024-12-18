  float battAvg;

float modifiedMap(float value, float inMin, float inMax, float outMin, float outMax) {
    // Map the input value from the input range to the output range
    float mappedValue = (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return mappedValue;
}

uint8_t mvToPercent()
 {
  float measuredvbat;

  //get a quick average to stablize the readings
  for(byte b; b<100;b++)
  { 
    battAvg += analogRead(A6);
  }

  measuredvbat = battAvg/100;

  measuredvbat *= 2;    // we divided by 2(on HW), so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  //Serial.print("VBat: " ); Serial.println(measuredvbat);
  
  //move to a percent and constrain between 0-100 for any abnomalities
  uint8_t battery_level;
  battery_level = constrain(modifiedMap(measuredvbat, 3.30, 3.65, 0, 100), 0, 100);
  
  // Serial.print("VBat %: " ); 
  // Serial.println(battery_level);

  return battery_level;
}


void sendBatteryData()
{
  uint8_t vbat_per = mvToPercent();
  
  // 'v' starts the logging followed nodeID and 22%
  // as characters "v122"
  // as keycodes "86" "49" "50" "50"

  uint8_t batteryData[10] = {0}; //making it 10 bytes as equal to eggData[]
  batteryData[0] = 86;
  batteryData[1] = NODEID;
  batteryData[2] = vbat_per;
  batteryData[3] = 0; //future
  batteryData[4] = 0; //future
  batteryData[5] = 0; //future
  batteryData[6] = 0; //future
  batteryData[7] = 0; //future
  batteryData[8] = 0; //future
  batteryData[9] = 0; //future
  if (Bluefruit.connected()) {
    hrmc.notify(batteryData, sizeof(batteryData));   // Note: We use .notify instead of .write!
  }
  blebas.write(vbat_per);
}
