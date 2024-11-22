/*
  9/8/2016 - fixed issue when registered 0% destroyed string to integers conversion
  9/9/2016 - took out 1ms delay on read to better read 200G at quick impact
  9/13/2016 - fixed battery voltage so didn't say 110%
  9/14/2016 - added notes, changed variable names, increased deceleration array to 50, added global timeout of 3 seconds.
  3/20/2017 - updated bluetooth... sending deceleration 1st...

  3/20/17 - all bluetooth eggs are SAMD
  3/20/17 - increased metering time to 25ms after impact as it seemed it needed extra time to measure medium cushy drops
  3/20/17 - updated ADC to 12-bit, zero 200g accelerometer in freefall, added impact angle in degrees
  3/21/17 - set to high power "AT+BLEPOWERLEVEL=4"
  4/24/2017 - reads MMA8 over i2c more raw using mma.x rather than mma.accerleration
  4/24/2017 - found old bug... added line 279 exit loop if(totAcc > 100 && debounceFreefall == 1){    //exits first loop... false throw...
  4/24/2017 - uncommented serial.begin
  4/26/2017 - increased time to 55 seconds between battery sends
  5/19/17 - only send battery data when egg is thrown
*/


// adapted for bluetooth
// new data format 3/21/17 is 4,9,55,0,185 (egg number, deceleration, impact angle, thrown, freefall time)
// new data keyboard input for battery V250('v' for voltage, '2' for egg 2, '50' for 50%)
// new data keyboard input for event is a2]12]65]0]173/ ('a' event start, ']' deliminator, '/' ending)
// 7/8/17 - adapted to nRF52
// 11/8/17 - seperated out battery data
// 11/29/17 - updated code to new nrf52 library Adafruit Library 10a9af52aa  https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/10a9af52aaa3b64e5f3832e008013d0b833f60da
//  01/03/18 - changed battery mvolts threshold from 3000 to 2900 to signify full 100% voltage as before it was only showing ~80%
//  01/03/18 - added bleConnected flag... and delay(1000) when not connected to slow down processor and save battery.. added waitForEvent();
// 02/23/18 - according to adafruit website waitForEvent() does not work until cp2104 is changed out in the next hardware run: https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/51
// 02/23/18 - reduced the power transmission after 45minutes of inactivity when connected or 20 minutes of inactivity when not connected

//04/24/18 - changed low BLE power from -12 to -16...
//         - Changed 40min of inactivity during BLE reduces BLE power and 30min of no BLE connection will also reduce power
//         - changed the inactivity threshold to greater than 455 (totAcc) where as a resting stat is around 400 totAcc
//sent to Poland 4/27/18?

//5/15/19 - updated DFU 0.2.9 (S132 6.1.1)
//        - adafruit board package Bluefruit nrF52 0.10.1
//        - bleConnected = true; //moved out of cccd_callback as that stopped working with DFU update 5/15/19

//3/18/20 - added name changes bound to NODEID
//        - change NODEID on about line 92 to change the name of the device.

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
//added for nRF52
#include <bluefruit.h>
//--------------------ACCELEROMETER LSM6DS3TRC
#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
Adafruit_LSM6DS3TRC mma = Adafruit_LSM6DS3TRC();


float xNeutral,yNeutral,zNeutral;
//capture the near neutral position
float x1,Y1,z1;
float x2,Y2,z2;


void setup()
{
  Serial.begin(115200);
  while(!Serial);
  Wire.setClock(10000);

  if (!mma.begin_I2C()) {
    Serial.println("Couldnt start");
    while (1);
  }

  Serial.println("LSM6DS3TRC Connected.");

  mma.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  


  for (int i = 0; i < 100; i++) 
  {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    mma.getEvent(&accel, &gyro, &temp);

    x1 = accel.acceleration.x;
    x2 = x1 + x2;

    Y1 = accel.acceleration.y;
    Y2 = Y1 + Y1;

    z1 = accel.acceleration.z;
    z2 = z1 + z2;
    delay(5);
  }

  xNeutral = x2 / 100;
  Serial.print("x Base:");
  Serial.println(xNeutral);

  yNeutral = Y2 / 100;
  Serial.print("y Base:");
  Serial.println(yNeutral);

  zNeutral = z2 / 100;
  Serial.print("z Base:");
  Serial.println(zNeutral);



  Serial.println("leaving setup");

  delay(5000);
}

void loop()
{

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mma.getEvent(&accel, &gyro, &temp);

    x1 = xNeutral - accel.acceleration.x;
    Y1 = yNeutral - accel.acceleration.y;
    z1 = zNeutral - accel.acceleration.z;


    Serial.print("X:");
    Serial.print(x1);
    Serial.print("  Y:");
    Serial.print(Y1);
    Serial.print("  Z:");
    Serial.println(z1);

    Serial.print("Gyro X:");
    Serial.print(gyro.gyro.x);
    Serial.print("  Gyro Y:");
    Serial.print(gyro.gyro.y);
    Serial.print("  Gyro Z:");
    Serial.println(gyro.gyro.z);

    Serial.print("Distace: ");
    Serial.println(accel.distance);

    if(x1 > 20 || Y1 > 20 || z1 > 20)
    {
      delay(10000);
    }
    
}
