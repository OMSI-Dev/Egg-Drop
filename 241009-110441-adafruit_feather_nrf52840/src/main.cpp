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
#include <Math.h>
#include <SPI.h>

//added for nRF52
#include <bluefruit.h>

/* HRM Service Definitions
   Heart Rate Monitor Service:  0x180D
   Heart Rate Measurement Char: 0x2A37
*/
BLEService        hrms = BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic hrmc = BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

// Advanced function prototypes
void startAdv(void);
void setupHRM(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void cccd_callback(BLECharacteristic& chr, ble_gatts_evt_write_t* request);


//--------------------ACCELEROMETER MMA8451
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>
//#include <Adafruit_MMA8451.h>

Adafruit_LSM6DS3TRC mma = Adafruit_LSM6DS3TRC();



//*********************************************************************************************


#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
//#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
//#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider


//*************************************************************************
//                         EGG DROP VARIABLES
//*************************************************************************
int freeFallThreshold = 20; // ms before freefall event triggered
byte NODEID = 1;    // The unique identifier of this node. Default Range 1 - 6 will also change the name to "Blue Egg #1" and such.

//**************************************************************************


//These variables will be used to hold the x,y and z axis accelerometer values.
int x = 10;
int y = 10;
int z = 10;
double xg, yg, zg;
char tapType = 0;


long now = 0;
int startFreefall = 0;
long startFreefallTime = 0;
long totalFreefallTime = 0;
long startDeceleration = 0;
long decelerationTime = 0;
long lastHeartBeatSent = 0;

const int sizeEggDecel = 300;
const int sizeRolling = 100;

int eggDecel[sizeEggDecel];
int eggDecelX[sizeEggDecel];
int eggDecelY[sizeEggDecel];
int eggDecelZ[sizeEggDecel];
int eggDecelO[sizeEggDecel];

int orientation = 0;

int rollingAcc[sizeRolling];

int captureDeceleration = 0;
int incrementDecel = 0;
int incrementRolling = 2;

int incrementEnd = 0;
int decelSwitch = 0;

int highestDecel = 0;

int totAcc = 10;

//200g variables
const int zAxis = A0;
const int yAxis = A1;
const int xAxis = A2;

int xNeutral = 0;
int yNeutral = 0;
int zNeutral = 0;

long reading = 0;
int totAcc2 = 0;
float xg2, yg2, zg2;

byte sendRolling[sizeRolling / 2];

byte averageValueThrow = 0;

char radioPacketSend[30];

int debounceFreefall = 0;

String stringSend;

int sendBatteryInterval = 5000; //send battery data every 30 seconds
long lastTimeSentBatteryData = 0;



int zero200G = 0;
double impactAngle = 0;

boolean bleConnected = false;
boolean goingIntoLowPowerFlag = 1; //stays in high power for 20min during charging from dead
boolean alreadyWent = 0; //.. sets to into low power after above 20min flag is met...
boolean activeModeFlag = true;
long lastActiveMode = 0;

boolean activeLowPowerFlag = 0;





//------------------------------------------------------  BEGIN SETUP

void setup()
{

  Serial.begin(115200);
  while(!Serial)
  Serial.println("OMSI Egg Drop");
  Serial.println("--------------------------------");


  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name to 'egg#'");

  if (NODEID == 1) {
    Bluefruit.setName("Egg #1");
  } else if (NODEID == 2) {
    Bluefruit.setName("Egg#2");
  } else if (NODEID == 3) {
    Bluefruit.setName("Egg#3");
  } else if (NODEID == 4) {
    Bluefruit.setName("Egg#4");
  } else if (NODEID == 5) {
    Bluefruit.setName("Egg#5");
  } else if (NODEID == 6) {
    Bluefruit.setName("Egg#6");
  } else {
    Bluefruit.setName("Smart Egg");
  }


  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("OMSI");
  bledis.setModel("Thomas Hudson nRF52");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();

  // Setup the Heart Rate Monitor service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Heart Rate Monitor Service");
  setupHRM();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();

  // Start Advertising
  Serial.println("Ready Player One!!!");
  Serial.println("\nAdvertising");
  Bluefruit.Advertising.start();

  Serial.println("setting BLE power to -16");
  Bluefruit.setTxPower(-16);


  //---------------------- BLE variables

  //--------------- BLE variables

  analogReadResolution(12);
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095),
  // Set the analog reference to 3.0V (default = 3.6V)
  //analogReference(AR_INTERNAL_3_3);
  //analogReference(EXTERNAL); //default is 3.6V

  //Serial.println("we get here");

  Wire.setClock(10000);

  if (! mma.begin_I2C()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LSM6DS3TRC found!");

  mma.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);

  //Serial.print("Range = "); Serial.print(2 << mma.getRange());
  //Serial.println("G");

  //capture the near neutral position of the 200G accerometer..at 12bit or 50% of 0-4095=2048
  long x2 = 0;
  long y2 = 0;
  long z2 = 0;
  int temp = analogRead(xAxis);  //read and throw away
  for (int i = 0; i < 20; i++) {
    int x1 = analogRead(xAxis);
    delay(5);
    x1 = analogRead(xAxis);
    x2 = x1 + x2;
    delay(5);
    int y1 = analogRead(yAxis);
    y2 = y1 + y2;
    delay(5);
    int z1 = analogRead(zAxis);
    z2 = z1 + z2;
    delay(5);
  }
  xNeutral = (int)x2 / 20;
  Serial.println(xNeutral);
  yNeutral = (int)y2 / 20;
  Serial.println(yNeutral);
  zNeutral = (int)z2 / 20;
  Serial.println(zNeutral);


  Serial.println("leaving setup");

  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  //Bluefruit.setTxPower(-20);  //defaults to high power and stays on high power for 20min from dead battery while first charging

}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(hrms);

  // Include Name
  Bluefruit.Advertising.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void setupHRM(void)
{
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Heart Rate Measurement       0x2A37  Mandatory   Notify
  hrms.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  hrmc.setProperties(CHR_PROPS_NOTIFY);
  hrmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hrmc.setFixedLen(10); //was 2 changed to 4 to 10.... important to be right if in... comment out?
  //hrmc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  hrmc.begin();
  //not sure why this is needed
  uint8_t hrmdata1[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }; // this used to send out data about the heart rate meter to the client
  hrmc.notify(hrmdata1, 10);    //important... tells it's sending 10 bytes              // Use .notify instead of .write!
}


void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
  Serial.println("connected here");

  //going to high power
  Serial.println("going into high transmission power mode sense we're connecting to BLE");
  Bluefruit.setTxPower(4); // goes to high power when connecting bo ble

  lastActiveMode = now; //indicating that we have activity.... so we stay in high power mode for at least 30 min TBD
  bleConnected = true; //moved out of cccd_callback as that stopped working with DFU update 5/15/19
  lastTimeSentBatteryData = 0; //trigger battery send immediately after connected
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");
  Serial.println("Advertising!");
  bleConnected = false;
  alreadyWent = 0;  //flag that lets device go into low power after disconnected

}

void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
  // Display the raw request packet
  Serial.print("CCCD Updated: ");
  //Serial.printBuffer(request->data, request->len);
  Serial.print(cccd_value);
  Serial.println("");

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr.uuid == hrmc.uuid) {
    if (chr.notifyEnabled()) {
      Serial.println("OMSI 'Notify' enabled");
      //bleConnected = true;
    } else {
      Serial.println("OMSI 'Notify' disabled");
    }
  }
}
//------------------------------------------------------  END SETUP



//-------------------------------------------------------BEGIN LOOP

void loop(void)
{
  now = millis();

  if (bleConnected)
  {
    if (startFreefall == 0 && captureDeceleration == 0) //if nothing is happening send battery data
    {
      if (now - lastTimeSentBatteryData > sendBatteryInterval) // send battery data every 30 seconds
      {
        sendBatteryData();
        lastTimeSentBatteryData = now;
      }
    }


    if (startFreefall == 0 && captureDeceleration == 0) { //if nothing is happening keep a rolling average of acceleration values
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      mma.getEvent(&accel, &gyro, &temp);
      x = accel.acceleration.x / 10;
      y = accel.acceleration.y / 10;
      z = accel.acceleration.z / 10;

      totAcc = sqrt(x * x + y * y + z * z);

      rollingAcc[incrementRolling] = totAcc;      //keeps track of thrown or dropped
      incrementRolling++;
      if (incrementRolling == sizeRolling) {
        incrementRolling = 0;
      }
      if (totAcc > 455) //device is getting moved around... should be high power.. resting totACC state is ~404
      {
        //Serial.print("device is moving around:");
        //Serial.println(totAcc);
        //activeModeFlag = true;
        lastActiveMode = now;
        if (activeLowPowerFlag == 1) // just coming out of low power mode... go to high power transmission
        {
          Bluefruit.setTxPower(4);
          Serial.println("device is moving around after being in Low Power mode for 45min, going to high power mode");
          activeLowPowerFlag = 0;
        }
      }
      delay(1);
    }

    if (now - lastActiveMode > 2400000) //40 minutes has passed without activity, going into low power transmission mode
    {
      Serial.println("going into low transmission power mode after 40min of inactivity while ble connected");
      Bluefruit.setTxPower(-16);
      lastActiveMode = now;
      activeLowPowerFlag = 1;
    }



    if (totAcc <= 50 && debounceFreefall == 0 && startFreefall == 0 && captureDeceleration == 0) { //accerlation is less than a 10th ***OBJECT IN FREEFALL***
      startFreefallTime = now;
      debounceFreefall = 1; //set debounce flag
    }

    if (totAcc <= 50 && now - startFreefallTime > freeFallThreshold && debounceFreefall == 1 && startFreefall == 0 && captureDeceleration == 0) { //passed debounce test for free-fall, ***OBJECT IN FREEFALL***
      startFreefall = 1;
      zero200gAxis();
      incrementDecel = 0;
      //Serial.println("free falling");
    }

    if (totAcc > 100 && debounceFreefall == 1) {  //exits first loop... false throw...
      debounceFreefall = 0;
    }


    if (startFreefall == 1) { //start monitoring for deceleration
      debounceFreefall = 0; //release flag
      read200G();              //switch to 200G accelerometer
    }

    if (now - startFreefallTime > 3000) { //3 seconds have passed in freefall there was no deceleration(impact)... restart everything
      startFreefall = 0;
      debounceFreefall = 0;
    }

    if (totAcc2 >= 4 && startFreefall == 1 && captureDeceleration == 0) { //freefall has ended, only monitor deceleration for another 15ms
      captureDeceleration = 1;
      startDeceleration = now;
      decelerationTime = 0;
      highestDecel = 0;
      incrementDecel = 0;
      decelSwitch = totAcc2;
    }


    if (captureDeceleration == 1) { //start monitor time as the egg hits the ground
      decelerationTime = now - startDeceleration;
      if (incrementDecel == sizeEggDecel) {
        incrementDecel = 0;
      }
      if (incrementDecel < sizeEggDecel) {
        eggDecel[incrementDecel] = totAcc2;   //log 200G accelerometer
        eggDecelX[incrementDecel] = xg2;   //log 200G accelerometer
        eggDecelY[incrementDecel] = yg2;   //log 200G accelerometer
        eggDecelZ[incrementDecel] = zg2;   //log 200G accelerometer
        eggDecelO[incrementDecel] = orientation;
        incrementDecel++;
      }
    }

    if (decelerationTime > 50) { //everything stops after 50ms has passed

      decelerationTime = 0;
      captureDeceleration = 0;
      startFreefall = 0;
      incrementEnd = incrementDecel;
      totalFreefallTime = now - startFreefallTime - 50;
      averageValueThrow = findAverageRolling();
      highestDecel = findHighestDecel();

      uint8_t eggData[10] = {0};

      eggData[0] = 65; //'a' for deceleration
      eggData[1] = NODEID;

      if (highestDecel > 200) {
        highestDecel = 200;  //it should never be greater than 200!
      }
      eggData[2] = highestDecel;
      eggData[3] = impactAngle;

      if (averageValueThrow > 99) {
        averageValueThrow = 99; //unlikely to get this big but clamping value
      }
      eggData[4] = averageValueThrow;

      if (totalFreefallTime < 10) {
        totalFreefallTime = 10;
      }
      if (totalFreefallTime > 9999) {
        totalFreefallTime = 9999;
      }
      Serial.print("finding bytes from: ");
      Serial.println(totalFreefallTime);
      byte firstByte = findFirstByte(totalFreefallTime);
      eggData[5] = firstByte;
      byte secondByte = findSecondByte(totalFreefallTime);
      eggData[6] = secondByte;

      eggData[7] = 0; //future
      eggData[8] = 0; //future
      eggData[9] = 0;  //future

      if (Bluefruit.connected()) {
        //Serial.println("BLE connected, sending eggData[10]");
        hrmc.notify(eggData, sizeof(eggData));   // Note: We use .notify instead of .write!
      } else {
        Serial.println("BLE not connected");
      }
      delay(2000);
    }
  } else
  {
    //Serial.println(now-lastActiveMode);
    //Serial.println(goingIntoLowPowerFlag);
    //Serial.println(alreadyWent);
    if (now - lastActiveMode > 1800000) //after 30 minutes of not connecting... go to low power mode
    {
      goingIntoLowPowerFlag = 0;
      Serial.println("BLE not connected..& greater than 30min of inactivity, setting flag for low power... ");
      lastActiveMode = now;
    }
  }
  if (goingIntoLowPowerFlag == 0 && alreadyWent == 0) //going to low power mode!
  {
    Serial.println("going into low power mode");
    Bluefruit.setTxPower(-16);  //set to low power
    alreadyWent = 1;
    waitForEvent();  //low power?

  }
}

//******************************************************** END LOOP


void sendBatteryData()
{
  // Get a raw ADC reading
  int vbat_raw = readVBAT();
  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);
  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095),
  // VBAT voltage divider is 2M + 0.806M, which needs to be added back
  //float vbat_mv = (float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
  byte percent = vbat_per;
  //Serial.print("Percent voltage: " ); Serial.print(percent); Serial.println("%");

  // 'v' starts the logging followed nodeID and 22%
  // as characters "v122"
  // as keycodes "86" "49" "50" "50"

  uint8_t batteryData[10] = {0}; //making it 10 bytes as equal to eggData[]
  batteryData[0] = 86;
  batteryData[1] = NODEID;
  batteryData[2] = percent;
  batteryData[3] = 0; //future
  batteryData[4] = 0; //future
  batteryData[5] = 0; //future
  batteryData[6] = 0; //future
  batteryData[7] = 0; //future
  batteryData[8] = 0; //future
  batteryData[9] = 0; //future
  if (Bluefruit.connected()) {
    //Serial.println("BLE connected, sending batteryData[10]");
    hrmc.notify(batteryData, sizeof(batteryData));   // Note: We use .notify instead of .write!
  }
  blebas.write(percent);
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
    //    Serial.print(bitRead(freefall, i));
    //    Serial.print(",");
  }
  byte x = bitRead(freefall, 0);
  if (x)
  {
    bitSet(firstByte, 0);
  }
  //  Serial.print(bitRead(freefall,0));
  //  Serial.println();
  //  Serial.println(firstByte);
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
    //    Serial.print(bitRead(freefall, i));
    //    Serial.print(",");
  }
  //  Serial.println();
  //  Serial.println(secondByte);
  return secondByte;
}




int findHighestDecel() {
  int highValue = 0;
  int impactIncrement = 0;
  Serial.print("deceleration starting at impact(> 4gs) for 50ms:  ");
  for (int i = 0; i < incrementEnd; i++) {
    Serial.print(eggDecel[i]);
    Serial.print(",");
    if (eggDecel[i] > highValue) {
      highValue = eggDecel[i];
      impactIncrement = i;
    }
  }
  Serial.println();
  Serial.print("high value: ");
  Serial.print(highValue);

  memset(eggDecel, 0, sizeof(eggDecel));
  Serial.println();

  for (int i = incrementDecel; i < sizeEggDecel; i++) {
    //Serial.print(eggDecelY[i]);
    //Serial.print(",");
  }
  for (int i = 0; i < incrementDecel; i++) {
    //Serial.print(eggDecelY[i]);
    //Serial.print(",");
  }
  //Serial.println();

  for (int i = incrementDecel; i < sizeEggDecel; i++) {
    //Serial.print(eggDecelX[i]);
    //Serial.print(",");
  }
  for (int i = 0; i < incrementDecel; i++) {
    //Serial.print(eggDecelX[i]);
    //Serial.print(",");
  }
  //Serial.println();

  for (int i = incrementDecel; i < sizeEggDecel; i++) {
    //Serial.print(eggDecelZ[i]);
    //Serial.print(",");
  }
  for (int i = 0; i < incrementDecel; i++) {
    //Serial.print(eggDecelZ[i]);
    //Serial.print(",");
  }
  //Serial.println();


  for (int i = incrementDecel; i < sizeEggDecel; i++) {
    //Serial.print(eggDecelO[i]);
    //Serial.print(",");
  }
  for (int i = 0; i < incrementDecel; i++) {
    //Serial.print(eggDecelO[i]);
    //Serial.print(",");
  }
  //Serial.println();
  //Serial.print("DecelX: ");
  //Serial.println(eggDecelX[impactIncrement]);
  //Serial.print("DecelY: ");
  //Serial.println(eggDecelY[impactIncrement]);
  //Serial.print("DecelZ: ");
  //Serial.println(eggDecelZ[impactIncrement]);
  double xzAverage = sqrt(sq(eggDecelX[impactIncrement]) + sq(eggDecelZ[impactIncrement]));
  //Serial.println(xzAverage);
  impactAngle = atan(xzAverage / eggDecelY[impactIncrement]);
  //Serial.println(impactAngle);
  impactAngle = (impactAngle * 4068) / 71; //radians to degrees
  //Serial.println(impactAngle);
  if (eggDecelO[impactIncrement] == 0) { //egg hit upside down
    impactAngle = 180 - impactAngle;
  }
  impactAngle = impactAngle;
  Serial.print("impact angle(0-180): ");
  Serial.println(impactAngle);

  return highValue;
}


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




void zero200gAxis() { //adafruit capacitors are selected for 500hz reading.. we're probably closer to 1000hz here

  long x2 = 0;
  long y2 = 0;
  long z2 = 0;
  int x1 = analogRead(xAxis);  //read and throw away
  delayMicroseconds(200);
  for (int i = 0; i < 5; i++) {
    x1 = analogRead(xAxis);
    x2 = x1 + x2;
    delayMicroseconds(200);
    int y1 = analogRead(yAxis);
    y2 = y1 + y2;
    delayMicroseconds(200);
    int z1 = analogRead(zAxis);
    z2 = z1 + z2;
    delayMicroseconds(200);
  }
  xNeutral = (int)x2 / 5;
  //Serial.println(xNeutral);
  yNeutral = (int)y2 / 5;
  //Serial.println(yNeutral);
  zNeutral = (int)z2 / 5;
  //Serial.println(zNeutral);

}



void read200G() {
  // y is up down orientation
  //x and z are the axis that are the symetrical orienatation

  int x1 = analogRead(xAxis);
  delayMicroseconds(200);
  int y1 = analogRead(yAxis);
  delayMicroseconds(200);
  int z1 = analogRead(zAxis);
  delayMicroseconds(200);

  if (x1 >= xNeutral) {
    x1 = x1 - xNeutral;
  } else {
    x1 = xNeutral - x1;
  }
  if (y1 >= yNeutral) {
    y1 = y1 - yNeutral;
    //Serial.println("positive");
    //landing upside down
    orientation = 0;
  } else {
    y1 = yNeutral - y1;
    //Serial.println("negative");
    //landing right side up
    orientation = 1;
  }
  if (z1 >= zNeutral) {
    z1 = z1 - zNeutral;
  } else {
    z1 = zNeutral - z1;
  }

  // using +/- 8 Gs is 16/1024 = 0.0156
  // using +/- 16 Gs is 32/1024 = 0.0312

  // using +/- 200 Gs is 200/1024 = 0.1953
  // using +/- 200 Gs is 200/2048 = .04883 for 12 bit ... was 200 divided by 4095

  //@ 3.6 volt reverence, 1.65/3.6 = 1877/4096 so 200/1877 = 0.1066

  xg2 = x1 * 0.1066;
  yg2 = y1 * 0.1066;
  zg2 = z1 * 0.1066;

  long totAccTemp = sqrt(xg2 * xg2 + yg2 * yg2 + zg2 * zg2);
  totAcc2 = (int)totAccTemp;
  //Serial.println(totAcc2);

}



void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i = 0; i < loops; i++)
  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}



/**
   RTOS Idle callback is automatically invoked by FreeRTOS
   when there are no active threads. E.g when loop() calls delay() and
   there is no bluetooth or hw event. This is the ideal place to handle
   background data.

   NOTE: It is recommended to call waitForEvent() to put MCU into low-power mode
   at the end of this callback. You could also turn off other Peripherals such as
   Serial/PWM and turn them back on if wanted

   e.g

   void rtos_idle_callback(void)
   {
      Serial.stop(); // will lose data when sleeping
      waitForEvent();
      Serial.begin(115200);
   }

   NOTE2: If rtos_idle_callback() is not defined at all. Bluefruit will force
   waitForEvent() to save power. If you don't want MCU to sleep at all, define
   an rtos_idle_callback() with empty body !

   WARNING: This function MUST NOT call any blocking FreeRTOS API
   such as delay(), xSemaphoreTake() etc ... for more information
   http://www.freertos.org/a00016.html
*/
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here

  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}

int readVBAT(void) {
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  //analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);
  analogReference(AR_DEFAULT);

  return raw;
}

uint8_t mvToPercent(float mvolts) {
  uint8_t battery_level;
  //Serial.println(mvolts);

  if (mvolts >= 2900)
  {
    battery_level = 100;
  }
  else if (mvolts > 2800)
  {
    battery_level = 100 - ((2900 - mvolts) * 58) / 100;
  }
  else if (mvolts > 2740)
  {
    battery_level = 42 - ((2800 - mvolts) * 24) / 160;
  }
  else if (mvolts > 2440)
  {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
  }
  else if (mvolts > 2100)
  {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}
