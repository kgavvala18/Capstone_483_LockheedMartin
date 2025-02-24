// The MIT License (MIT)

// Copyright (c) 2013-2023 Damien P. George

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>
#include <bluefruit.h>

Adafruit_LIS3MDL lis3mdl;   // magnetometer
Adafruit_LSM6DS3TRC lsm6ds3trc; // accelerometer, gyroscope
Adafruit_LSM6DS33 lsm6ds33;


BLEDis bledis;
BLEHidAdafruit blehid;

#define SMALL_MOVE_STEP     1
#define MEDIUM_MOVE_STEP    4
#define LARGE_MOVE_STEP     9

//uint8_t proximity;
//uint16_t r, g, b, c;
//float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
//float humidity;
//int32_t mic;
int x_step, y_step;
long int accel_array[6];
long int check_array[6]={0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

bool new_rev = true;

int sensorpin0 = A0;  // sensor pin
int sensorpin1 = A1;  // sensor pin
int sensor0;  
int sensor1;  
int leftclick = 0;
int rightclick = 0;

void setup(void) {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  //Serial.println("Feather Sense Sensor Demo");

  // initialize the sensors
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  // check for readings from LSM6DS33
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  accel_array[0] = accel.acceleration.x;
  accel_array[1] = accel.acceleration.y;
  accel_array[2] = accel.acceleration.z;
  accel_array[3] = gyro.gyro.x;
  accel_array[4] = gyro.gyro.y;
  accel_array[5] = gyro.gyro.z;
  // if all readings are empty, then new rev
  for (int i =0; i < 5; i++) {
    if (accel_array[i] != check_array[i]) {
      new_rev = false;
      break;
    }
  }
  // and we need to instantiate the LSM6DS3TRC
  if (new_rev) {
    lsm6ds3trc.begin_I2C();
  }
  //sht30.begin();
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);

  //Mouse Setup

  Bluefruit.begin();
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16); // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // BLE HID
  blehid.begin();

  // Set up and start advertising
  startAdv();
}

void loop(void) {
  sensor0 = analogRead(sensorpin0);
  sensor1 = analogRead(sensorpin1);

  lis3mdl.read();
  accel_x = 0;
  accel_y = 0;
  accel_z = 0;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  if (new_rev) {
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
  }
  else {
    lsm6ds33.getEvent(&accel, &gyro, &temp);
  }
  accel_x = accel.acceleration.x;
  accel_y = accel.acceleration.y;
  accel_z = accel.acceleration.z;
  gyro_x = gyro.gyro.x;
  gyro_y = gyro.gyro.y;
  gyro_z = gyro.gyro.z;

  if (rightclick){ //left as a polling event because analog pins on our board does not support external interupts
    if (sensor0 < 80){
      rightclick = 0;
      blehid.mouseButtonRelease(MOUSE_BUTTON_RIGHT);
    }
  }
  else{
    if(sensor0>80){
      //Serial.println("click!: " + String(sensor0));
      rightclick = 1;
      blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
      // Small delay to simulate a real click
      delay(100); //may remove
    }
  }

  //left mouse button
  if (leftclick){
    if (sensor1 < 80){
      leftclick = 0;
      blehid.mouseButtonRelease(MOUSE_BUTTON_LEFT);
    }
  }
  else{
    if(sensor1>80){
      //Serial.println("click!: " + String(sensor0));
      leftclick = 1;
      blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
      // Small delay to simulate a real click
      delay(100);
    }
  }

  x_step = 0;
  y_step = 0;
  if(accel_x>4.2){ 
    y_step = -LARGE_MOVE_STEP; 
  }
  else if(accel_x>2.5){ 
    y_step = -MEDIUM_MOVE_STEP; 
  }
  else if(accel_x>1){ 
    y_step = -SMALL_MOVE_STEP; 
  }
  if(accel_x<-4.2){
    y_step = LARGE_MOVE_STEP;
  }
  else if(accel_x<-2.5){
    y_step = MEDIUM_MOVE_STEP;
  }
  else if(accel_x<-1){
    y_step = SMALL_MOVE_STEP;
  }
  if(accel_y>4.2){
    x_step = LARGE_MOVE_STEP; 
  }
  else if(accel_y>2.5){
    x_step = MEDIUM_MOVE_STEP; 
  }
  else if(accel_y>1){
    x_step = SMALL_MOVE_STEP;
  }
  if(accel_y<-4.2){
    x_step = -LARGE_MOVE_STEP;
  }
  else if(accel_y<-2.5){
    x_step = -MEDIUM_MOVE_STEP;
  }
  else if(accel_y<-1){
    x_step = -SMALL_MOVE_STEP;
  }
  if(y_step>0){
      x_step *= -1;
  }
  blehid.mouseMove(x_step, y_step);  
}

/*****************************************************************/
int32_t getPDMwave(int32_t samples) {
  short minwave = 30000;
  short maxwave = -30000;

  while (samples > 0) {
    if (!samplesRead) {
      yield();
      continue;
    }
    for (int i = 0; i < samplesRead; i++) {
      minwave = min(sampleBuffer[i], minwave);
      maxwave = max(sampleBuffer[i], maxwave);
      samples--;
    }
    // clear the read count
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  
  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for 'Name' in the advertising packet
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}
