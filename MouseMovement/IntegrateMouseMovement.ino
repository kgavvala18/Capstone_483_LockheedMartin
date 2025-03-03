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


Adafruit_APDS9960 apds9960; // proximity, light, color, gesture
Adafruit_BMP280 bmp280;     // temperautre, barometric pressure
Adafruit_LIS3MDL lis3mdl;   // magnetometer
Adafruit_LSM6DS3TRC lsm6ds3trc; // accelerometer, gyroscope
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_SHT31 sht30;       // humidity

BLEDis bledis;
BLEHidAdafruit blehid;

//adjustable configurations
#define MOVE_STEP    1000
#define SCROLL_STEP    1
#define DAMPING 0.9 //damping threshold to reduce drift
#define SCOLL_THRESHOLD 1.0 //how far from base
#define POSITION_THRESHOLD .001 //how far from base
//for integration
float velocity_z = 0.0, velocity_y = 0.0 , velocity_x = 0.0 ;
float position_z = 0.0, position_y = 0.0 , position_x = 0.0 ;
unsigned long prev_time = 0.0;

//gravity values
float gravity_z = 0.0, gravity_y = 0.0 , gravity_x = 0.0 ;

uint8_t proximity;
uint16_t r, g, b, c;
float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;
int x_step, y_step;
long int accel_array[6];
long int check_array[6]={0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
float last_time;

extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

bool new_rev = true;

void setup(void) {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  Serial.println("Feather Sense Sensor Demo");
  //Serial.print("test1");
  // initialize the sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  bmp280.begin();
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

  //set start accel
  gravity_x = accel.acceleration.x;
  gravity_y = accel.acceleration.y;
  gravity_z = accel.acceleration.z;

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
  sht30.begin();
  //can remove below unless use gesture control
  //PDM.onReceive(onPDMdata);
  //PDM.begin(1, 16000);

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
  //Serial.print("test4");
  startAdv();
  //`Serial.print("test2");
}

void loop(void) {
    // proximity = apds9960.readProximity();
    // while (!apds9960.colorDataReady()) {
    //   delay(5);
    // }
    // apds9960.getColorData(&r, &g, &b, &c);
  
    // temperature = bmp280.readTemperature();
    // pressure = bmp280.readPressure();
    // altitude = bmp280.readAltitude(1013.25);
    
    //below is testing for keyboard inputs
    while(false){
      
      //blehid.keyPress(HID_KEY_SHIFT_LEFT);  // Required for '+'
      //blehid.keyPress(HID_KEY_EQUAL); 
      //blehid.mouseScroll(SCROLL_STEP);
      //blehid.keyPress(0x01);
      //blehid.keyPress('c');
      
      
      uint8_t keycode[6] = {0};
      keycode[0] = 0x06;     // HID code for 'C'
      //Serial.print("Modifier: ");
      //Serial.println(0x01);    // Should print 1 for Ctrl
      //Serial.print("Keycode: ");
      //Serial.println(keycode[0], HEX);  // Should print 06 for 'C'
      delay(10000);
      blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0x01, keycode);  // Send Ctrl + C as ctrl modifier
      delay(10000);
      uint8_t emptyKeycode[6] = {0};                                  
      blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0, emptyKeycode); 
      blehid.keyRelease();

      blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0x01, emptyKeycode);  // Hold Ctrl (0x01 = Left Ctrl) is modifier for control

      for (int i = 0; i < 5; i++) {          
        blehid.mouseScroll(1);            
        delay(100);                        
      }

      blehid.keyboardReport(BLE_CONN_HANDLE_INVALID, 0, emptyKeycode); // Release Ctrl modifier

      blehid.keyRelease();
      while(true){
        Serial.print("done");
        delay(10000);
      }


      //blehid.keyPress(0x2D);
      //blehid.mouseMove(5, 5);
      //delay(10000);
      //blehid.keyRelease();
      //delay(10000);
    }
    lis3mdl.read();
    magnetic_x = lis3mdl.x;
    magnetic_y = lis3mdl.y;
    magnetic_z = lis3mdl.z;
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
    //attempt to account for gravity for now
    accel_x = accel.acceleration.x - gravity_x;
    accel_y = accel.acceleration.y - gravity_y;
    accel_z = accel.acceleration.z - gravity_z;
    gyro_x = gyro.gyro.x;
    gyro_y = gyro.gyro.y;
    gyro_z = gyro.gyro.z;
    
    //get diff time
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; // Convert to seconds
    last_time = current_time;

    
    //
    //integrate 
    //if need to save computation maybe check if these changed?
    velocity_z = accel_z * dt;
    position_z = velocity_z * dt;
    velocity_y = accel_y * dt;
    position_y = velocity_y * dt;
    velocity_x = accel_x * dt;
    position_x += velocity_x * dt;
    //damp velocity to avoid drift
    velocity_z *= DAMPING;
    velocity_y *= DAMPING;
    velocity_x *= DAMPING;
    //TODO Add max velocity and maybe use velocity? and add average to get rid of noise?

    //Serial.print("magnetic_x: "); Serial.println(magnetic_x);
    //Serial.print("magnetic_y: "); Serial.println(magnetic_y);
    //Serial.print("magnetic_z: "); Serial.println(magnetic_z);


    Serial.print("velocity_z: "); Serial.println(velocity_z);
    Serial.print("position_z: "); Serial.println(position_z);
    Serial.print("velocity_y: "); Serial.println(velocity_y);
    Serial.print("position_y: "); Serial.println(position_y);
    Serial.print("velocity_x: "); Serial.println(velocity_x);
    Serial.print("position_x: "); Serial.println(position_x);

    //mouse movement logic
    int y_step = 0;
    int z_step = 0;
    if(position_z > POSITION_THRESHOLD || position_z < -POSITION_THRESHOLD){
      z_step=(int)(position_z * MOVE_STEP);
      Serial.println(z_step);
    }
    else {
      z_step = 0;
    }
    if(position_y > POSITION_THRESHOLD || position_y < -POSITION_THRESHOLD){
      y_step=(int)(position_y * MOVE_STEP);
      Serial.println(y_step);
    }
    else{
      y_step = 0;
    }
    //blehid.mouseMove(x_step, y_step);
    blehid.mouseMove(y_step, z_step);
    //scroll if outside of deadzone
    if(position_x > SCOLL_THRESHOLD){
      blehid.mouseScroll(SCROLL_STEP);
    }
    else if(position_x < -SCOLL_THRESHOLD){
      blehid.mouseScroll(-SCROLL_STEP);
    }
    //Serial.print("test1");
    //TODO 
    //ADD/MERGE CLICKs
    //ADD GESTURE RECOGNITION
    //ADD MAGNETOMETER TO KNOW ELEVATION AND START 
    



    //delay might help with smoother movement
    delay(100);

    // humidity = sht30.readHumidity();
  
    // samplesRead = 0;
    // mic = getPDMwave(4000);
    
    // Serial.println("\nFeather Sense Sensor Demo");
    // Serial.println("---------------------------------------------");
    // Serial.print("Proximity: ");
    // Serial.println(apds9960.readProximity());
    // Serial.print("Red: ");
    // Serial.print(r);
    // Serial.print(" Green: ");
    // Serial.print(g);
    // Serial.print(" Blue :");
    // Serial.print(b);
    // Serial.print(" Clear: ");
    // Serial.println(c);
    // Serial.print("Temperature: ");
    // Serial.print(temperature);
    // Serial.println(" C");
    // Serial.print("Barometric pressure: ");
    // Serial.println(pressure);
    // Serial.print("Altitude: ");
    // Serial.print(altitude);
    // Serial.println(" m");
    // Serial.print("Magnetic: ");
    // Serial.print(magnetic_x);
    // Serial.print(" ");
    // Serial.print(magnetic_y);
    // Serial.print(" ");
    // Serial.print(magnetic_z);
    // Serial.println(" uTesla");
    // Serial.print("Acceleration: ");
    // Serial.print(accel_x);
    // Serial.print(" ");
    // Serial.print(accel_y);
    // Serial.print(" ");
    // Serial.print(accel_z);
    // Serial.println(" m/s^2");
    // Serial.print("Gyro: ");
    // Serial.print(gyro_x);
    // Serial.print(" ");
    // Serial.print(gyro_y);
    // Serial.print(" ");
    // Serial.print(gyro_z);
    // Serial.println(" dps");
    // Serial.print("Humidity: ");
    // Serial.print(humidity);
    // Serial.println(" %");
    // Serial.print("Mic: ");
    // Serial.println(mic);
    //delay(300);
    //accel_y -= 4.85; //trimmed off extra processes
    /*x_step = 0;
    y_step = 0;
    if(accel_x>1){ //edit the code such that the if statements do the coord vals and then update all at once at the end
      y_step = -MOVE_STEP; //eg movestepx = 1
    }
    if(accel_x<-1){
      y_step = MOVE_STEP;
    }
    if(accel_y>1){
      x_step = MOVE_STEP; //eg movestep y = y
    }
    if(accel_y<-1){
      x_step = -MOVE_STEP;
    }
    if(y_step>0){
        x_step *= -1; //because sides are reversed when upside down
    }
    blehid.mouseMove(x_step, y_step); //update coordinates at the same time to avoid stair step-ing
  
    //implement scroll detection
    if(accel_z>1){
      blehid.mouseScroll(SCROLL_STEP);
    }
    else if(accel_z<-1){
      blehid.mouseScroll(-SCROLL_STEP);*/
    //}
  
      //   // LRMBF for mouse button(s)
      //   case 'L':
      //     blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
      //   break;
  
      //   case 'R':
      //     blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
      //   break;
  
      //   case 'M':
      //     blehid.mouseButtonPress(MOUSE_BUTTON_MIDDLE);
      //   break;
  
      //   case 'B':
      //     blehid.mouseButtonPress(MOUSE_BUTTON_BACKWARD);
      //   break;
  
      //   case 'F':
      //     // This key is not always supported by every OS
      //     blehid.mouseButtonPress(MOUSE_BUTTON_FORWARD);
      //   break;
  
      //   case 'X':
      //     // X to release all buttons
      //     blehid.mouseButtonRelease();
      //   break;
  
      //   default: break;
      // }
    
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
    Serial.println("BLE Advertising Started...");

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
  