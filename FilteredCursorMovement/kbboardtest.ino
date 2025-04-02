
#define EKF_N 4     //  Y angVel, Y angACC, Z angVel, Z angACC
#define EKF_M 2  //  Y angAcc, Z angAcc

//#include <bluefruit.h>
//#include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <tinyekf.h>
//#include <Mouse.h>
#include <math.h>
#include <Adafruit_TinyUSB.h>



// BLEDis bledis;
// BLEHidAdafruit blehid;



//HID report descriptor using TinyUSB's template
//Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID object
Adafruit_USBD_HID usb_hid;


static const float EPS = 1.5e-6;

static const float Q[EKF_N*EKF_N] = {   

    EPS, 0, 0, 0,  
    0,  EPS*10, 0, 0,
    0, 0, EPS, 0,
    0, 0, 0, EPS*10
};

static const float R[EKF_M*EKF_M] = {  // 2x2

    0.0005, 0,   
    0, 0.0005, 
    
};

// So process model Jacobian is identity matrix
// static const float F[EKF_N*EKF_N] = {   // 
//     1, 1, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 1, 1,
//     1, 1, 0, 1
// };

static const float H[EKF_M*EKF_N] = {
    0, 1, 0, 0,
    0, 0, 0, 1,
};

static ekf_t _ekf;

Adafruit_LSM6DSOX sox;  
unsigned long myTime;
float prev; 
float curr; 
float dt = 0.01;

int sensorpin0 = A0;  // sensor pin
int sensorpin1 = A1;  // sensor pin
int sensor0;  
int sensor1;  
int leftclick = 0;
int rightclick = 0;
uint8_t buttons = 0;

uint8_t const report_id = 0; // no ID

//float F[EKF_N*EKF_N];

// the setup routine runs once when you press reset:
void setup() {

  const float Pdiag[EKF_N] = {0.001, 0.001, 0.001, 0.001};

  ekf_initialize(&_ekf, Pdiag);

  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("Motion Control Glove - Starting");

  //if (!sox.begin_I2C()) {
  if (!sox.begin_SPI(1)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSOX Found!");
  prev = 0;
  // Configure accelerometer/gyroscope
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  sox.setAccelDataRate(LSM6DS_RATE_208_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_208_HZ);


  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  // Set up HID
  usb_hid.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setStringDescriptor("TinyUSB Mouse");
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }


  // Bluefruit.begin();
  // // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  // Bluefruit.Periph.setConnInterval(9, 16); // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  // Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  // // Configure and Start Device Information Service
  // bledis.setManufacturer("Adafruit Industries");
  // bledis.setModel("Bluefruit Feather 52");
  // bledis.begin();

  // // BLE HID
  // blehid.begin();

  // // Set up and start advertising
  // startAdv();
  
}

// the loop routine runs over and over again forever:
void loop() {

  sensor0 = analogRead(sensorpin0);
  sensor1 = analogRead(sensorpin1);
  curr = millis();

  dt = (curr - prev)/1000.0000;
  prev = curr;

  sensors_event_t accel;
  sensors_event_t gyro;
  sox.getEvent(&accel, &gyro);


  float ax = accel.acceleration.x;
  float ay =accel.acceleration.y + 0.32;
  float az =accel.acceleration.z - 10.03;

  float gx =   gyro.gyro.x;
  float gy = 0.01+ gyro.gyro.y;
  float gz = 0.01+ gyro.gyro.z;


  const float z[EKF_M] = {gy, gz};

  const float F[EKF_N*EKF_N] = {
    1, dt, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, dt,
    0, 0, 0, 1
  };
  // Process model is f(x) = x
  const float fx[EKF_N] = { _ekf.x[0] + dt*_ekf.x[1], _ekf.x[1], _ekf.x[2] + dt*_ekf.x[3], _ekf.x[3] };   // velocity y , velocity z

  // Run the prediction step of the DKF
  ekf_predict(&_ekf, fx, F, Q);


  const float hx[EKF_M] = {_ekf.x[1], _ekf.x[3]};
  //   hx[2] = .9987 * this->x[1] + .001;
  
  //const float hx[EKF_M] = {_ekf.x[0], _ekf.x[1] };

  // Run the update step
  ekf_update(&_ekf, z, hx, H, R);

  
  // if (abs(gy) > 0.05 || abs(gz) > 0.8){
  //   vy = 20*tan(gy);
  //   vx = -20*tan(gz);

  // }
  // else{
  //   vx = 0;
  //   vy = 0;
  // }; 
  float my = 10*_ekf.x[1];
  float mz = -10*_ekf.x[3];

  // uint8_t report_id = 0;
  // uint8_t buttons = 0;

  // // Make sure x and y are int8_t, not uint8_t
  // int8_t x = mz;  // Replace with your x value
  // int8_t y = my;   // Replace with your y value



 
  //blehid.mouseMove(mz, my); 



  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }
  // Remote wakeup
  if (TinyUSBDevice.suspended()) {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    TinyUSBDevice.remoteWakeup();
  }

  if (usb_hid.ready()) {
    
     // Convert to int8_t (scaling if needed)
    //int8_t deltaX = limit_xy(mz);
    //int8_t deltaY = limit_xy(my);
    
    //Serial.print(deltaX);Serial.print(",");
    //Serial.print(deltaY);Serial.print(",");
    if (rightclick){ //left as a polling event because analog pins on our board does not support external interupts
      if (sensor0 < 40){
        rightclick = 0;
        Serial.print("right released");

        //usb_hid.mouseButtonRelease(report_id);
        delay(1000);
      }
    }
    else{
      if(sensor0>50){
        Serial.println("right click!: " + String(sensor0));
        rightclick = 1;
        //usb_hid.mouseButtonPress(report_id, MOUSE_BUTTON_RIGHT);
        // Small delay to simulate a real click
        delay(100); //may remove
      }
    }

    //left mouse button
    if (leftclick){
      if (sensor1 < 40){
        leftclick = 0;
        Serial.print("left released");
        //usb_hid.mouseButtonRelease(report_id);
        //delay(1000);
      }
    }(report_id,MOUSE_BUTTON_LEFT);
        // Small delay to simulate a real click
        delay(100);
      }
    else{
      if(sensor1>50){
        Serial.println("left click!: " + String(sensor0));
        leftclick = 1;
        //usb_hid.mouseButtonPress
    }

    buttons = 0;
    if (rightclick) buttons |= MOUSE_BUTTON_RIGHT;
    if (leftclick) buttons |= MOUSE_BUTTON_LEFT;

    int8_t deltaX = (int8_t) mz;   // Convert float to int8_t
    int8_t deltaY = (int8_t) my;   // Convert float to int8_t

    // Send movement with the correct button state
    usb_hid.mouseReport(report_id, buttons, deltaX, -deltaY, 0, 0);
  }

  
  
  // Serial.print(my); Serial.print(",");
  // Serial.print(mz); Serial.print(",");

  // Serial.print(ax); Serial.print(",");
  // Serial.print(_ekf.x[3]); Serial.print(",");
  Serial.print(sensor0); Serial.print(",");
  Serial.print(sensor1); Serial.print(",");
  // Serial.print(ax); Serial.print(",");
  // Serial.print(ay); Serial.print(",");
  Serial.print("right state: "); 
  Serial.print(rightclick); Serial.print(",");
  Serial.print("left state: "); 
  Serial.print(leftclick);Serial.print(",");
  Serial.print(curr/1000.000); Serial.println(); 

}



// void startAdv(void)
// {  
//   // Advertising packet
//   Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
//   Bluefruit.Advertising.addTxPower();
//   Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  
//   // Include BLE HID service
//   Bluefruit.Advertising.addService(blehid);

//   // There is enough room for 'Name' in the advertising packet
//   Bluefruit.Advertising.addName();
  
//   /* Start Advertising
//    * - Enable auto advertising if disconnected
//    * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
//    * - Timeout for fast mode is 30 seconds
//    * - Start(timeout) with timeout = 0 will advertise forever (until connected)
//    * 
//    * For recommended advertising interval
//    * https://developer.apple.com/library/content/qa/qa1931/_index.html   
//    */
//   Bluefruit.Advertising.restartOnDisconnect(true);
//   Bluefruit.Advertising.setInterval(16, 160);    // in unit of 0.625 ms
//   Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
//   Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
// }
