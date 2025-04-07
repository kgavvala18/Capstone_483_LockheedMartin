
//#include <tinyekf.hpp>

/*this is an implementation of mouse control using the gyroscope of the LSM6DSOX IMU
An Extended Kalman Filter is used to smooth the acceleration values
which are scaled and used directly for control input
the KB2040 board was used to test this implmentation using usb hid and we are still
converting this to use on the feather nRF52840 sense board with BLe HID
  */

#define EKF_N 6 //  States: Y angular Velocity, Y angACC, Z angVel, Z angACC,
#define EKF_M 4 //  Measurements Y angAcc, Z angAcc

#include <bluefruit.h>
// #include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <tinyekf.h>
// #include <Mouse.h>
#include <math.h>
// #include <Adafruit_TinyUSB.h>s

BLEDis bledis;
BLEHidAdafruit blehid;

static const float EPS = 1.5e-6;

// static const float Q[EKF_N * EKF_N] = {

//     EPS, 0, 0, 0,
//     0, EPS * 10, 0, 0,
//     0, 0, EPS, 0,
//     0, 0, 0, EPS * 10};
static const float Q[EKF_N * EKF_N] = {

  EPS, 0, 0, 0, 0, 0,
  0, EPS * 10, 0, 0, 0, 0,
  0, 0, EPS, 0, 0, 0,
  0, 0, 0, EPS * 10, 0, 0,
  0, 0, 0, 0, EPS * 10, 0,
  0, 0, 0, 0, 0, EPS * 10
};


// static const float R[EKF_M * EKF_M] = {
//     // 2x2

//     0.0005,
//     0,
//     0,
//     0.0005,

// };

static const float R[EKF_M * EKF_M] = {
  // 2x2

  0.0005, 0, 0, 0,
  0, 0.0005, 0, 0,
  0, 0, 0.0005, 0, 
  0, 0, 0, 0.0005, 

};

// So process model Jacobian is identity matrix
// static const float F[EKF_N*EKF_N] = {   //
//     1, 1, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 1, 1,
//     1, 1, 0, 1
// };

// static const float H[EKF_M * EKF_N] = {
//     0,
//     1,
//     0,
//     0,
//     0,
//     0,
//     0,
//     1,
// };

static const float H[EKF_M * EKF_N] = {
  0,1,0,0,0,0,
  0,0,0,1,0,0,
  0,0,0,0,1,0,
  0,0,0,0,0,1
};

static ekf_t _ekf;

Adafruit_LSM6DSOX sox;
unsigned long myTime;
float prev;
float curr;
float dt = 0.01;


int sensorpin0 = A0;  // ring flex sensor pin
int sensorpin1 = A1;  // middle flex sensor pin
int sensorpin2 = A2;  // thumb sensor pin
int sensorpin3 = A3;  // pointer sensor pin
int sensorpin4 = A4;  // pinky sensor pin

int sensor0;  //ring
int sensor1;  //middle
int sensor2;  //middle
int sensor3;  //middle
int sensor4;  //middle

int leftclick = 0;
int rightclick = 0;
int laser = 0;
// float F[EKF_N*EKF_N];

// the setup routine runs once when you press reset:
void setup()
{

  const float Pdiag[EKF_N] = {0.001, 0.001, 0.001, 0.001};

  ekf_initialize(&_ekf, Pdiag);

  Serial.begin(115200);
  while ( !Serial ) {Serial.println("cannot proceed!"); delay(10);}

  Serial.println("Motion Control Glove - Starting");

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

  configBluetooth();

  Bluefruit.begin();
  Bluefruit.setTxPower(4);                 // Check bluefruit.h for supported values

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // BLE HID
  blehid.begin();

  // Set up and start advertising
  startAdv();
  pinMode(A5, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop()
{
  curr = millis();

  dt = (curr - prev) / 1000.0000;
  prev = curr;

  sensor0 = analogRead(sensorpin0);  //ring
  sensor1 = analogRead(sensorpin1); //middle

  sensor2 = analogRead(sensorpin2);
  sensor3 = analogRead(sensorpin3); //pointer
  sensor4 = analogRead(sensorpin4);

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y + 0.32;
  float az = accel.acceleration.z - 10.03;

  float gx = gyro.gyro.x;
  float gy = 0.01 + gyro.gyro.y;
  float gz = 0.01 + gyro.gyro.z;

  const float z[EKF_M] = { gy, gz, sensor3, sensor1 };

  const float F[EKF_N * EKF_N] = {
    1, dt, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, dt, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
  };
  // Process model is f(x) = x
  const float fx[EKF_N] = { _ekf.x[0] + dt * _ekf.x[1], _ekf.x[1], _ekf.x[2] + dt * _ekf.x[3], _ekf.x[3], _ekf.x[4], _ekf.x[5] };  // velocity y , velocity z

  // Run the prediction step of the eKF
  ekf_predict(&_ekf, fx, F, Q);


  const float hx[EKF_M] = { _ekf.x[1], _ekf.x[3], _ekf.x[4], _ekf.x[5] };
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

  // scale the acceleration values by 20
  /*the scaling factor was empiricaly determined. We need to find why this works
  to see if there may be a more opmtimal value*/
  float my = 20 * _ekf.x[1];
  float mz = -20 * _ekf.x[3];

  float sense1 = _ekf.x[4];
  float sense0 = _ekf.x[5];


  blehid.mouseMove(mz, my);

  
 
  if(sense1 > 40){
    //Serial.println("click!: " + String(sensor0));
    blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
    // Small delay to simulate a real click
    delay(100); //may remove
    blehid.mouseButtonRelease(MOUSE_BUTTON_RIGHT);
  }

  //left mouse button
  if (leftclick){
    if (sense0 < 50){
      leftclick = 0;
      blehid.mouseButtonRelease(MOUSE_BUTTON_LEFT);
    }
  }
  else{
    if(sense0 > 50){
      //Serial.println("click!: " + String(sensor0));
      leftclick = 1;
      blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
    }
  }

  if (laser == 0){
    if (sensor2 > 75){
      laser = 1;
      digitalWrite(A5, HIGH);  // Turn the pin on (set it HIGH)
    }
  }
  else {
    if (sensor2 < 75){
      laser = 0;          
      digitalWrite(A5, LOW);   // Turn the pin off (set it LOW)
    }
  }

  // // Serial.print(_ekf.x[1]); Serial.print(",");
  // // Serial.print(_ekf.x[3]); Serial.print(",");
  // // Serial.print(gy); Serial.print(",");
  // // Serial.print(gz); Serial.print(",");
  // // // Serial.print(ax); Serial.print(",");
  // // // Serial.print(_ekf.x[3]); Serial.print(",");
  // // Serial.print(ay); Serial.print(",");
  // // Serial.print(az); Serial.print(",");


  // Serial.print(sense0);
  // Serial.print(",");
  // Serial.print(sense1);
  // Serial.print(",");
  // Serial.print(sense2);
  // Serial.print(",");
  // Serial.print(sense3);
  // Serial.print(",");
  
  //Serial.print("right state: ");
  // Serial.print(rightclick);
  // Serial.print(",");
  // //Serial.print("left state: ");
  // Serial.print(leftclick);
  // Serial.print(",");

  Serial.print(sensor0); //thumb
  Serial.print(",");

  Serial.print(sensor1); //pointer
  Serial.print(",");
  Serial.print(sensor2); //pinky

  Serial.print(",");
  Serial.print(sensor3); //thumb
  Serial.print(",");

  Serial.print(sensor4); //ring
  Serial.print(",");

  // Serial.print(vy); Serial.print(",");
  // Serial.print(vx); Serial.print(",");
  // Serial.print(ax); Serial.print(",");
  // Serial.print(ay); Serial.print(",");
  // Serial.print(az); Serial.println();
  // Serial.print(vy); Serial.print(",");
  // Serial.print(vx); Serial.print(",");
  Serial.print(curr / 1000.000);
  Serial.println();
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
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void configBluetooth() {
  // add any bluetooth config stuff here
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configPrphConn(50, 8, 4, 4);
}
