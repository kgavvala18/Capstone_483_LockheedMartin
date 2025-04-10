/*this is an implementation of mouse control using the gyroscope of the LSM6DSOX IMU
An Extended Kalman Filter is used to smooth the acceleration values
which are scaled and used directly for control input
the KB2040 board was used to test this implmentation using usb hid and we are still
converting this to use on the feather nRF52840 sense board with BLe HID
  */

#define EKF_N 4 //  States: Y angular Velocity, Y angACC, Z angVel, Z angACC,
#define EKF_M 2 //  Measurements Y angAcc, Z angAcc

#include <bluefruit.h>
// #include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <EKF.h>
// #include <Mouse.h>
#include <math.h>
// #include <Adafruit_TinyUSB.h>s

BLEDis bledis;
BLEHidAdafruit blehid;

static const float EPS = 1.5e-6;

static const float Q[EKF_N * EKF_N] = {

    EPS, 0, 0, 0,
    0, EPS * 10, 0, 0,
    0, 0, EPS, 0,
    0, 0, 0, EPS * 10};

static const float R[EKF_M * EKF_M] = {
    // 2x2

    0.0005,
    0,
    0,
    0.0005,

};

// So process model Jacobian is identity matrix
// static const float F[EKF_N*EKF_N] = {   //
//     1, 1, 0, 0,
//     0, 1, 0, 0,
//     0, 0, 1, 1,
//     1, 1, 0, 1
// };

static const float H[EKF_M * EKF_N] = {
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    1,
};

static ekf_t _ekf;

Adafruit_LSM6DSOX sox;
unsigned long myTime;
float prev;
float curr;
float dt = 0.01;

// float F[EKF_N*EKF_N];

// the setup routine runs once when you press reset:
void setup()
{

  const float Pdiag[EKF_N] = {0.001, 0.001, 0.001, 0.001};

  ekf_initialize(&_ekf, Pdiag);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Motion Control Glove - Starting");

  // if (!sox.begin_I2C()) {
  if (!sox.begin_I2C(0X6B))
  {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSOX chip");
    while (1)
    {
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

  Bluefruit.begin();
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16); // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  Bluefruit.setTxPower(4);                 // Check bluefruit.h for supported values

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // BLE HID
  blehid.begin();

  // Set up and start advertising
  startAdv();
}

// the loop routine runs over and over again forever:
void loop()
{
  curr = millis();

  dt = (curr - prev) / 1000.0000;
  prev = curr;

  sensors_event_t accel;
  sensors_event_t gyro;
  sox.getEvent(&accel, &gyro);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y + 0.32;
  float az = accel.acceleration.z - 10.03;

  float gx = gyro.gyro.x;
  float gy = 0.01 + gyro.gyro.y;
  float gz = 0.01 + gyro.gyro.z;

  const float z[EKF_M] = {gy, gz};

  const float F[EKF_N * EKF_N] = {
      1, dt, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, dt,
      0, 0, 0, 1};
  // Process model is f(x) = x
  const float fx[EKF_N] = {_ekf.x[0] + dt * _ekf.x[1], _ekf.x[1], _ekf.x[2] + dt * _ekf.x[3], _ekf.x[3]}; // velocity y , velocity z

  // Run the prediction step of the EKF
  ekf_predict(&_ekf, fx, F, Q);

  const float hx[EKF_M] = {_ekf.x[1], _ekf.x[3]};
  //   hx[2] = .9987 * this->x[1] + .001;

  // const float hx[EKF_M] = {_ekf.x[0], _ekf.x[1] };

  // Run the update step of EKF
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

  uint8_t report_id = 0;
  uint8_t buttons = 0;

  int8_t x = mz;
  int8_t y = my;

  blehid.mouseMove(x, y);

  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.print(",");

  Serial.print(_ekf.x[1]);
  Serial.print(",");
  Serial.print(_ekf.x[3]);
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
