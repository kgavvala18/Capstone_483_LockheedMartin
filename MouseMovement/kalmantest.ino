/**************************************
 * Kalman Filter Implementation
 **************************************/
class KalmanFilter {
  public:
    float Q;   // Process noise covariance
    float R;   // Measurement noise covariance
    float x;   // Filtered value (state)
    float P;   // Estimation error covariance

    KalmanFilter(float processNoise, float measurementNoise, float initialValue) {
      Q = processNoise;
      R = measurementNoise;
      x = initialValue;
      P = 1.0;
    }

    float update(float measurement) {
      // Prediction update: assume constant state
      P = P + Q;
      // Measurement update:
      float K = P / (P + R);       // Kalman gain
      x = x + K * (measurement - x);
      P = (1 - K) * P;
      return x;
    }
};

// Create one filter for each axis (tune Q and R as needed)
KalmanFilter kf_x(0.01, 0.1, 0.0);
KalmanFilter kf_y(0.01, 0.1, 0.0);
KalmanFilter kf_z(0.01, 0.1, 0.0);

/**************************************
 * Existing includes, sensor and BLE declarations
 **************************************/
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>
#include <bluefruit.h>

// sensor objects
Adafruit_APDS9960 apds9960; 
Adafruit_BMP280 bmp280;     
Adafruit_LIS3MDL lis3mdl;   
Adafruit_LSM6DS3TRC lsm6ds3trc; 
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_SHT31 sht30;       

BLEDis bledis;
BLEHidAdafruit blehid;

#define MOVE_STEP    1000
#define SCROLL_STEP    1
#define DAMPING 0.9 
#define SCOLL_THRESHOLD 1.0 
#define POSITION_THRESHOLD 0.001 

// Integration variables (now accumulating)
float velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0;
float position_x = 0.0, position_y = 0.0, position_z = 0.0;
unsigned long prev_time = 0;

// Gravity offset values
float gravity_x = 0.0, gravity_y = 0.0, gravity_z = 0.0;

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
long int check_array[6]={0, 0, 0, 0, 0, 0};
float last_time;

extern PDMClass PDM;
short sampleBuffer[256];  
volatile int samplesRead; 

bool new_rev = true;

void setup(void) {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  Serial.println("Feather Sense Sensor Demo");

  // initialize the sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  bmp280.begin();
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  // read sensor to initialize arrays
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

  // set gravity offset using initial acceleration
  gravity_x = accel.acceleration.x;
  gravity_y = accel.acceleration.y;
  gravity_z = accel.acceleration.z;

  for (int i = 0; i < 5; i++) {
    if (accel_array[i] != check_array[i]) {
      new_rev = false;
      break;
    }
  }
  if (new_rev) {
    lsm6ds3trc.begin_I2C();
  }
  sht30.begin();

  // Mouse and BLE setup
  Bluefruit.begin();
  Bluefruit.Periph.setConnInterval(9, 16);
  Bluefruit.setTxPower(4);
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();
  blehid.begin();
  startAdv();

  // Initialize time for integration
  last_time = millis();
}

void loop(void) {
  // Read magnetometer
  lis3mdl.read();
  magnetic_x = lis3mdl.x;
  magnetic_y = lis3mdl.y;
  magnetic_z = lis3mdl.z;

  // Read accelerometer and gyro data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  if (new_rev) {
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
  } else {
    lsm6ds33.getEvent(&accel, &gyro, &temp);
  }

  // Remove gravity bias from acceleration reading
  accel_x = accel.acceleration.x - gravity_x;
  accel_y = accel.acceleration.y - gravity_y;
  accel_z = accel.acceleration.z - gravity_z;
  gyro_x = gyro.gyro.x;
  gyro_y = gyro.gyro.y;
  gyro_z = gyro.gyro.z;

  // Calculate elapsed time in seconds
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  // --- Kalman filtering for each acceleration axis ---
  float filtered_accel_x = kf_x.update(accel_x);
  float filtered_accel_y = kf_y.update(accel_y);
  float filtered_accel_z = kf_z.update(accel_z);

  // --- Integration: update velocity and position ---
  velocity_x += filtered_accel_x * dt;
  velocity_y += filtered_accel_y * dt;
  velocity_z += filtered_accel_z * dt;

  // Apply damping to reduce drift over time
  velocity_x *= DAMPING;
  velocity_y *= DAMPING;
  velocity_z *= DAMPING;

  position_x += velocity_x * dt;
  position_y += velocity_y * dt;
  position_z += velocity_z * dt;

  // Debug prints to see filtered and integrated values
  Serial.print("Filtered Accel X: "); Serial.println(filtered_accel_x);
  Serial.print("Velocity X: "); Serial.println(velocity_x);
  Serial.print("Position X: "); Serial.println(position_x);
  Serial.print("Filtered Accel Y: "); Serial.println(filtered_accel_y);
  Serial.print("Velocity Y: "); Serial.println(velocity_y);
  Serial.print("Position Y: "); Serial.println(position_y);
  Serial.print("Filtered Accel Z: "); Serial.println(filtered_accel_z);
  Serial.print("Velocity Z: "); Serial.println(velocity_z);
  Serial.print("Position Z: "); Serial.println(position_z);

  // --- Mouse movement logic based on integrated positions ---
  int move_y = 0;
  int move_z = 0;
  if (fabs(position_z) > POSITION_THRESHOLD) {
    move_z = (int)(position_z * MOVE_STEP);
  }
  if (fabs(position_y) > POSITION_THRESHOLD) {
    move_y = (int)(position_y * MOVE_STEP);
  }
  blehid.mouseMove(move_y, move_z);

  // Scroll control based on translation along X (if moved outside deadzone)
  if (fabs(position_x) > SCOLL_THRESHOLD) {
    if (position_x > 0)
      blehid.mouseScroll(SCROLL_STEP);
    else
      blehid.mouseScroll(-SCROLL_STEP);
  }

  delay(100);  // adjust delay as needed for smoother movement
}

/**************************************
 * Existing functions below remain unchanged
 **************************************/
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
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

void startAdv(void) { 
  Serial.println("BLE Advertising Started...");
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}
