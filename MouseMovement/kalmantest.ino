#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>
#include <bluefruit.h>
#include <math.h>

// ----- Kalman Filter Class -----
class KalmanFilter {
  public:
    float Q;   // Process noise covariance
    float R;   // Measurement noise covariance
    float x;   // Filtered state estimate
    float P;   // Estimation error covariance

    KalmanFilter(float processNoise, float measurementNoise, float initialValue) {
      Q = processNoise;
      R = measurementNoise;
      x = initialValue;
      P = 1.0;
    }

    float update(float measurement) {
      // Prediction update
      P = P + Q;
      // Measurement update
      float K = P / (P + R); // Kalman gain
      x = x + K * (measurement - x);
      P = (1 - K) * P;
      return x;
    }
};

// Create one Kalman filter for each axis (tune Q and R as needed)
KalmanFilter kf_x(0.01, 0.1, 0.0);
KalmanFilter kf_y(0.01, 0.1, 0.0);
KalmanFilter kf_z(0.01, 0.1, 0.0);

// ----- Sensor Objects -----
Adafruit_APDS9960 apds9960;
Adafruit_BMP280 bmp280;     
Adafruit_LIS3MDL lis3mdl;   
Adafruit_LSM6DS3TRC lsm6ds3trc; 
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_SHT31 sht30;

BLEDis bledis;
BLEHidAdafruit blehid;

// ----- Adjustable configurations -----
#define MOVE_STEP         1000
#define SCROLL_STEP       1
#define DAMPING           0.9
#define SCOLL_THRESHOLD   1.0
#define POSITION_THRESHOLD 0.001

// Complementary filter coefficient for orientation fusion
const float comp_alpha = 0.98;

// Integration variables for translation (in world frame)
float velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0;
float position_x = 0.0, position_y = 0.0, position_z = 0.0;
unsigned long last_time = 0;

// Gravity estimation (initially set; will be removed by sensor fusion)
float gravity_x = 0.0, gravity_y = 0.0, gravity_z = 0.0;

uint8_t proximity;
uint16_t r, g, b, c;
float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;
long int accel_array[6];
long int check_array[6]={0, 0, 0, 0, 0, 0};

bool new_rev = true;

// For microphone
extern PDMClass PDM;
short sampleBuffer[256];  
volatile int samplesRead; 

// Variables for sensor fusion (orientation in radians)
float pitch = 0.0, roll = 0.0, yaw = 0.0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Feather Sense Sensor Demo");

  // Sensor initialization
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  bmp280.begin();
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  sensors_event_t accel, gyro, temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  accel_array[0] = accel.acceleration.x;
  accel_array[1] = accel.acceleration.y;
  accel_array[2] = accel.acceleration.z;
  accel_array[3] = gyro.gyro.x;
  accel_array[4] = gyro.gyro.y;
  accel_array[5] = gyro.gyro.z;

  // Use initial accelerometer reading for gravity offset
  gravity_x = accel.acceleration.x;
  gravity_y = accel.acceleration.y;
  gravity_z = accel.acceleration.z;
  
  // Check sensor revision
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

  last_time = millis();
}

void loop(void) {
  // --- Read Sensor Data ---
  sensors_event_t accel, gyro, temp;
  if (new_rev) {
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
  } else {
    lsm6ds33.getEvent(&accel, &gyro, &temp);
  }
  // Read magnetometer (for yaw)
  lis3mdl.read();
  magnetic_x = lis3mdl.x;
  magnetic_y = lis3mdl.y;
  magnetic_z = lis3mdl.z;
  
  // Read BMP altitude for vertical correction
  float bmp_alt = bmp280.readAltitude(1013.25);

  // --- Compute time delta ---
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  // --- Sensor Fusion: Estimate Orientation ---
  // Compute accelerometer angles (in radians)
  float accel_pitch = atan2(accel.acceleration.y, sqrt(accel.acceleration.x * accel.acceleration.x + accel.acceleration.z * accel.acceleration.z));
  float accel_roll  = atan2(-accel.acceleration.x, accel.acceleration.z);

  // Update orientation with complementary filter using gyro (gyroscope values in rad/s assumed)
  // Note: if your gyro outputs degrees/s, convert them to radians/s first.
  pitch = comp_alpha * (pitch + gyro.gyro.y * dt) + (1.0 - comp_alpha) * accel_pitch;
  roll  = comp_alpha * (roll  + gyro.gyro.x * dt) + (1.0 - comp_alpha) * accel_roll;
  yaw   = atan2(magnetic_y, magnetic_x);  // Yaw from magnetometer (in radians)

  // --- Build Rotation Matrix (from body frame to world frame) ---
  float cosY = cos(yaw);
  float sinY = sin(yaw);
  float cosP = cos(pitch);
  float sinP = sin(pitch);
  float cosR = cos(roll);
  float sinR = sin(roll);

  // Using ZYX Euler angles (yaw, pitch, roll)
  float R11 = cosY * cosP;
  float R12 = cosY * sinP * sinR - sinY * cosR;
  float R13 = cosY * sinP * cosR + sinY * sinR;
  float R21 = sinY * cosP;
  float R22 = sinY * sinP * sinR + cosY * cosR;
  float R23 = sinY * sinP * cosR - cosY * sinR;
  float R31 = -sinP;
  float R32 = cosP * sinR;
  float R33 = cosP * cosR;

  // --- Transform Accelerometer Reading into World Frame ---
  // Raw accelerometer (body frame)
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  // Rotate into world frame
  float world_ax = R11 * ax + R12 * ay + R13 * az;
  float world_ay = R21 * ax + R22 * ay + R23 * az;
  float world_az = R31 * ax + R32 * ay + R33 * az;

  // --- Remove Gravity from World Frame Acceleration ---
  // Assuming gravity = 9.81 m/s^2 downward (z-axis)
  float lin_ax = world_ax;
  float lin_ay = world_ay;
  float lin_az = world_az - 9.81;

  // --- Kalman Filter on Linear Acceleration ---
  float filt_lin_ax = kf_x.update(lin_ax);
  float filt_lin_ay = kf_y.update(lin_ay);
  float filt_lin_az = kf_z.update(lin_az);

  // --- Integration: Update Velocity and Position ---
  velocity_x += filt_lin_ax * dt;
  velocity_y += filt_lin_ay * dt;
  velocity_z += filt_lin_az * dt;

  // Damping to reduce drift
  velocity_x *= DAMPING;
  velocity_y *= DAMPING;
  velocity_z *= DAMPING;

  position_x += velocity_x * dt;
  position_y += velocity_y * dt;
  position_z += velocity_z * dt;

  // --- Correct Vertical Drift with BMP Altitude ---
  // Blend integrated vertical position with BMP altitude reading
  position_z = comp_alpha * position_z + (1.0 - comp_alpha) * bmp_alt;

  // --- Debug Output ---
  Serial.print("Pitch: "); Serial.println(pitch * 57.2958); // in degrees
  Serial.print("Roll:  "); Serial.println(roll * 57.2958);
  Serial.print("Yaw:   "); Serial.println(yaw * 57.2958);
  Serial.print("Filtered Linear Accel (X,Y,Z): ");
  Serial.print(filt_lin_ax); Serial.print(", ");
  Serial.print(filt_lin_ay); Serial.print(", ");
  Serial.println(filt_lin_az);
  Serial.print("Velocity (X,Y,Z): ");
  Serial.print(velocity_x); Serial.print(", ");
  Serial.print(velocity_y); Serial.print(", ");
  Serial.println(velocity_z);
  Serial.print("Position (X,Y,Z): ");
  Serial.print(position_x); Serial.print(", ");
  Serial.print(position_y); Serial.print(", ");
  Serial.println(position_z);

  // --- Mouse Movement Logic ---
  int move_y = 0;
  int move_z = 0;
  if (fabs(position_z) > POSITION_THRESHOLD) {
    move_z = (int)(position_z * MOVE_STEP);
  }
  if (fabs(position_y) > POSITION_THRESHOLD) {
    move_y = (int)(position_y * MOVE_STEP);
  }
  blehid.mouseMove(move_y, move_z);

  // Scroll control based on X-axis translation (if moved outside deadzone)
  if (fabs(position_x) > SCOLL_THRESHOLD) {
    if (position_x > 0)
      blehid.mouseScroll(SCROLL_STEP);
    else
      blehid.mouseScroll(-SCROLL_STEP);
  }
  
  delay(100); // Adjust delay for smooth operation
}

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
