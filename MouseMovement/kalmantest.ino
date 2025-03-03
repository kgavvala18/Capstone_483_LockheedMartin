#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>
#include <bluefruit.h>
#include <math.h>

// ---------------------
// Global Calibration Variables
// ---------------------
bool calibrated = false;
float calibRot[3][3];      // Calibration matrix computed from accelerometer (to align gravity)
float mag_calibRot[3][3];  // Calibration matrix computed from magnetometer (to align x-axis)

// ---------------------
// Utility Functions
// ---------------------
float vectorMagnitude(const float v[3]) {
  return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void normalizeVector(const float v[3], float result[3]) {
  float mag = vectorMagnitude(v);
  result[0] = v[0] / mag;
  result[1] = v[1] / mag;
  result[2] = v[2] / mag;
}

float dotProduct(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void crossProduct(const float a[3], const float b[3], float result[3]) {
  result[0] = a[1]*b[2] - a[2]*b[1];
  result[1] = a[2]*b[0] - a[0]*b[2];
  result[2] = a[0]*b[1] - a[1]*b[0];
}

float calculateAngle(const float a[3], const float b[3]) {
  float dot = dotProduct(a, b);
  float magA = vectorMagnitude(a);
  float magB = vectorMagnitude(b);
  float cosTheta = dot / (magA * magB);
  if (cosTheta > 1) cosTheta = 1;
  if (cosTheta < -1) cosTheta = -1;
  return acos(cosTheta);
}

// Standard axis-angle rotation matrix (Rodrigues’ rotation formula)
void calculateRotationMatrix(const float axis[3], float angle, float R[3][3]) {
  float normAxis[3];
  normalizeVector(axis, normAxis);
  float kx = normAxis[0], ky = normAxis[1], kz = normAxis[2];
  float c = cos(angle);
  float s = sin(angle);
  float v = 1 - c;
  R[0][0] = c + kx*kx*v;
  R[0][1] = kx*ky*v - kz*s;
  R[0][2] = kx*kz*v + ky*s;
  R[1][0] = ky*kx*v + kz*s;
  R[1][1] = c + ky*ky*v;
  R[1][2] = ky*kz*v - kx*s;
  R[2][0] = kz*kx*v - ky*s;
  R[2][1] = kz*ky*v + kx*s;
  R[2][2] = c + kz*kz*v;
}

// Multiply a 3x3 matrix M by a 3-vector v, result in res[3]
void multiplyMatrixVector(const float M[3][3], const float v[3], float res[3]) {
  for (int i = 0; i < 3; i++) {
    res[i] = M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2];
  }
}

// ---------------------
// Kalman Filter Class
// ---------------------
class KalmanFilter {
  public:
    float Q;   // Process noise covariance
    float R;   // Measurement noise covariance
    float x;   // State estimate
    float P;   // Estimation error covariance

    KalmanFilter(float processNoise, float measurementNoise, float initialValue) {
      Q = processNoise;
      R = measurementNoise;
      x = initialValue;
      P = 1.0;
    }

    float update(float measurement) {
      P = P + Q;
      float K = P / (P + R);
      x = x + K * (measurement - x);
      P = (1 - K) * P;
      return x;
    }
};

KalmanFilter kf_x(0.01, 0.1, 0.0);
KalmanFilter kf_y(0.01, 0.1, 0.0);
KalmanFilter kf_z(0.01, 0.1, 0.0);

// ---------------------
// Sensor Objects
// ---------------------
Adafruit_APDS9960 apds9960;
Adafruit_BMP280 bmp280;     
Adafruit_LIS3MDL lis3mdl;   
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_SHT31 sht30;

BLEDis bledis;
BLEHidAdafruit blehid;

// ---------------------
// Configurations and Globals
// ---------------------
#define MOVE_STEP          1000
#define SCROLL_STEP        1
#define DAMPING            0.9
#define SCOLL_THRESHOLD    1.0
#define POSITION_THRESHOLD 0.001

const float comp_alpha = 0.98; // Complementary filter coefficient

float velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0;
float position_x = 0.0, position_y = 0.0, position_z = 0.0;
unsigned long last_time = 0;

// Orientation variables (in radians)
float pitch = 0.0, roll = 0.0, yaw = 0.0;

// ---------------------
// Calibration Routine
// ---------------------
void calibrateSensors() {
  const int numSamples = 100;
  float accSum[3] = {0, 0, 0};
  float magSum[3] = {0, 0, 0};
  
  Serial.println("Gathering calibration samples...");
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t accel, gyro, temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
    accSum[0] += accel.acceleration.x;
    accSum[1] += accel.acceleration.y;
    accSum[2] += accel.acceleration.z;
    
    lis3mdl.read();
    magSum[0] += lis3mdl.x;
    magSum[1] += lis3mdl.y;
    magSum[2] += lis3mdl.z;
    
    delay(20);
  }
  
  float accAvg[3] = { accSum[0]/numSamples, accSum[1]/numSamples, accSum[2]/numSamples };
  float magAvg[3] = { magSum[0]/numSamples, magSum[1]/numSamples, magSum[2]/numSamples };
  
  Serial.println("Calibration Averages:");
  Serial.print("Acc: ");
  Serial.print(accAvg[0]); Serial.print(", ");
  Serial.print(accAvg[1]); Serial.print(", ");
  Serial.println(accAvg[2]);
  Serial.print("Mag: ");
  Serial.print(magAvg[0]); Serial.print(", ");
  Serial.print(magAvg[1]); Serial.print(", ");
  Serial.println(magAvg[2]);
  
  // Reference vectors
  float gravityRef[3] = {0, 0, -1};  // Gravity points down in world frame
  float xRef[3] = {1, 0, 0};          // Desired magnetometer x-axis
  
  // Normalize measured vectors
  float normAcc[3];
  normalizeVector(accAvg, normAcc);
  float normMag[3];
  normalizeVector(magAvg, normMag);
  
  // --- First Rotation: Align accelerometer (gravity) ---
  float angleAcc = calculateAngle(normAcc, gravityRef);
  float adjustedAngle = 2 * PI - angleAcc;
  float rotAxis[3];
  crossProduct(normAcc, gravityRef, rotAxis);
  calculateRotationMatrix(rotAxis, adjustedAngle, calibRot);
  
  // --- Second Rotation: Align magnetometer ---
  float rotatedMag[3];
  multiplyMatrixVector(calibRot, normMag, rotatedMag);
  // Project onto X-Y plane and normalize
  float projMag[3] = { rotatedMag[0], rotatedMag[1], 0 };
  float projMagNorm[3];
  normalizeVector(projMag, projMagNorm);
  
  float angleMag = calculateAngle(projMagNorm, xRef);
  float adjustedAngleMag = 2 * PI - angleMag;
  float magRotAxis[3];
  crossProduct(projMagNorm, xRef, magRotAxis);
  calculateRotationMatrix(magRotAxis, adjustedAngleMag, mag_calibRot);
  
  Serial.println("Calibration complete.");
  Serial.println("Calibration Matrix (Acc):");
  for (int i = 0; i < 3; i++) {
    Serial.print(calibRot[i][0]); Serial.print(" ");
    Serial.print(calibRot[i][1]); Serial.print(" ");
    Serial.println(calibRot[i][2]);
  }
  Serial.println("Mag Calibration Matrix:");
  for (int i = 0; i < 3; i++) {
    Serial.print(mag_calibRot[i][0]); Serial.print(" ");
    Serial.print(mag_calibRot[i][1]); Serial.print(" ");
    Serial.println(mag_calibRot[i][2]);
  }
  
  calibrated = true;
}

// ---------------------
// BLE Advertising Setup
// ---------------------
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

// ---------------------
// Setup
// ---------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Feather Sense Sensor Demo (Calibrated)");

  // Initialize sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  bmp280.begin();
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  sht30.begin();

  // Initialize BLE and mouse functionality
  Bluefruit.begin();
  Bluefruit.Periph.setConnInterval(9, 16);
  Bluefruit.setTxPower(4);
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();
  blehid.begin();
  startAdv();

  // Perform calibration (ensure the board is steady during this period)
  Serial.println("Calibrating sensors... Please keep the board steady.");
  calibrateSensors();
  
  last_time = millis();
}

// ---------------------
// Main Loop: Sensor Fusion & Mouse Control
// ---------------------
void loop() {
  if (!calibrated) return;  // Wait for calibration (should be done in setup)

  // Read accelerometer data
  sensors_event_t accel, gyro, temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  float acc_raw[3] = { accel.acceleration.x, accel.acceleration.y, accel.acceleration.z };

  // Apply calibration to accelerometer
  float acc_cal[3];
  multiplyMatrixVector(calibRot, acc_raw, acc_cal);

  // Read magnetometer data
  lis3mdl.read();
  float mag_raw[3] = { lis3mdl.x, lis3mdl.y, lis3mdl.z };
  // Apply calibration: first apply calibRot then mag_calibRot
  float tempMag[3];
  multiplyMatrixVector(calibRot, mag_raw, tempMag);
  float mag_cal[3];
  multiplyMatrixVector(mag_calibRot, tempMag, mag_cal);

  // Read BMP altitude for vertical correction
  float bmp_alt = bmp280.readAltitude(1013.25);

  // Compute time delta
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  // --- Sensor Fusion: Estimate Orientation ---
  // Compute pitch and roll from calibrated accelerometer data
  float accel_pitch = atan2(acc_cal[1], sqrt(acc_cal[0]*acc_cal[0] + acc_cal[2]*acc_cal[2]));
  float accel_roll  = atan2(-acc_cal[0], acc_cal[2]);
  
  // Complementary filter with gyroscope data (assumed in rad/s)
  pitch = comp_alpha * (pitch + gyro.gyro.y * dt) + (1.0 - comp_alpha) * accel_pitch;
  roll  = comp_alpha * (roll  + gyro.gyro.x * dt) + (1.0 - comp_alpha) * accel_roll;
  // Yaw estimated from calibrated magnetometer data
  yaw = atan2(mag_cal[1], mag_cal[0]);

  // --- Build Rotation Matrix from Orientation (ZYX Euler angles) ---
  float cosY = cos(yaw), sinY = sin(yaw);
  float cosP = cos(pitch), sinP = sin(pitch);
  float cosR = cos(roll), sinR = sin(roll);
  float R11 = cosY * cosP;
  float R12 = cosY * sinP * sinR - sinY * cosR;
  float R13 = cosY * sinP * cosR + sinY * sinR;
  float R21 = sinY * cosP;
  float R22 = sinY * sinP * sinR + cosY * cosR;
  float R23 = sinY * sinP * cosR - cosY * sinR;
  float R31 = -sinP;
  float R32 = cosP * sinR;
  float R33 = cosP * cosR;

  // Transform calibrated accelerometer reading into world frame
  float world_ax = R11 * acc_cal[0] + R12 * acc_cal[1] + R13 * acc_cal[2];
  float world_ay = R21 * acc_cal[0] + R22 * acc_cal[1] + R23 * acc_cal[2];
  float world_az = R31 * acc_cal[0] + R32 * acc_cal[1] + R33 * acc_cal[2];

  // Remove gravity (assume 9.81 m/s² along world z-axis)
  float lin_ax = world_ax;
  float lin_ay = world_ay;
  float lin_az = world_az - 9.81;

  // --- Kalman Filtering on Linear Acceleration ---
  float filt_lin_ax = kf_x.update(lin_ax);
  float filt_lin_ay = kf_y.update(lin_ay);
  float filt_lin_az = kf_z.update(lin_az);

  // --- Integration: Update Velocity and Position ---
  velocity_x += filt_lin_ax * dt;
  velocity_y += filt_lin_ay * dt;
  velocity_z += filt_lin_az * dt;

  velocity_x *= DAMPING;
  velocity_y *= DAMPING;
  velocity_z *= DAMPING;

  position_x += velocity_x * dt;
  position_y += velocity_y * dt;
  position_z += velocity_z * dt;

  // Correct vertical drift using BMP altitude (blended with complementary filter)
  position_z = comp_alpha * position_z + (1.0 - comp_alpha) * bmp_alt;

  // --- Debug Output ---
  Serial.print("Pitch: "); Serial.println(pitch * 57.2958); // degrees
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
  int move_y = 0, move_z = 0;
  if (fabs(position_z) > POSITION_THRESHOLD) {
    move_z = (int)(position_z * MOVE_STEP);
  }
  if (fabs(position_y) > POSITION_THRESHOLD) {
    move_y = (int)(position_y * MOVE_STEP);
  }
  blehid.mouseMove(move_y, move_z);
  
  // Scroll control based on X-axis translation
  if (fabs(position_x) > SCOLL_THRESHOLD) {
    if (position_x > 0)
      blehid.mouseScroll(SCROLL_STEP);
    else
      blehid.mouseScroll(-SCROLL_STEP);
  }

  delay(100); // Adjust delay for smooth operation
}
