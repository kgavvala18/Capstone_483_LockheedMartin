// The MIT License (MIT)
// [License text omitted for brevity]

#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>
#include <bluefruit.h>

Adafruit_LIS3MDL lis3mdl;         // magnetometer
Adafruit_LSM6DS3TRC lsm6ds3trc;    // accelerometer, gyroscope
Adafruit_LSM6DS33 lsm6ds33;

BLEDis bledis;
BLEHidAdafruit blehid;

#define SMALL_MOVE_STEP     1
#define MEDIUM_MOVE_STEP    4
#define LARGE_MOVE_STEP     9

// Global sensor variables
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

int sensorpin0 = A0;  // sensor pin
int sensorpin1 = A1;  // sensor pin
int sensor0;  
int sensor1;  
int leftclick = 0;
int rightclick = 0;

// Variables for translation integration
float vel_x = 0.0;
float vel_y = 0.0;
unsigned long lastTime = 0;

long int accel_array[6];
long int check_array[6] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

bool new_rev = true;

void setup(void) {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native USB

  // Initialize sensors
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  
  // Check for readings from LSM6DS33
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
  
  // Determine if this is a new revision
  for (int i = 0; i < 5; i++) {
    if (accel_array[i] != check_array[i]) {
      new_rev = false;
      break;
    }
  }
  if (new_rev) {
    lsm6ds3trc.begin_I2C();
  }
  
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);

  // Mouse Setup
  Bluefruit.begin();
  // Set connection interval (in 0.625ms units)
  Bluefruit.Periph.setConnInterval(9, 16);
  Bluefruit.setTxPower(4);

  // Configure Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // Start BLE HID
  blehid.begin();

  // Set up and start advertising
  startAdv();

  // Initialize time for integration
  lastTime = millis();
}

void loop(void) {
  sensor0 = analogRead(sensorpin0);
  sensor1 = analogRead(sensorpin1);

  lis3mdl.read();
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  if (new_rev) {
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
  }
  else {
    lsm6ds33.getEvent(&accel, &gyro, &temp);
  }
  
  // Get current acceleration and gyro values
  accel_x = accel.acceleration.x;
  accel_y = accel.acceleration.y;
  accel_z = accel.acceleration.z;
  gyro_x = gyro.gyro.x;
  gyro_y = gyro.gyro.y;
  gyro_z = gyro.gyro.z;

  // Handle right mouse button (using sensor0)
  if (rightclick) {
    if (sensor0 < 80) {
      rightclick = 0;
      blehid.mouseButtonRelease(MOUSE_BUTTON_RIGHT);
    }
  }
  else {
    if (sensor0 > 80) {
      rightclick = 1;
      blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
      delay(100);
    }
  }

  // Handle left mouse button (using sensor1)
  if (leftclick) {
    if (sensor1 < 80) {
      leftclick = 0;
      blehid.mouseButtonRelease(MOUSE_BUTTON_LEFT);
    }
  }
  else {
    if (sensor1 > 80) {
      leftclick = 1;
      blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
      delay(100);
    }
  }

  // --- New Translation-Based Mouse Movement ---
  // Compute the time elapsed since the last loop (in seconds)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Update velocity by integrating acceleration.
  // Mapping: horizontal movement from accel_y and vertical movement from -accel_x
  vel_x += accel_y * dt;
  vel_y += -accel_x * dt;

  // Apply damping to reduce drift
  vel_x *= 0.9;
  vel_y *= 0.9;

  // Scale the integrated velocity to determine mouse movement.
  // (You can tune the scale factor below as needed.)
  int mouse_dx = (int)(vel_x * 10);
  int mouse_dy = (int)(vel_y * 10);

  // Move the mouse accordingly
  blehid.mouseMove(mouse_dx, mouse_dy);
  // --- End Translation-Based Movement ---
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
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

void startAdv(void)
{  
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
