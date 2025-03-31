#include <Adafruit_LSM6DS33.h>
#include <bluefruit.h>

Adafruit_LSM6DS33 lsm6ds33;
BLEDis bledis;
BLEHidAdafruit blehid;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("Starting BLE Accelerometer Data Transmission...");
    
    if (!lsm6ds33.begin_I2C()) {
        Serial.println("Failed to initialize LSM6DS33!");
        while (1);
    }
    
    Bluefruit.begin();
    Bluefruit.Periph.setConnInterval(9, 16);
    Bluefruit.setTxPower(4);
    
    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather 52");
    bledis.begin();
    
    blehid.begin();
    startAdv();
}

void loop() {
    sensors_event_t accel, gyro, temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
    
    float accel_x = accel.acceleration.x;
    float accel_y = accel.acceleration.y;
    float accel_z = accel.acceleration.z;
    
    Serial.print("Accel X: "); Serial.print(accel_x);
    Serial.print(" Y: "); Serial.print(accel_y);
    Serial.print(" Z: "); Serial.println(accel_z);
    
    uint8_t accel_data[6];
    memcpy(accel_data, &accel_x, sizeof(float));
    memcpy(accel_data + sizeof(float), &accel_y, sizeof(float));
    
    blehid.rawReport(BLE_CONN_HANDLE_INVALID, accel_data, sizeof(accel_data));
    
    delay(100); // Adjust transmission rate
}

void startAdv() {
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.start(0);
}
