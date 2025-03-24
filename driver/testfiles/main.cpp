#include <iostream>
#include "GloveDriver.h"

int main() {
    std::cout << "Starting Bluetooth Glove Driver..." << std::endl;
    
    BluetoothGloveDriver glove;
    
    if (glove.connectToGlove()) {
        std::cout << "Connected to Glove. Reading sensor data..." << std::endl;

        while (true) {
            glove.readSensorData();
            Sleep(100);  // Adjust update rate
        }
    } else {
        std::cerr << "Failed to connect to the glove." << std::endl;
    }

    return 0;
}
