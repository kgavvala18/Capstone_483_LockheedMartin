#include "GloveDriver.h"
#include <iostream>

BluetoothGloveDriver::BluetoothGloveDriver() {}

bool BluetoothGloveDriver::connectToGlove() {
    auto deviceSelector = BluetoothLEDevice::GetDeviceSelector();
    auto devices = Windows::Devices::Enumeration::DeviceInformation::FindAllAsync(deviceSelector).get();

    for (auto& device : devices) {
        if (device.Name == "GloveMouse") {  // Replace with actual name
            gloveDevice = BluetoothLEDevice::FromIdAsync(device.Id()).get();
            auto services = gloveDevice->GetGattServicesAsync().get();

            for (auto& service : services.Services) {
                auto characteristics = service.GetCharacteristicsAsync().get();
                for (auto& characteristic : characteristics.Characteristics) {
                    if (characteristic.CharacteristicProperties & GattCharacteristicProperties::Read) {
                        gloveCharacteristic = characteristic;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void BluetoothGloveDriver::readSensorData() {
    if (!gloveDevice) return;
    
    auto result = gloveCharacteristic.ReadValueAsync().get();
    if (result.Status == GattCommunicationStatus::Success) {
        auto reader = Windows::Storage::Streams::DataReader::FromBuffer(result.Value);
        float accelX = reader.ReadSingle();
        float accelY = reader.ReadSingle();
        float accelZ = reader.ReadSingle();
        std::cout << "Acceleration: X=" << accelX << " Y=" << accelY << " Z=" << accelZ << std::endl;
    } else {
        std::cerr << "Failed to read data." << std::endl;
    }
}
