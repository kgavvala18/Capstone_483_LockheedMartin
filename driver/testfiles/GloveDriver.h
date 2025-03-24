#pragma once
#include <windows.devices.bluetooth.h>
#include <windows.devices.bluetooth.genericattributeprofile.h>
#include <wrl.h>

using namespace Microsoft::WRL;
using namespace Windows::Devices::Bluetooth;
using namespace Windows::Devices::Bluetooth::GenericAttributeProfile;

class BluetoothGloveDriver {
public:
    BluetoothGloveDriver();
    bool connectToGlove();
    void readSensorData();

private:
    ComPtr<BluetoothLEDevice> gloveDevice;
    GattCharacteristic gloveCharacteristic;
};
