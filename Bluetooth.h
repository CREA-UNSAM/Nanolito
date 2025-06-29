#ifndef BLUETOOTH
#define BLUETOOTH

#include <BluetoothSerial.h>
#include <Commands.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define BT_DISCOVER_TIME 10000

extern BluetoothSerial SerialBT;
static bool btScanAsync;
static bool btScanSync;
extern bool inMessageIn;
extern bool inMessageOut;
extern String messageIn;
extern String messageOut;

extern int sensores[11];
extern unsigned long temp;


namespace Bluetooth {
  
  void btAdvertisedDeviceFound(BTAdvertisedDevice *pDevice);

  void setupBT();

  void sendSensorData();

  void readBT();
}

#endif