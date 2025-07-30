#ifndef BLUETOOTH
#define BLUETOOTH

//#include <BluetoothSerial.h>
#include <Commands.h>
#include <Globals.h>


namespace Bluetooth {

  void setupBT();

  void sendSensorData();

  void readBT();
}

#endif