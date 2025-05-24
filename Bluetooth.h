#include <BluetoothSerial.h>
#include <Commands.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define BT_DISCOVER_TIME 10000

namespace Bluetooth {
  BluetoothSerial SerialBT;
  static bool btScanAsync = true;
  static bool btScanSync = true;
  bool inMessageIn = false;
  bool inMessageOut = false;
  String messageIn = "";
  String messageOut = "";

  int sensores[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  unsigned long temp;

  void btAdvertisedDeviceFound(BTAdvertisedDevice *pDevice) {
    Serial.printf("Found a device asynchronously: %s\n", pDevice->toString().c_str());
  }

  void setupBT() {
    SerialBT.begin("Nanolito");  //Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");
    temp = millis();
  }

  void sendSensorData()
  {
    unsigned long t = millis();
    if (t - temp > 1000)
    {
      String msg = "B:";
      for(int i{0}; i < 11; i++)
      {
        msg += String(random(1024)) + ":";
      }
      SerialBT.print(msg);
      temp = t;
    }
  }

  void readBT()
  {
    if (SerialBT.available()) {
      inMessageIn = true;
      char data = SerialBT.read();
      Serial.flush();
      if (data == '\n')
      {
        if(messageIn == "ping")
        {
          SerialBT.print(getMessage('A'));
        }

        processMessage(messageIn);

        Serial.println(messageIn);
        inMessageIn = false;
        messageIn = "";
      }
      else
      {
        messageIn += String(data);
      }
    }

    if (Serial.available()) {
      inMessageOut = true;
      char data = Serial.read();
      if (data == '\n')
      {
        SerialBT.print(messageOut);
        inMessageOut = false;
        messageOut = "";
      }
      else
      {
        messageOut += String(data);
      }
    }
    sendSensorData();
  }
}