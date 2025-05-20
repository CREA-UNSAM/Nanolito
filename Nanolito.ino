#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define BT_DISCOVER_TIME 10000

static bool btScanAsync = true;
static bool btScanSync = true;
bool inMessageIn = false;
bool inMessageOut = false;
String messageIn = "";
String messageOut = "";

void btAdvertisedDeviceFound(BTAdvertisedDevice *pDevice) {
  Serial.printf("Found a device asynchronously: %s\n", pDevice->toString().c_str());
}

void setup() {
  Serial.begin(9600);

  SerialBT.begin("Nanolito");  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (SerialBT.available()) {
    inMessageIn = true;
    char data = SerialBT.read();
    if (data == '\n')
    {
      if(messageIn == "ping")
      {
        SerialBT.print("pong");
      }

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
  delay(20);
}
