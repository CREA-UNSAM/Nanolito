#include <Bluetooth.h>

BluetoothSerial SerialBT();
bool inMessageIn = false;
bool inMessageOut = false;
String messageIn = "";
String messageOut = "";

unsigned long temp;

void btAdvertisedDeviceFound(BTAdvertisedDevice *pDevice) 
{
  Serial.printf("Found a device asynchronously: %s\n", pDevice->toString().c_str());
}

void Bluetooth::setupBT() 
{
  SerialBT.begin("Nanolito");  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  temp = millis();
}

void Bluetooth::sendSensorData()
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

void Bluetooth::readBT()
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