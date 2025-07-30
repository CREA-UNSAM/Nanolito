#include <Bluetooth.h>

const int RXD2 = 3;
const int TXD2 = 1;

//BluetoothSerial SerialBT;
HardwareSerial espSerial(2);
bool inMessageIn = false;
bool inMessageOut = false;
String messageIn = "";
String messageOut = "";

unsigned long temp;

void Bluetooth::setupBT() 
{
  //SerialBT.begin("Nanolito");  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  espSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  temp = millis();
}

void Bluetooth::sendSensorData()
{
  unsigned long t = millis();
  if (t - temp > 1000)
  {
    String msg = "B:";
    for(int i{0}; i < N_SENSORES; i++)
    {
      msg += String(analogRead(sensorPins[i])) + ":";
    }
    espSerial.print(msg);
    temp = t;
  }
}

void Bluetooth::readBT()
{
  if (espSerial.available()) {
    inMessageIn = true;
    char data = espSerial.read();
    Serial.flush();
    if (data == '\n')
    {
      if(messageIn == "ping")
      {
        espSerial.print(getMessage('A'));
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
      espSerial.print(messageOut);
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