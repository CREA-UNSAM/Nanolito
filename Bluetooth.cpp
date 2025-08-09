#include <Bluetooth.h>

//BluetoothSerial SerialBT;
//HardwareSerial espSerial(2);
bool inMessageIn = false;
bool inMessageOut = false;
String messageIn = "";
String messageOut = "";

unsigned long temp;

void Bluetooth::setupBT() 
{
  //SerialBT.begin("Nanolito");  //Bluetooth device name
  //Serial.println("The device started, now you can pair it with bluetooth!");
  //espSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  temp = millis();
  Serial.begin(9600);
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
    Serial.print(msg + "\n");
    temp = t;
  }
}

void Bluetooth::readBT()
{
  if (Serial.available()) {
    inMessageIn = true;
    char data = Serial.read();
    Serial.flush();
    if (data == '\n')
    {
      if(messageIn == "ping")
      {
        Serial.print(getMessage('A'));
      }

      processMessage(messageIn);

      //Serial.println(messageIn);
      inMessageIn = false;
      messageIn = "";
    }
    else
    {
      messageIn += String(data);
    }
  }

  // if (Serial.available()) {
  //   inMessageOut = true;
  //   char data = Serial.read();
  //   if (data == '\n')
  //   {
  //     espSerial.print(messageOut);
  //     inMessageOut = false;
  //     messageOut = "";
  //   }
  //   else
  //   {
  //     messageOut += String(data);
  //   }
  // }
  sendSensorData();
}