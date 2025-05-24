#include <Globals.h>
#include <Bluetooth.h>
#include <WiFi.h>

void IOHandleTask(void* params)
{
  while(true)
  {
    Bluetooth::readBT();

    Serial.print("kp: ");
    Serial.print(pid.kp);
    Serial.print("|ki: ");
    Serial.print(pid.ki);
    Serial.print("|kd: ");
    Serial.println(pid.kd);

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void ControlTask(void* params)
{
  while(true)
  {
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.begin(9600);

  loadGlobals();
  Bluetooth::setupBT();

  xTaskCreatePinnedToCore(IOHandleTask, "IO", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ControlTask, "PIDControl", 4096, NULL, 1, NULL, 1);
}

void loop(){}
