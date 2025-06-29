#include <Globals.h>
#include <Bluetooth.h>
extern "C" {
  #include "esp_wifi.h"
}

void ioHandleTask(void* params)
{
  while(true)
  {
    Bluetooth::readBT();

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void controlTask(void* params)
{
  while(true)
  {
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);
  esp_wifi_deinit();
  loadGlobals();
  Bluetooth::setupBT();

  xTaskCreatePinnedToCore(ioHandleTask, "IO", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlTask, "PIDControl", 4096, NULL, 1, NULL, 1);
}

void loop(){}
