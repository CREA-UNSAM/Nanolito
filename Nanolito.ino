#include <Globals.h>
#include <Bluetooth.h>
extern "C" {
  #include "esp_wifi.h"
}

void readSensors() {
  // Leer sensores analógicos
  const uint8_t sensor_pins[] = {SENSOR_01, SENSOR_02, SENSOR_03, 
                                SENSOR_04, SENSOR_05, SENSOR_06,
                                SENSOR_07, SENSOR_08, SENSOR_09,
                                SENSOR_10, SENSOR_11};
  
  for(uint8_t i = 0; i < N_SENSORES; i++) {
      uint16_t value = analogRead(sensor_pins[i]);

      sensores[i] = line_type ? (value > umbrals[i]) : (value <= umbrals[i]);
  }
}

double calculateLinePosition() 
{
  int8_t acumulador = 0;
  uint8_t contador = 0;

  for(uint8_t i = 0; i < N_SENSORES; i++)
  {
    if(sensores[i])
    {
      acumulador += weights[i];
      contador++;
    }
  }
  
  return (double)acumulador / (double)contador;
}

double updatePID(double position) {
  double error = pid.setPoint - position;
  pid.accError += error;
  double derivative = error - pid.prevError;
  pid.prevError = error;
  // Cálculo de la corrección PID
  double correction = (pid.kp * error) + 
                      (pid.ki * pid.accError) + 
                      (pid.kd * derivative);
  return correction;
}

void ioHandleTask(void* params)
{
  while(true)
  {
    readSensors();
    position = calculateLinePosition();

    if(debug)
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
