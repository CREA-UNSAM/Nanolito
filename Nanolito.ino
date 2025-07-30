#include <Globals.h>
#include <Bluetooth.h>
extern "C" {
  #include "esp_wifi.h"
}

void readSensors() 
{
  for(uint8_t i = 0; i < N_SENSORES; i++) {
      uint16_t value = analogRead(sensorPins[i]);

      sensores[i] = lineType ? (value > umbrals[i]) : (value <= umbrals[i]);
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

  for(int i{0}; i < N_SENSORES; i++)
    pinMode(sensorPins[i], INPUT);

  pinMode(BUTTON_01, INPUT);
  pinMode(BUTTON_02, INPUT);

  pinMode(LED_02, OUTPUT);
  pinMode(LED_01, OUTPUT);
  
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);

  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  ledcAttach(MOTOR_L_PWM, 20000, 8);
  ledcAttach(MOTOR_R_PWM, 20000, 8);

  xTaskCreatePinnedToCore(ioHandleTask, "IO", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlTask, "PIDControl", 4096, NULL, 1, NULL, 1);
}

void loop(){}
