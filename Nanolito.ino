#include <Globals.h>
#include <Bluetooth.h>
extern "C" {
  #include "esp_wifi.h"
}

hw_timer_t* runTimer = NULL;
hw_timer_t* calibrateTimer = NULL;
hw_timer_t* idleTimer = NULL;

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

void IRAM_ATTR onCalibrate()
{
  if(status != CALIBRATION)
  {
    status = CALIBRATION;
  }
  else
  {
    status = IDLE;
  }
}

void IRAM_ATTR onRun()
{
  if(status != RUN)
  {
    status = RUN;
  }
  else
  {
    status = IDLE;
  }
}

void IRAM_ATTR handleButton01()
{
  if(!digitalRead(BUTTON_01))
  {
    timerAlarm(calibrateTimer, 1, false, 0);
    timerWrite(calibrateTimer, 0);
    timerStart(calibrateTimer);
  }
  else
  {
    timerStop(calibrateTimer);
  }
}

void IRAM_ATTR handleButton02()
{
  if(!digitalRead(BUTTON_02))
  {
    timerAlarm(runTimer, 1, false, 0);
    timerWrite(runTimer, 0);
    timerStart(runTimer);
  }
  else
  {
    timerStop(runTimer);
  }
}

void setMotorSpeed(Motor &motor, int16_t speed)
{
    speed = constrain(speed, vMin, vMax);

    if(speed > 0)
    {
      digitalWrite(motor.in1_pin, HIGH);
      digitalWrite(motor.in2_pin, LOW);
    }
    else if(speed < 0)
    {
      digitalWrite(motor.in1_pin, LOW);
      digitalWrite(motor.in2_pin, HIGH);
      speed = -speed;
    }
    else
    {
      digitalWrite(motor.in1_pin, LOW);
      digitalWrite(motor.in2_pin, LOW);
    }

    ledcWrite(motor.pwm_pin, speed);
}

void calibrationMovement()
{
  const int MAX_MOVEMENTS = 5;
  const int MOVEMENT_TIME = 200;
  const int CALIBRATION_SPEED = 150;
  static int movements = 0;
  static unsigned long timer = millis();
  static bool dir = false;
  unsigned long currentTime = millis();

  if(currentTime - timer >= MOVEMENT_TIME || timer == currentTime)
  {
    setMotorSpeed(leftMotor,  dir ? 80 : -80);
    setMotorSpeed(rightMotor,  dir ? 80 : -80);

    dir = !dir;
    movements++;
    timer = currentTime;
  }

  if(movements >= MAX_MOVEMENTS * 2)
  {
    status = IDLE;
  }

}

void ioHandleTask(void* params)
{
  while(true)
  {
    readSensors();
    position = calculateLinePosition();

    if(debug)
      Bluetooth::readBT();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void controlTask(void* params)
{
  while(true)
  {
    double output{};
    bool inLine{false};
    switch(status)
    {
      case IDLE:
        digitalWrite(LED_01, HIGH);
        digitalWrite(LED_02, LOW);
        setMotorSpeed(leftMotor,  0);
        setMotorSpeed(rightMotor, 0);
        break;
      case RUN:
        digitalWrite(LED_01,  HIGH);
        digitalWrite(LED_02, HIGH);
        for(int i{0}; i < N_SENSORES; i++)
        {
          if(sensores[i])
          {
            inLine = true;
            break;
          }
        }
        
        if(inLine)
        {
          output = updatePID(position);
          setMotorSpeed(leftMotor,  80 + output);
          setMotorSpeed(rightMotor, 80 - output);
        }
        else
        {
          setMotorSpeed(leftMotor, 0);
          setMotorSpeed(rightMotor, 0);
        }
        break;
      case CALIBRATION:
        digitalWrite(LED_01, LOW);
        digitalWrite(LED_02, LOW);
        //calibrationMovement();
        setMotorSpeed(leftMotor,  0);
        setMotorSpeed(rightMotor, 0);
        break;
      default:
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
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

  runTimer = timerBegin(1000000);
  calibrateTimer = timerBegin(1000000);
  
  timerAttachInterrupt(runTimer, onRun);

  timerAttachInterrupt(calibrateTimer, onCalibrate);

  attachInterrupt(BUTTON_01, handleButton01, CHANGE);
  attachInterrupt(BUTTON_02, handleButton02, CHANGE);

  //Serial.println("Holi :D");

  xTaskCreatePinnedToCore(ioHandleTask, "IO", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlTask, "PIDControl", 4096, NULL, 1, NULL, 1);
}

void loop(){}
