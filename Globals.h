#ifndef GLOBALES

#define GLOBALES

#define WHITE true
#define BLACK false

#include <Arduino.h>
#include <Preferences.h>

enum Pins {
  // Motores
  MOTOR_L_PWM = 19, 
  MOTOR_L_IN1 = 22, 
  MOTOR_L_IN2 = 23, 
  MOTOR_R_PWM = 18,
  MOTOR_R_IN1 = 21,
  MOTOR_R_IN2 = 5,
  
  // Sensores
  SENSOR_01 = 25,
  SENSOR_02 = 33,
  SENSOR_03 = 32,
  SENSOR_04 = 35,
  SENSOR_05 = 34,
  SENSOR_06 = 13,
  SENSOR_07 = 12,
  SENSOR_08 = 14,
  SENSOR_09 = 27,
  SENSOR_10 = 26,
  SENSOR_11 = 2,
  
  LED_01 = 17, 
  LED_02 = 16, 

  BUTTON_01 = 4,
  BUTTON_02 = 0
};

enum Status
{
  IDLE,
  RUN,
  CALIBRATION
};

const uint8_t sensorPins[] = {SENSOR_01, SENSOR_02, SENSOR_03, 
                              SENSOR_04, SENSOR_05, SENSOR_06,
                              SENSOR_07, SENSOR_08, SENSOR_09,
                              SENSOR_10, SENSOR_11};

extern Status status;
extern bool debug;

const uint8_t N_SENSORES = 11; 
const short PWM_IN_MAX = 4095;

const String KP_KEY = "KP";
const String KI_KEY = "KI";
const String KD_KEY = "KD";
const String VMIN_KEY = "VMIN";
const String VBASE_KEY = "VBASE";
const String VMAX_KEY = "VMAX";
const String LINETYPE_KEY = "LINETYPE";
const String WEIGHT_KEY = "WEIGHT";
const String UMBRAL_KEY = "UMBRAL";

struct PIDController {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float setPoint = 0.0f;
  float accError = 0.0f;
  float prevError = 0.0f;
};

extern PIDController pid;

extern int vMax;
extern int vBase;
extern int vMin;

extern int sensores[N_SENSORES];
extern double position;

extern bool lineType;
extern int weights[N_SENSORES];
extern short umbrals[N_SENSORES];

void loadGlobals();
void saveGlobals();

#endif