#ifndef GLOBALES

#define GLOBALES

#define WHITE true
#define BLACK false

#include <Arduino.h>
#include <Preferences.h>

enum Pins {
  // Motores
  MOTOR_L_PWM = 31, 
  MOTOR_L_IN1 = 36, 
  MOTOR_L_IN2 = 37, 
  MOTOR_R_PWM = 30,
  MOTOR_R_IN1 = 33,
  MOTOR_R_IN2 = 29,
  
  // Sensores
  SENSOR_01 = 11,
  SENSOR_02 = 10,
  SENSOR_03 = 9,
  SENSOR_04 = 8,
  SENSOR_05 = 7,
  SENSOR_06 = 16,
  SENSOR_07 = 14,
  SENSOR_08 = 13,
  SENSOR_09 = 12,
  SENSOR_10 = 11,
  SENSOR_11 = 24,
  
  LED_01 = 28, 
  LED_02 = 27, 
  BUTTON_01 = 25,
  BUTTON_02 = 23 
};

enum Status
{
  IDLE,
  RUN,
  CALIBRATION
};

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