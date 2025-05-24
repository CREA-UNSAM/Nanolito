#ifndef GLOBALES

#define GLOBALES

#define WHITE true
#define BLACK false

#include <Arduino.h>
#include <Preferences.h>

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

extern bool lineType;
extern int weights[N_SENSORES];
extern short umbrals[N_SENSORES];

void loadGlobals();
void saveGlobals();

#endif