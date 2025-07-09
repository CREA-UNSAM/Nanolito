#include <Globals.h>

Status status = IDLE;
bool debug = 

PIDController pid;

int vMax{};
int vBase{};
int vMin{};

int sensores[] = {};
double position{};

bool lineType = WHITE;
int weights[N_SENSORES] = {};
short umbrals[N_SENSORES] = {PWM_IN_MAX / 2};

void loadGlobals()
{
  Preferences pref;
  pref.begin("Globals", true);

  pid.kp = pref.getFloat(KP_KEY.c_str(), 1.0f);
  pid.ki = pref.getFloat(KI_KEY.c_str(), 1.0f);
  pid.kd = pref.getFloat(KD_KEY.c_str(), 1.0f);

  vMin = pref.getInt(VMIN_KEY.c_str(), -255);
  vBase = pref.getInt(VBASE_KEY.c_str(), 255);
  vMax = pref.getInt(VMAX_KEY.c_str(), 255);

  lineType = pref.getBool(LINETYPE_KEY.c_str(), WHITE);

  for(int i{0}; i < N_SENSORES; i++)
  {
    weights[i] = pref.getInt((WEIGHT_KEY + String(i)).c_str(), 0);
    umbrals[i] = pref.getShort((UMBRAL_KEY + String(i)).c_str(), PWM_IN_MAX / 2);
  }
}

void saveGlobals()
{
  Preferences pref;
  pref.begin("Globals", false);

  pref.putFloat(KP_KEY.c_str(), pid.kp);
  pref.putFloat(KI_KEY.c_str(), pid.ki);
  pref.putFloat(KD_KEY.c_str(), pid.kd);

  pref.putInt(VMIN_KEY.c_str(), vMin);
  pref.putInt(VBASE_KEY.c_str(), vBase);
  pref.putInt(VMAX_KEY.c_str(), vMax);

  pref.getBool(LINETYPE_KEY.c_str(), lineType);

  for(int i{0}; i < N_SENSORES; i++)
  {
    pref.getInt((WEIGHT_KEY + String(i)).c_str(), weights[i]);
    pref.getShort((UMBRAL_KEY + String(i)).c_str(), umbrals[i]);
  }
}