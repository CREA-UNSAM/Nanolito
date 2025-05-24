#include <Commands.h>

String getAllValues()
{
  String msg = "A:" + String(pid.kp) + ":" + String(pid.ki) + ":" + String(pid.kd) + ":" 
          + String(vMax) + ":" + String(vBase) + ":" + String(vMin) + ":" + String(lineType);
  for(int i{0}; i < N_SENSORES; i++)
  {
    msg += ":" + String(weights[i]);
  }
  for(int i{0}; i < N_SENSORES; i++)
  {
    msg += ":" + String(umbrals[i]);
  }
  return msg;
}

void processMessage(const String& message)
  {
    char command = message.charAt(0);
    switch(command)
    {
      case 'a':
        pid.kp = getSplit(message, ':', 1).toFloat();
        pid.ki = getSplit(message, ':', 2).toFloat();
        pid.kd = getSplit(message, ':', 3).toFloat();
        break;
      case 'b':
        saveGlobals();
        break;
      default:
        break;
    }
  }

  String getMessage(char comando)
  {
    switch(comando)
    {
      case 'A':
        return getAllValues();
      default:
        return "";
    }
  }