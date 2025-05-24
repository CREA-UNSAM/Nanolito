#include <Utils.h>

String getSplit(const String& data, char separator, int index)
{
  int found = 0;
  int start = 0;
  int end = -1;

  for (int i = 0; i <= data.length(); i++) {
    if (data.charAt(i) == separator || i == data.length()) {
      if (found == index) 
      {
        return data.substring(start, i);
      }
      found++;
      start = i + 1;
    }
  }
  return "";
}