/* 

Need to put typdef struct definitions in a .h file.  It's some kind of work-around
Source: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264977701/8#8
Putting definition in .h file is an Arduino work-around.  Ref: http://www.arduino.cc/playground/Code/Struct#FAQ

*/

#ifndef Types_h
#define Types_h

#include <Arduino.h>

typedef struct {
  bool online;         // True if panStamp is online
  byte ID;             // panStamp Transmitter ID
  bool IsWet;          // Wet = true, Dry = false
  int temp;            // temp from TMP36
  bool lowTempMsgFlag; // one shot trigger for low temp tweet
  int volts;           // millivolts
  bool lowVoltMsgFlag; // one shot trigger for low volts tweet 
} RemoteSensorData_t;

#endif 
