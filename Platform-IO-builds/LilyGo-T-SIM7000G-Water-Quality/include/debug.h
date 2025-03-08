#ifndef DEBUG_H
#define DEBUG_H

#define DEBUG_ON  // Comment this out to disable debugging

#ifdef DEBUG_ON
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)  // Empty
  #define DEBUG_PRINTLN(...)  // Empty
#endif

#endif // DEBUG_H
