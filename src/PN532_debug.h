#ifndef __DEBUG_H__
#define __DEBUG_H__

#define DEBUG
#include <esp_log.h>

// #ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
//     #define SERIAL SerialUSB
// #else
//     #define SERIAL Serial
// #endif

#ifdef DEBUG
#define DMSG(fmt, ...) ESP_LOGI("PN532", fmt, ##__VA_ARGS__)
#else
#define DMSG(args...)
#define DMSG_STR(str)
#define DMSG_HEX(num)
#define DMSG_INT(num)
#endif

#endif
