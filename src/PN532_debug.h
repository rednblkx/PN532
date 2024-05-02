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
#define DMSG_STR(tag, fmt, ...)  ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define DMSG_HEX(tag, buf, len) ESP_LOG_BUFFER_HEX(tag, buf, len)
#else
#define DMSG(args...)
#define DMSG_STR(tag, fmt, ...)
#define DMSG_HEX(tag, buf, len)
#define DMSG_INT(num)
#endif

#endif
