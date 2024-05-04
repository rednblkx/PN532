#ifndef __DEBUG_H__
#define __DEBUG_H__

// #define DEBUG
#include <esp_log.h>
#include <cstdarg>

// #ifdef DEBUG
#define DMSG(fmt, ...) if(!ignore_log){ESP_LOGD(TAG, fmt, ##__VA_ARGS__);}
#define DMSG_STR(tag, fmt, ...)  ESP_LOGD(tag, fmt, ##__VA_ARGS__)
#define DMSG_HEX(tag, buf, len) ESP_LOG_BUFFER_HEX_LEVEL(tag, buf, len, ESP_LOG_VERBOSE)
// #else
// #define DMSG(args...)
// #define DMSG_STR(tag, fmt, ...)
// #define DMSG_HEX(tag, buf, len)
// #define DMSG_INT(num)
// #endif


struct PN532_debug
{
  bool ignore_log = false;
  const char* TAG = "PN532";
  void setDebug(esp_log_level_t level) {
    esp_log_level_set(TAG, level);
  }
};

#endif