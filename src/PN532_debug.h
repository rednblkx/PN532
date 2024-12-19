#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <esp_log.h>
#include <cstdarg>
#define DMSG(fmt, ...) if(!ignore_log){ESP_LOGD(TAG, "%s:%d > " fmt, __FUNCTION__ , __LINE__ __VA_OPT__(, ) __VA_ARGS__);}
#define DMSG_STR(tag, fmt, ...)  ESP_LOGD(TAG, "%s:%d > " fmt, __FUNCTION__ , __LINE__ __VA_OPT__(, ) __VA_ARGS__)
#define DMSG_HEX(buf, len) if(!ignore_log){ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, len, ESP_LOG_VERBOSE);}


struct PN532_debug
{
  bool ignore_log = false;
  const char* TAG = "PN532";
  void setDebug(esp_log_level_t level) {
    esp_log_level_set(TAG, level);
  }
};

#endif