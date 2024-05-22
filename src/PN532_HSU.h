
#ifndef __PN532_HSU_H__
#define __PN532_HSU_H__

#include "PN532Interface.h"
#include "Arduino.h"

#define PN532_HSU_DEBUG 1

#define PN532_HSU_READ_TIMEOUT (1000)

class PN532_HSU : public PN532Interface
{
public:
    PN532_HSU(HardwareSerial &serial, uint8_t tx = 0, uint8_t rx = 0);

    void begin();
    void wakeup();
    virtual int8_t writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);
    int16_t readResponse(uint8_t buf[], uint8_t len, uint16_t timeout);

private:
    HardwareSerial *_serial;
    uint8_t _tx_pin;
    uint8_t _rx_pin;
    uint8_t command;

    int8_t readAckFrame();

    int8_t receive(uint8_t *buf, int len, uint16_t timeout = PN532_HSU_READ_TIMEOUT);
};

#endif
