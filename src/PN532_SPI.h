
#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__

#include "PN532Interface.h"
#include "PN532_debug.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>

using namespace std;

class PN532_SPI : public PN532Interface, PN532_debug
{
public:
    PN532_SPI(uint8_t ss = CONFIG_PN532_SS, uint8_t sck = CONFIG_PN532_SCK, uint8_t miso = CONFIG_PN532_MISO, uint8_t mosi = CONFIG_PN532_MOSI, int bus_speed = 250 * 1000);

    void begin();
    void wakeup();
    int8_t writeCommand(const uint8_t* header, uint8_t hlen, const uint8_t* body = 0, uint8_t blen = 0, bool ignore_log = false);

    int16_t readResponse(uint8_t buf[], uint16_t len, uint16_t timeout, bool ignore_log = false);

private:
    const gpio_num_t _ss;
    const gpio_num_t _clk;
    const gpio_num_t _miso;
    const gpio_num_t _mosi;
    int bus_speed;
    spi_device_handle_t spi;
    uint16_t command;

    bool isReady(bool);
    void writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0, bool ignore_log = false);
    int32_t readAckFrame(bool ignore_log);

    void transfer(const uint8_t* txbuf, uint8_t* rxbuf, size_t length);
    void write_array(const uint8_t* ptr, size_t length) { this->transfer(ptr, nullptr, length); }
    void read_array(uint8_t *ptr, size_t length) { this->transfer(nullptr, ptr, length); }
    void write_byte(uint8_t data) { this->write_array(&data, 1); }
    uint8_t read_byte() { uint8_t rxbuf;this->transfer(nullptr, &rxbuf, 1);return rxbuf; }
};

#endif
