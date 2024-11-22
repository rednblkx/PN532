
#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_master.h>
#include <driver/spi_common.h>
#include <driver/gpio.h>
#include "PN532Interface.h"
#include "PN532_debug.h"
#include <esp_log.h>
#include <string.h>
#include <cmath>

using namespace std;

class PN532_SPI final : public PN532Interface, PN532_debug
{
public:
    PN532_SPI(uint8_t ss = CONFIG_PN532_SS, uint8_t sck = CONFIG_PN532_SCK, uint8_t miso = CONFIG_PN532_MISO, uint8_t mosi = CONFIG_PN532_MOSI, int bus_speed = 250 * 1000);
    ~PN532_SPI();
    void begin();
    void stop();
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

    esp_err_t write(uint8_t *data, size_t len = 1, bool cmd = false)
    {
        esp_err_t err;
        spi_transaction_ext_t transaction;
        memset(&transaction, 0, sizeof(transaction));
        if (cmd) {
            transaction.base.cmd = 0x01;
            transaction.command_bits = 8;
            transaction.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_MODE_OCT;
        }
        else {
            transaction.base.flags = SPI_TRANS_MODE_OCT;
        }
        transaction.base.length = len * 8;
        transaction.base.tx_buffer = data;
        err = spi_device_transmit(spi, (spi_transaction_t*)&transaction);
        return err;
    };

    esp_err_t read(uint8_t* out_data, size_t len = 1, bool rdy = false, bool cmd = false)
    {
        spi_transaction_ext_t transaction;
        memset(&transaction, 0, sizeof(transaction));
        if (cmd) {
            transaction.base.cmd = rdy ? (uint16_t)0x02 : (uint16_t)0x03;
            transaction.command_bits = 8;
            transaction.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_MODE_OCT;
        }
        else {
            transaction.base.flags = SPI_TRANS_MODE_OCT;
        }
        transaction.base.rxlength = len * 8;
        transaction.base.rx_buffer = out_data;
        esp_err_t err = spi_device_transmit(spi, (spi_transaction_t*)&transaction);
        return err;
    };
};

#endif
