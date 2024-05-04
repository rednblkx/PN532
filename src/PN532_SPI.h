
#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__


#include <freertos/FreeRTOS.h>
#include <driver/spi_master.h>
#include <driver/spi_common.h>
#include <driver/gpio.h>
#include "PN532Interface.h"
#include "PN532_debug.h"
#include <esp_log.h>
#include <string.h>
#include <cmath>

using namespace std;

class PN532_SPI : public PN532Interface, PN532_debug
{
public:
    ~PN532_SPI();
    PN532_SPI();

    void begin();
    void wakeup();
    int8_t writeCommand(const uint8_t* header, uint8_t hlen, const uint8_t* body = 0, uint8_t blen = 0, bool ignore_log = false);

    int16_t readResponse(uint8_t buf[], uint16_t len, uint16_t timeout, bool ignore_log = false);

private:
    uint8_t* writeBuf;
    gpio_num_t _ss = gpio_num_t(CONFIG_PN532_SS);
    gpio_num_t _clk = gpio_num_t(CONFIG_PN532_SCK);
    gpio_num_t _miso = gpio_num_t(CONFIG_PN532_MISO);
    gpio_num_t _mosi = gpio_num_t(CONFIG_PN532_MOSI);
    spi_device_handle_t spi;
    uint8_t command;
    static void post_cb(spi_transaction_t *t);

    bool isReady();
    void writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0, bool ignore_log = false);
    int32_t readAckFrame(bool ignore_log);

    IRAM_ATTR esp_err_t write(uint8_t *data, size_t len = 1, bool cmd = false)
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
        err = spi_device_polling_transmit(spi, (spi_transaction_t*)&transaction);
        return err;
    };

    IRAM_ATTR uint8_t read(uint8_t* out_data, size_t len = 1, bool rdy = false, bool cmd = false)
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
        esp_err_t err = spi_device_polling_transmit(spi, (spi_transaction_t*)&transaction);
        if (err != ESP_OK) {
            return err;
        }
        return ESP_OK;
    };
};

#endif
