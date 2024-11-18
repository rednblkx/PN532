
#include "PN532_SPI.h"
#include <esp_log.h>
#include <string.h>
#include <cmath>
#include <string.h>
#include <vector>
#include <algorithm>

#define STATUS_READ 0x02
#define DATA_WRITE 0x01
#define DATA_READ 0x03

using namespace std;

PN532_SPI::PN532_SPI(uint8_t ss, uint8_t sck, uint8_t miso, uint8_t mosi, int bus_speed) : _ss(gpio_num_t(ss)), _clk(gpio_num_t(sck)), _miso(gpio_num_t(miso)), _mosi(gpio_num_t(mosi)), bus_speed(bus_speed)
{
    TAG = "PN532_SPI";
    gpio_config_t ss_conf = {};
    ss_conf.pin_bit_mask = (1ULL << ss);
    ss_conf.mode = GPIO_MODE_OUTPUT;
    ss_conf.intr_type = GPIO_INTR_DISABLE;
    ss_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ss_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config_t sck_conf = {};
    sck_conf.pin_bit_mask = (1ULL << sck);
    sck_conf.mode = GPIO_MODE_OUTPUT;
    sck_conf.intr_type = GPIO_INTR_DISABLE;
    sck_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    sck_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config_t miso_conf = {};
    miso_conf.pin_bit_mask = (1ULL << miso);
    miso_conf.mode = GPIO_MODE_INPUT;
    miso_conf.intr_type = GPIO_INTR_DISABLE;
    miso_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    miso_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config_t mosi_conf = {};
    mosi_conf.pin_bit_mask = (1ULL << mosi);
    mosi_conf.mode = GPIO_MODE_OUTPUT;
    mosi_conf.intr_type = GPIO_INTR_DISABLE;
    mosi_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    mosi_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&ss_conf);
    gpio_config(&sck_conf);
    gpio_config(&miso_conf);
    gpio_config(&mosi_conf);
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}

void PN532_SPI::begin()
{
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 0,
        .clock_speed_hz = this->bus_speed,
        .spics_io_num = -1,
        .flags = SPI_DEVICE_BIT_LSBFIRST,
        .queue_size = 1,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
}

void PN532_SPI::wakeup()
{
    gpio_set_level(_ss, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(_ss, 1);
}

void PN532_SPI::transfer(const uint8_t *txbuf, uint8_t *rxbuf, size_t length) {
        if (rxbuf != nullptr && txbuf != nullptr) {
            ESP_LOGE(TAG, "full-duplex not available!");
            return;
        }
        spi_transaction_t desc = {};
        desc.flags = 0;
        while (length != 0) {
        size_t const partial = std::min(length, static_cast<size_t>(4092));
        desc.length = partial * 8;
        desc.rxlength = txbuf == nullptr ? 0 : partial * 8;
        desc.tx_buffer = txbuf;
        desc.rx_buffer = rxbuf;
        esp_err_t err = spi_device_polling_start(this->spi, &desc, portMAX_DELAY);
        if (err == ESP_OK) {
            err = spi_device_polling_end(this->spi, portMAX_DELAY);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Transmit failed - err %X", err);
            break;
        }
        length -= partial;
        if (txbuf != nullptr)
            txbuf += partial;
        if (rxbuf != nullptr)
            rxbuf += partial;
        }
}

int8_t PN532_SPI::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen, bool ignore_log)
{
    command = header[0];
    writeFrame(header, hlen, body, blen, ignore_log);

    uint8_t timeout = PN532_ACK_WAIT_TIME;
    while (!isReady(ignore_log))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        timeout--;
        if (0 == timeout)
        {
            DMSG("Time out when waiting for ACK");
            return PN532_TIMEOUT;
        }
    }
    if (readAckFrame(ignore_log))
    {
        DMSG("Invalid ACK");
        return PN532_INVALID_ACK;
    }
    return 0;
}

int16_t PN532_SPI::readResponse(uint8_t buf[], uint16_t len, uint16_t timeout, bool ignore_log)
{
    uint16_t time = 0;
    while (!isReady(ignore_log))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        time++;
        if (time > timeout)
        {
            DMSG("readResponse: Time out when waiting for ACK\n");
            return PN532_TIMEOUT;
        }
    }
    if (spi_device_acquire_bus(this->spi, portMAX_DELAY) != ESP_OK) {
        return PN532_SPI_ERROR;
    }
    gpio_set_level(_ss, 0);

    int16_t result = -1;
    DMSG("read:");
    do
    {
        uint8_t* header = (uint8_t*)heap_caps_malloc(5, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
        write_byte(0x03);
        read_array(header, 5);
        DMSG("PREAMBLE: %02x", header[0]);
        DMSG("STARTCODE: 0x%02x , 0x%02X", header[1], header[2]);

        if (0x00 != header[0] ||   // PREAMBLE
            0x00 != header[1] || // STARTCODE1
            0xFF != header[2]    // STARTCODE2
        )
        {
            ESP_LOGE(TAG, "PN532::INVALID HEADER");
            result = PN532_INVALID_FRAME;
            heap_caps_free(header);
            break;
        }
        uint8_t msByte, lsByte, lcs;
        uint16_t rxLen = header[3] >= 2 ? header[3] - 2 : header[3];
        DMSG("DATA LENGTH: %02x", header[3]);
        DMSG("LENGTH CHECKSUM: %02x", header[4]);
        if (header[3] == 0xFF && header[4] == 0xFF) {
            msByte = read_byte();
            lsByte = read_byte();
            lcs = read_byte();
            DMSG("EXTENDED FRAME MsByte: %d LsByte: %d LENGTH: %d", msByte, lsByte, ((((uint16_t)msByte) << 8) | lsByte));
            DMSG("EXTENDED DATA CHECKSUM %02x", (uint8_t)(msByte + lsByte + lcs));
            if (0 != (uint8_t)(msByte + lsByte + lcs))
            {
                ESP_LOGE(TAG, "PN532::FAILED EXTENDED CHECKSUM LENGTH");
                result = PN532_INVALID_FRAME;
                heap_caps_free(header);
                break;
            }
            rxLen = ((((uint16_t)msByte) << 8) | lsByte) - 2;
        }
        else if (0 != (uint8_t)(header[3] + header[4]))
        {
            ESP_LOGE(TAG, "PN532::FAILED CHECKSUM LENGTH");
            result = PN532_INVALID_FRAME;
            heap_caps_free(header);
            break;
        }
        uint8_t tfi;
        uint8_t cmd;
        uint8_t* body = (uint8_t*)heap_caps_malloc(rxLen, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
        tfi = read_byte();
        cmd = read_byte();
        DMSG("TFI: %02x", tfi);
        DMSG("CMD: %02x", cmd);
        DMSG("PD LEN: %d", rxLen);
        read_array(body, rxLen);
        if (rxLen == 1 && tfi == 0x7f && cmd == 0x81) {
            result = PN532_ERROR_FRAME;
            ESP_LOGE(TAG, "Received Error Frame - TFI: %02x CMD:%02x", tfi, cmd);
            uint8_t dummy[2];
            read_array(dummy, 2);
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }
        if (PN532_PN532TOHOST != tfi || (command + 1) != cmd)
        {
            result = PN532_INVALID_FRAME;
            ESP_LOGE(TAG, "PN532::COMMAND NOT VALID - TFI: %02x CMD:%02x", tfi, cmd);
            uint8_t dummy[2];
            read_array(dummy, 2);
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }


        if (rxLen > len)
        {
            ESP_LOGE(TAG, "Not enough space %d > %d", rxLen, len);
            uint8_t dummy[2];
            read_array(dummy, 2);
            result = PN532_NO_SPACE; // not enough space
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }

        uint8_t sum = PN532_PN532TOHOST + cmd;
        for (uint16_t i = 0; i < rxLen; i++)
        {
            sum += body[i];
            buf[i] = body[i];
            DMSG("body: %02x", body[i]);
        }
        DMSG("DATA COPIED TO BUFFER");
        uint8_t checksum;
        checksum = read_byte();
        DMSG("DATA CHECKSUM: %02x", checksum);
        DMSG("SUM: %02x", sum);
        DMSG("CHECKSUM VERIFY: %02x", sum + checksum);
        if (0 != (uint8_t)(sum + checksum))
        {
            ESP_LOGE(TAG, "checksum is not ok Sum: %02x checksum: %02x", sum, checksum);
            result = PN532_INVALID_FRAME;
            uint8_t dummy;
            dummy = read_byte();
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }
        uint8_t POSTAMBLE;
        POSTAMBLE = read_byte();
        DMSG("POSTAMBLE: %02X", POSTAMBLE);
        result = rxLen;
        heap_caps_free(header);
        heap_caps_free(body);
    } while (0);

    gpio_set_level(_ss, 1);
    spi_device_release_bus(this->spi);
    return result;
}

bool PN532_SPI::isReady(bool ignore_log)
{
    if (spi_device_acquire_bus(this->spi, portMAX_DELAY) != ESP_OK) {
        return PN532_SPI_ERROR;
    }
    gpio_set_level(_ss, 0);
    write_byte(0x02);
    uint8_t ready = this->read_byte();
    DMSG("%02x", ready);
    gpio_set_level(_ss, 1);
    spi_device_release_bus(this->spi);
    return ready == 0x01;
}

void PN532_SPI::writeFrame(const uint8_t* header, uint8_t hlen, const uint8_t* body, uint8_t blen, bool ignore_log) {
    if (spi_device_acquire_bus(this->spi, portMAX_DELAY) != ESP_OK) {
        return;
    }
    gpio_set_level(_ss, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS); // wake up PN532
    uint8_t length = hlen + blen + 1; // length of data field: TFI + DATA
    std::vector<uint8_t> writeBuf = {PN532_PREAMBLE, PN532_STARTCODE1, PN532_STARTCODE2, length, (uint8_t)(~length + 1), PN532_HOSTTOPN532};

    uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA

    DMSG("header: ");

    for (uint8_t i = 0; i < hlen; i++)
    {
        writeBuf.push_back(header[i]);
        sum += header[i];

        DMSG("%02x", header[i]);
    }
    DMSG("body: ");
    for (uint8_t i = 0; i < blen; i++)
    {
        writeBuf.push_back(body[i]);
        sum += body[i];

        DMSG("%02x", body[i]);
    }

    uint8_t checksum = ~sum + 1; // checksum of TFI + DATA
    writeBuf.push_back(checksum);
    writeBuf.push_back(PN532_POSTAMBLE);
    DMSG("WriteFrame");
    DMSG_HEX(writeBuf.data(), writeBuf.size());
    write_byte(0x01);
    write_array(writeBuf.data(), writeBuf.size());

    gpio_set_level(_ss, 1);
    spi_device_release_bus(this->spi);
}

int32_t PN532_SPI::readAckFrame(bool ignore_log)
{
    if (spi_device_acquire_bus(this->spi, portMAX_DELAY) != ESP_OK) {
        return PN532_SPI_ERROR;
    }
    const uint8_t PN532_ACK[] = { 0, 0, 0xFF, 0, 0xFF, 0 };

    gpio_set_level(_ss, 0);

    std::vector<uint8_t> ackBuf(sizeof(PN532_ACK));

    write_byte(0x03);
    read_array(ackBuf.data(), sizeof(PN532_ACK));

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        DMSG("%02x", ackBuf[i]);
    }

    gpio_set_level(_ss, 1);
    spi_device_release_bus(this->spi);
    return memcmp(ackBuf.data(), PN532_ACK, sizeof(PN532_ACK));
}
