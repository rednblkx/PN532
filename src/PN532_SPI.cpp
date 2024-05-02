
#include "PN532_SPI.h"
#include "PN532_debug.h"
#include <string.h>
#include <vector>

#define STATUS_READ 0x02
#define DATA_WRITE 0x01
#define DATA_READ 0x03

using namespace std;

PN532_SPI::PN532_SPI()
{
    gpio_config_t ss_conf = {};
    ss_conf.pin_bit_mask = (1ULL << CONFIG_PN532_SS);
    ss_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    ss_conf.intr_type = GPIO_INTR_DISABLE;
    ss_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ss_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config_t sck_conf = {};
    sck_conf.pin_bit_mask = (1ULL << CONFIG_PN532_SCK);
    sck_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    sck_conf.intr_type = GPIO_INTR_DISABLE;
    sck_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    sck_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config_t miso_conf = {};
    miso_conf.pin_bit_mask = (1ULL << CONFIG_PN532_MISO);
    miso_conf.mode = GPIO_MODE_INPUT;
    miso_conf.intr_type = GPIO_INTR_DISABLE;
    miso_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    miso_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config_t mosi_conf = {};
    mosi_conf.pin_bit_mask = (1ULL << CONFIG_PN532_MOSI);
    mosi_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    mosi_conf.intr_type = GPIO_INTR_DISABLE;
    mosi_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    mosi_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&ss_conf);
    gpio_config(&sck_conf);
    gpio_config(&miso_conf);
    gpio_config(&mosi_conf);
    spi_bus_config_t buscfg = {
        .mosi_io_num = _mosi,
        .miso_io_num = _miso,
        .sclk_io_num = _clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}

void PN532_SPI::post_cb(spi_transaction_t *t) {
    PN532_SPI* nfc = (PN532_SPI*)t->user;
    if (t->cmd == STATUS_READ) {
        uint8_t* buf = (uint8_t*)t->rx_data;
        // ESP_LOG_BUFFER_HEX("POST_CB", buf, t->rxlength / 8);
        // ESP_LOGI("POST_CB", "READY? %d", buf[0] & 1);
        if (buf[0] & 1) {
            nfc->is_ready = true;
        }
    }
    if (t->cmd == DATA_READ && t->rxlength == 6 * 8) {
        const uint8_t PN532_ACK[] = { 0, 0, 0xFF, 0, 0xFF, 0 };
        ESP_LOG_BUFFER_HEX("POST_CB", t->rx_buffer, t->rxlength / 8);
        if (!memcmp(t->rx_buffer, PN532_ACK, sizeof(PN532_ACK))) {
            nfc->ack = true;
        }
    }
}

void PN532_SPI::begin()
{
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 0,                              //SPI mode 0
        .clock_speed_hz = 2 * 1000 * 1000,     //Clock out at 2 MHz
        .spics_io_num = -1,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST,
        .queue_size = 8,
        .post_cb = post_cb
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
}

void PN532_SPI::wakeup()
{
    gpio_set_level(_ss, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(_ss, 1);
}

int8_t PN532_SPI::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    command = header[0];
    writeFrame(header, hlen, body, blen);

    uint8_t timeout = PN532_ACK_WAIT_TIME;
    // ESP_LOGI("PN532", "WC");
    while (!is_ready)
    {
        isReady();
        vTaskDelay(1 / portTICK_PERIOD_MS);
        timeout--;
        if (0 == timeout)
        {
            DMSG("Time out when waiting for ACK\n");
            is_ready = false;
            return -2;
        }
    }
    is_ready = false;
    readAckFrame();
    if (!ack)
    {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }
    ack = false;
    return 0;
}

int16_t PN532_SPI::readResponse(uint8_t buf[], uint8_t len, uint16_t timeout)
{
    uint16_t time = 0;
    // ESP_LOGI("PN532", "RR");
    while (!is_ready)
    {
        isReady();
        vTaskDelay(1 / portTICK_PERIOD_MS);
        time++;
        if (time > timeout)
        {
            DMSG("readResponse: Time out when waiting for ACK\n");
            is_ready = false;
            return PN532_TIMEOUT;
        }
    }
    is_ready = false;
    gpio_set_level(_ss, 0);
    // vTaskDelay(1 / portTICK_PERIOD_MS);

    int16_t result;
    DMSG("read:");
    do
    {
        // cmd(DATA_READ);
        uint8_t* header = (uint8_t *)heap_caps_malloc(5 * sizeof(uint8_t), MALLOC_CAP_DMA);

        read(header, 5, false, true);
        DMSG("PREAMBLE: %02x", header[0]);
        DMSG("STARTCODE: 0x%02x , 0x%02X", header[1], header[2]);

        if (0x00 != header[0] ||   // PREAMBLE
            0x00 != header[1] || // STARTCODE1
            0xFF != header[2]    // STARTCODE2
        )
        {
            ESP_LOGE("PN532", "PN532::INVALID HEADER");
            result = PN532_INVALID_FRAME;
            heap_caps_free(header);
            break;
        }
        DMSG("DATA LENGTH: %02x", header[3]);
        DMSG("LENGTH CHECKSUM: %02x", header[4]);
        uint8_t msByte, lsByte, lcs;
        uint8_t len = header[3];
        if (header[3] == 0xFF && header[4] == 0xFF) {
            DMSG("SOMETHING STRANGE");
            read(&msByte, 1);
            read(&lsByte, 1);
            read(&lcs, 1);
            DMSG("che4cksum");
            DMSG("%02x", (uint8_t)(msByte + lsByte + lcs));
            if (0 != (uint8_t)(msByte + lsByte + lcs))
            {
                ESP_LOGE("PN532", "PN532::FAILED EXTENDED CHECKSUM LENGTH");
                result = PN532_INVALID_FRAME;
                heap_caps_free(header);
                break;
            }
            len = msByte * 256 + lsByte;
        }
        else if (0 != (uint8_t)(header[3] + header[4]))
        { // checksum of length
            ESP_LOGE("PN532", "PN532::FAILED CHECKSUM LENGTH");
            result = PN532_INVALID_FRAME;
            heap_caps_free(header);
            break;
        }
        uint8_t* body = (uint8_t*)heap_caps_malloc(header[3] * sizeof(uint8_t), MALLOC_CAP_DMA);
        read(body, header[3]);
        if (PN532_PN532TOHOST != body[0] || (command + 1) != body[1])
        {
            result = PN532_INVALID_FRAME;
            ESP_LOGE("PN532", "PN532::COMMAND NOT VALID");
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }

        DMSG("TFI: %02x", body[0]);
        DMSG("CMD: %02x", body[1]);

        // header[3] -= (header[4] == 0xFF ? 1 : 2);
        if (header[3] > len)
        {
            // uint8_t *dataBuf = (uint8_t *)heap_caps_malloc(header[3] * sizeof(uint8_t), MALLOC_CAP_DMA);
            // read(dataBuf, header[3]);
            ESP_LOGE("PN532", "Not enough space");
            result = PN532_NO_SPACE; // not enough space
            // heap_caps_free(dataBuf);
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }

        uint8_t sum = PN532_PN532TOHOST + body[1];
        memcpy(buf, body + 2, header[3] - 2);
        for (uint8_t i = 0; i < header[3] - 2; i++)
        {
            sum += buf[i];
            // DMSG("buf: %02x", buf[i]);
            DMSG("body: %02x", body[i + 2]);
        }
        uint8_t checksum;
        read(&checksum);
        DMSG("DATA CHECKSUM: %02x", checksum);
        DMSG("SUM: %02x", checksum);
        DMSG("CHECKSUM VERIFY: %02x", sum + checksum);
        if (0 != (uint8_t)(sum + checksum))
        {
            ESP_LOGE("PN532", "checksum is not ok\n");
            result = PN532_INVALID_FRAME;
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }
        uint8_t POSTAMBLE;
        read(&POSTAMBLE); // POSTAMBLE
        DMSG("POSTAMBLE: %02X", POSTAMBLE);

        result = len - 2;
        heap_caps_free(header);
        heap_caps_free(body);
    } while (0);

    gpio_set_level(_ss, 1);
    return result;
}

IRAM_ATTR bool PN532_SPI::isReady()
{
    gpio_set_level(_ss, 0);
    spi_transaction_ext_t t;
    t.base.cmd = STATUS_READ;
    t.command_bits = 8;
    t.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_MODE_OCT | SPI_TRANS_USE_RXDATA;
    t.base.user = this;
    t.base.tx_buffer = NULL;
    t.base.length = 0;
    t.base.rxlength = 8;
    esp_err_t err = spi_device_polling_transmit(spi, (spi_transaction_t*)&t);
    if (err != ESP_OK) {
        ESP_LOGE("PN532", "%s", esp_err_to_name(err));
        return false;
    }
    // uint8_t status;
    // read(&status, 1, true, true);
    // ESP_LOGI("PN532", "Is Ready? %02x", status);
    // DMSG_STR("PN532 - isReady", "%02x", status);
    gpio_set_level(_ss, 1);
    return true;
}

void PN532_SPI::writeFrame(const uint8_t* header, uint8_t hlen, const uint8_t* body, uint8_t blen) {
    gpio_set_level(_ss, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS); // wake up PN532
    uint8_t length = hlen + blen + 1; // length of data field: TFI + DATA
    uint8_t data[6 + length + 1] = { PN532_PREAMBLE, PN532_STARTCODE1, PN532_STARTCODE2, length, (uint8_t)(~length + 1), PN532_HOSTTOPN532 };

    uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA

    DMSG("write: ");

    for (uint8_t i = 0; i < hlen; i++)
    {
        data[6 + i] = header[i];
        sum += header[i];

        DMSG("%02x", header[i]);
    }
    for (uint8_t i = 0; i < blen; i++)
    {
        data[6 + hlen + i] = body[i];
        sum += body[i];

        DMSG("%02x", body[i]);
    }

    uint8_t checksum = ~sum + 1; // checksum of TFI + DATA
    data[6 + length - 1] = checksum;
    data[6 + length] = PN532_POSTAMBLE;
    DMSG_HEX("PN532 - writeFrame", data, sizeof(data));
    write(data, sizeof(data), true);

    gpio_set_level(_ss, 1);
}

IRAM_ATTR void PN532_SPI::readAckFrame()
{
    // ESP_LOGI("PN532", "ACK");
    const uint8_t PN532_ACK[] = { 0, 0, 0xFF, 0, 0xFF, 0 };

    // uint8_t ackBuf[sizeof(PN532_ACK)] = { };

    gpio_set_level(_ss, 0);

    spi_transaction_ext_t t;
    t.base.cmd = DATA_READ;
    t.command_bits = 8;
    t.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_MODE_OCT;
    t.base.user = this;
    t.base.tx_buffer = NULL;
    t.base.length = 0;
    t.base.rx_buffer = ackBuf;
    t.base.rxlength = sizeof(ackBuf) * 8;
    esp_err_t err = spi_device_polling_transmit(spi, (spi_transaction_t*)&t);
    if (err != ESP_OK) {
        ESP_LOGE("PN532", "%s", esp_err_to_name(err));
        // return true;
    }
    // vTaskDelay(1 / portTICK_PERIOD_MS);

    // read(ackBuf, 6, false, true);

    // for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    // {
    //     DMSG("%02x", ackBuf[i]);
    // }

    gpio_set_level(_ss, 1);

    // return memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK));
}
