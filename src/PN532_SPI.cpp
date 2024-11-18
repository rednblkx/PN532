
#include "PN532_SPI.h"
#include <string.h>
#include <vector>

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
        .max_transfer_sz = 0,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI
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
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST,
        .queue_size = 2
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
}

void PN532_SPI::wakeup()
{
    gpio_set_level(_ss, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(_ss, 1);
    // uint8_t long_preamble[16];
    // uint8_t cmd = DATA_WRITE;
    // std::fill(long_preamble, long_preamble + 16, 0x00);
    // write(&cmd);
    // for (size_t i = 0; i < 16; i++)
    // {
    //     write(&long_preamble[i]);
    // }
    // uint8_t hdr[] = { 0x00, 0x00 };
    // uint8_t length = 3;
    // std::vector<uint8_t> writeBuf;
    // writeBuf.push_back(PN532_PREAMBLE);
    // writeBuf.push_back(PN532_STARTCODE1);
    // writeBuf.push_back(PN532_STARTCODE2);
    // writeBuf.push_back(length);
    // writeBuf.push_back(~length + 1);
    // writeBuf.push_back(PN532_HOSTTOPN532);
    // uint8_t sum = PN532_HOSTTOPN532;
    // for (uint8_t i = 0; i < 2; i++)
    // {
    //     writeBuf.push_back(hdr[i]);
    //     sum += hdr[i];

    //     DMSG("%02x", hdr[i]);
    // }
    // uint8_t checksum = ~sum + 1;
    // writeBuf.push_back(checksum);
    // writeBuf.push_back(PN532_POSTAMBLE);
    // DMSG("WriteFrame");
    // DMSG_HEX(writeBuf.data(), writeBuf.size());
    // for (auto &v: writeBuf)
    // {
    //     write(&v);
    // }
    // uint8_t r[32];
    // gpio_set_level(_ss, 1);
    // uint8_t timeout = PN532_ACK_WAIT_TIME;
    // while (!isReady(ignore_log))
    // {
    //     vTaskDelay(2 / portTICK_PERIOD_MS);
    //     timeout--;
    //     if (0 == timeout)
    //     {
    //         DMSG("Time out when waiting for ACK");
    //         return;
    //     }
    // }
    // if (readAckFrame(ignore_log))
    // {
    //     DMSG("Invalid ACK");
    //     return;
    // }
    // readResponse(r, 32, 1000);
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
            return -2;
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
    gpio_set_level(_ss, 0);

    int16_t result;
    DMSG("read:");
    do
    {
        uint8_t* header = (uint8_t *)heap_caps_malloc(5, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
        read(header, 5, false, true);
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
            read(&msByte, 1);
            read(&lsByte, 1);
            read(&lcs, 1);
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
        { // checksum of length
            ESP_LOGE(TAG, "PN532::FAILED CHECKSUM LENGTH");
            result = PN532_INVALID_FRAME;
            heap_caps_free(header);
            break;
        }
        uint8_t tfi;
        uint8_t cmd;
        uint8_t* body = (uint8_t*)heap_caps_malloc(rxLen, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
        read(&tfi);
        read(&cmd);
        DMSG("TFI: %02x", tfi);
        DMSG("CMD: %02x", cmd);
        DMSG("PD LEN: %d", rxLen);
        for (size_t i = 0; i < rxLen; i++)
        {
            read(body + i);
            DMSG("DATA READ: %02x", body[i]);
        }
        if (rxLen == 1 && tfi == 0x7f && cmd == 0x81) {
            result = PN532_ERROR_FRAME;
            ESP_LOGE(TAG, "Received Error Frame - TFI: %02x CMD:%02x", tfi, cmd);
            uint8_t dummy[2];
            read(dummy, 2);
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }
        if (PN532_PN532TOHOST != tfi || (command + 1) != cmd)
        {
            result = PN532_INVALID_FRAME;
            ESP_LOGE(TAG, "PN532::COMMAND NOT VALID - TFI: %02x CMD:%02x", tfi, cmd);
            uint8_t dummy[2];
            read(dummy, 2);
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }


        if (rxLen > len)
        {
            ESP_LOGE(TAG, "Not enough space %d > %d", rxLen, len);
            uint8_t dummy[2];
            read(dummy, 2);
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
        read(&checksum);
        DMSG("DATA CHECKSUM: %02x", checksum);
        DMSG("SUM: %02x", sum);
        DMSG("CHECKSUM VERIFY: %02x", sum + checksum);
        if (0 != (uint8_t)(sum + checksum))
        {
            ESP_LOGE(TAG, "checksum is not ok Sum: %02x checksum: %02x", sum, checksum);
            result = PN532_INVALID_FRAME;
            uint8_t dummy;
            read(&dummy);
            heap_caps_free(header);
            heap_caps_free(body);
            break;
        }
        uint8_t POSTAMBLE;
        read(&POSTAMBLE); // POSTAMBLE
        DMSG("POSTAMBLE: %02X", POSTAMBLE);
        result = rxLen;
        heap_caps_free(header);
        heap_caps_free(body);
    } while (0);

    gpio_set_level(_ss, 1);
    return result;
}

bool PN532_SPI::isReady(bool ignore_log)
{
    gpio_set_level(_ss, 0);
    uint8_t status;
    uint8_t cmd = STATUS_READ;
    write(&cmd);
    read(&status);
    DMSG_HEX(&status, 1);
    gpio_set_level(_ss, 1);
    bool s = status == 1;
    return s;
}

void PN532_SPI::writeFrame(const uint8_t* header, uint8_t hlen, const uint8_t* body, uint8_t blen, bool ignore_log) {
    gpio_set_level(_ss, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS); // wake up PN532
    uint8_t length = hlen + blen + 1; // length of data field: TFI + DATA
    std::vector<uint8_t> writeBuf = {PN532_PREAMBLE, PN532_STARTCODE1, PN532_STARTCODE2, length, (uint8_t)(~length + 1), PN532_HOSTTOPN532};

    uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA

    DMSG("write: ");

    for (uint8_t i = 0; i < hlen; i++)
    {
        writeBuf.push_back(header[i]);
        sum += header[i];

        DMSG("%02x", header[i]);
    }
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
    write(writeBuf.data(), writeBuf.size(), true);

    gpio_set_level(_ss, 1);
}

int32_t PN532_SPI::readAckFrame(bool ignore_log)
{
    const uint8_t PN532_ACK[] = { 0, 0, 0xFF, 0, 0xFF, 0 };

    gpio_set_level(_ss, 0);

    std::vector<uint8_t> ackBuf(sizeof(PN532_ACK));

    read(ackBuf.data(), sizeof(PN532_ACK), false, true);

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        DMSG("%02x", ackBuf[i]);
    }

    gpio_set_level(_ss, 1);

    return memcmp(ackBuf.data(), PN532_ACK, sizeof(PN532_ACK));
}
