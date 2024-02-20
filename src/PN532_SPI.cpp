
#include "PN532_SPI.h"
#include "PN532_debug.h"
#include "Arduino.h"

#define STATUS_READ 2
#define DATA_WRITE 1
#define DATA_READ 3

PN532_SPI::PN532_SPI(SPIClass &spi, uint8_t ss)
{
    command = 0;
    _spi = &spi;
    _ss = ss;
}

void PN532_SPI::begin()
{
    pinMode(_ss, OUTPUT);

    _spi->begin();
    _spi->setDataMode(SPI_MODE0); // PN532 only supports mode0
    _spi->setBitOrder(LSBFIRST);
#if defined __SAM3X8E__
    /** DUE spi library does not support SPI_CLOCK_DIV8 macro */
    _spi->setClockDivider(42); // set clock 2MHz(max: 5MHz)
#elif defined __SAMD21G18A__
    /** M0 spi library does not support SPI_CLOCK_DIV8 macro */
    _spi->setClockDivider(24); // set clock 2MHz(max: 5MHz)
#else
    _spi->setClockDivider(SPI_CLOCK_DIV8); // set clock 2MHz(max: 5MHz)
#endif
}

void PN532_SPI::wakeup()
{
    digitalWrite(_ss, LOW);
    delay(2);
    digitalWrite(_ss, HIGH);
}

int8_t PN532_SPI::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    command = header[0];
    writeFrame(header, hlen, body, blen);

    uint8_t timeout = PN532_ACK_WAIT_TIME;
    while (!isReady())
    {
        delay(1);
        timeout--;
        if (0 == timeout)
        {
            DMSG("Time out when waiting for ACK\n");
            return -2;
        }
    }
    if (readAckFrame())
    {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }
    return 0;
}

int16_t PN532_SPI::readResponse(uint8_t buf[], uint8_t len, uint16_t timeout)
{
    uint16_t time = 0;
    while (!isReady())
    {
        delay(1);
        time++;
        if (time > timeout)
        {
            return PN532_TIMEOUT;
        }
    }

    digitalWrite(_ss, LOW);
    delay(1);

    int16_t result;
    do
    {
        write(DATA_READ);

        uint8_t PREAMBLE = read();
        uint8_t STARTCODE1 = read();
        uint8_t STARTCODE2 = read();

        DMSG_HEX(PREAMBLE);
        DMSG_HEX(STARTCODE1);
        DMSG_HEX(STARTCODE2);
        DMSG_STR("");

        if (0x00 != PREAMBLE ||   // PREAMBLE
            0x00 != STARTCODE1 || // STARTCODE1
            0xFF != STARTCODE2    // STARTCODE2
        )
        {
            DMSG_STR("PN532::INVALID HEADER");
            result = PN532_INVALID_FRAME;
            break;
        }

        uint8_t length = read();
        uint8_t lcs = read();
        uint8_t byte;
        uint8_t dataLength;
        DMSG_HEX(length);
        DMSG_STR("");
        DMSG_HEX(lcs);
        if(length == 0xFF && lcs == 0xFF){
            DMSG_STR("SOMETHING STRANGE");
            read();
            byte = read();
            dataLength = read();
            DMSG_HEX(byte);
            DMSG_HEX(dataLength);
            DMSG("che4cksum");
            DMSG_HEX((uint8_t)(dataLength + byte));
            if (0xFF != (uint8_t)(dataLength + byte))
            {
                DMSG("PN532::FAILED CHECKSUM LENGTH");
                result = PN532_INVALID_FRAME;
                break;
            }
        } else if (0 != (uint8_t)(length + lcs))
        { // checksum of length
            DMSG_STR("PN532::FAILED CHECKSUM LENGTH");
            result = PN532_INVALID_FRAME;
            break;
        }

        uint8_t cmd = command + 1; // response command
        uint8_t TO_HOST = read();
        uint8_t CMD_TO_HOST = read();
        DMSG_HEX(TO_HOST);
        DMSG_HEX(CMD_TO_HOST);
        // printf("\n");
        if (PN532_PN532TOHOST != TO_HOST || (cmd) != CMD_TO_HOST)
        {
            result = PN532_INVALID_FRAME;
            DMSG_STR("PN532::COMMAND NOT VALID");
            break;
        }

        DMSG("read:  ");
        DMSG_HEX(cmd);
        // printf("\n");

        length -= (lcs == 0xFF ? 1 : 2);
        if (length > len)
        {
            for (uint8_t i = 0; i < length; i++)
            {
                DMSG_HEX(read()); // dump message
            }
            DMSG("\nNot enough space\n");
            read();
            read();
            result = PN532_NO_SPACE; // not enough space
            break;
        }

        uint8_t sum = PN532_PN532TOHOST + cmd;
        for (uint8_t i = 0; i < length; i++)
        {
            buf[i] = read();
            sum += buf[i];

            DMSG_HEX(buf[i]);
        }
        DMSG('\n');
        uint8_t checksum = read();
        DMSG_HEX(checksum);
        if (0 != (uint8_t)(sum + checksum))
        {
            DMSG("checksum is not ok\n");
            result = PN532_INVALID_FRAME;
            break;
        }
        uint8_t POSTAMBLE = read(); // POSTAMBLE

        result = length;
    } while (0);

    digitalWrite(_ss, HIGH);

    return result;
}

bool PN532_SPI::isReady()
{
    digitalWrite(_ss, LOW);

    write(STATUS_READ);
    uint8_t status = read() & 1;
    digitalWrite(_ss, HIGH);
    return status;
}

void PN532_SPI::writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    digitalWrite(_ss, LOW);
    delay(2); // wake up PN532

    write(DATA_WRITE);
    write(PN532_PREAMBLE);
    write(PN532_STARTCODE1);
    write(PN532_STARTCODE2);

    uint8_t length = hlen + blen + 1; // length of data field: TFI + DATA
    write(length);
    write(~length + 1); // checksum of length

    write(PN532_HOSTTOPN532);
    uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA

    DMSG("write: ");

    for (uint8_t i = 0; i < hlen; i++)
    {
        write(header[i]);
        sum += header[i];

        DMSG_HEX(header[i]);
    }
    for (uint8_t i = 0; i < blen; i++)
    {
        write(body[i]);
        sum += body[i];

        DMSG_HEX(body[i]);
    }

    uint8_t checksum = ~sum + 1; // checksum of TFI + DATA
    write(checksum);
    write(PN532_POSTAMBLE);

    digitalWrite(_ss, HIGH);

    DMSG('\n');
}

int8_t PN532_SPI::readAckFrame()
{
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};

    uint8_t ackBuf[sizeof(PN532_ACK)];

    digitalWrite(_ss, LOW);
    delay(1);
    write(DATA_READ);

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        ackBuf[i] = read();
    }

    digitalWrite(_ss, HIGH);

    return memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK));
}
