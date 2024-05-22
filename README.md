# PN532 NFC library 

## Fork of Seed-Studio's PN532 library with some small additions

If you are looking for an ESP-IDF component, check out the esp-idf branch as that was reworked with the SPI Master Driver APIs

### Changes

- Added `setRFConfiguration` method
- Modified `readPassiveTargetID`'s signature to optionally return `atqa` and `sak`
- Added `timeout` arg to `inCommunicateThru`
- Implemented Extended Frame


