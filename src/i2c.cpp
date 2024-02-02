#include "i2c.h"
#include "bcm2835.h"

void i2c_read_buf(char *buf, uint32_t len) {
    uint8_t retcode = bcm2835_i2c_read(buf, len);
    if (retcode != bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK) {
//        std::string reason_str = reasoncode_to_str((bcm2835I2CReasonCodes)retcode);
//        throw std::runtime_error("bcm2835_i2c_read failed: " +  reason_str);
    }
}

void i2c_write_buf(const char *buf, uint32_t len) {
    uint8_t retcode = bcm2835_i2c_write(buf, len);
    if (retcode != bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK) {
// TODO: hoitele jotenkin
//        std::string reason_str = reasoncode_to_str((bcm2835I2CReasonCodes)retcode);
//        throw std::runtime_error("bcm2835_i2c_write failed: " +  reason_str);
    }
}

void i2c_write_char(char c) {
    /* Helper method to write single bytes to i2c.
    */
    const char buffer[] = {c};
    i2c_write_buf(buffer, 1);
}

char i2c_read_char(void) {
    char c;
    i2c_read_buf(&c, 1);
    return c;
}

