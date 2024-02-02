#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "vt3r0.h"

void i2c_read_buf(char *buf, uint32_t len);
void i2c_write_buf(const char *buf, uint32_t len);
void i2c_write_char(char c);
char i2c_read_char(void);

#endif
