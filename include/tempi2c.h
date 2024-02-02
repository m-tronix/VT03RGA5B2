#ifndef TEMPI2C_H
#define TEMPI2C_H

#include "module.h"

class TempI2cModule: public Module {
public:
    TempI2cModule( uint8_t **buf, uint16_t len );
    virtual ~TempI2cModule();
    virtual uint8_t begin();
    virtual void process( char **resBufPtr );
    virtual void print();
protected:
    uint8_t i2cAddress;
    uint8_t readBuffer[2];
};


class TempI2cChannel: public AnalogChannel {
public:
    TempI2cChannel( uint8_t **buf, uint16_t len );
    virtual ~TempI2cChannel();
    uint8_t begin(uint8_t *sampleData);
    virtual bool sample();
    virtual void process( char **resBufPtr, bool forceReporting );
    virtual void print();
private:
    uint8_t *readBuffer;
};

#endif // TEMPI2C_H

