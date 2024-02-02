#ifndef EXTI2C_H
#define EXTI2C_H

#include "module.h"

class ExtI2cModule: public Module {
public:
    ExtI2cModule( uint8_t **buf, uint16_t len );
    virtual ~ExtI2cModule();
    virtual uint8_t begin();
    virtual void process( char **resBufPtr );
    virtual void print();
protected:
    uint8_t i2cAddress;
    uint8_t readBuffer[EXP_NUM_DATA_CHANNELS * sizeof(uint32_t)];
};
//-------

class ExtI2cChannel: public AnalogChannel {
public:
    ExtI2cChannel( uint8_t **buf, uint16_t len );
    virtual ~ExtI2cChannel();
    uint8_t begin(uint8_t *sampleData);
    virtual bool sample();
    virtual void process( char **resBufPtr, bool forceReporting );
    virtual void print();
private:
    uint8_t *readBuffer;
};

#endif // EXTI2C_H

