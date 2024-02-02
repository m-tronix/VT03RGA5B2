#ifndef ADCI2C_H
#define ADCI2C_H

#include "conf.h"

class AdcI2cModule: public Module {
public:
    AdcI2cModule( uint8_t **buf, uint16_t len );
    virtual ~AdcI2cModule();
    virtual uint8_t begin();
    virtual void process( char **resBufPtr );
    virtual void print();
protected:
    uint8_t i2cAddress;
    uint8_t readBuffer[ADC_NUM_DATA_CHANNELS * sizeof(uint16_t)];
};

class AdcI2cChannel: public AnalogChannel {
public:
    AdcI2cChannel( uint8_t **buf, uint16_t len );
    virtual ~AdcI2cChannel();
    uint8_t begin(uint8_t *sampleData);
    virtual bool sample();
    virtual void process( char **resBufPtr, bool forceReporting );
    virtual void print();
private:
    uint8_t *readBuffer;
};

#endif // ADCI2C_H

