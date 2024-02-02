#ifndef DIGITALIN_H
#define DIGITALIN_H

#include <stdint.h>
#include "module.h"

class DigitalInModule: public Module {
public:
    DigitalInModule( uint8_t **buf, uint16_t len );
    virtual ~DigitalInModule();
    virtual uint8_t begin();
    virtual void process( char **resBufPtr );
    virtual void print();
protected:
};

class DigitalInChannel: public DigitalChannel {
public:
    DigitalInChannel( uint8_t **buf, uint16_t len );
    virtual ~DigitalInChannel();
    uint8_t begin(uint8_t *sampleData);
    virtual bool sample();
    virtual void process( char **resBufPtr, bool forceReporting );
    virtual void print();
};

class DigitalInChannelProcessor: public ChannelProcessor {
public:
    DigitalInChannelProcessor( Channel *c ) : ChannelProcessor(c) {};
    void process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting );
protected:
    uint8_t currResult;
};

#endif // DIGITALIN_H

