#ifndef MODULE_H
#define MODULE_H

#include <stdint.h>
#include <time.h>
#include <chrono>
#include <vector>

#include "vt3r0.h"

//#define PRBUF // define for printing out the EEPROM read buffer contents
void printBuffer(uint8_t * buf, uint16_t len);


class Module;
class Channel;
class ChannelProcessor;

using namespace std;
using namespace std::literals::chrono_literals; // ns, us, ms, s, h, etc.
using namespace std::chrono;
using std::chrono::steady_clock;
using interval_ms = duration<steady_clock::rep, std::ratio<1,1000>>;

// I/O-module base class
class Module {
public:
    Module( uint8_t **buf, uint16_t len );
    virtual ~Module();
    virtual uint8_t begin();
    virtual uint8_t getStatus() { return status; }
    virtual void print();
    virtual void process( char **resBufPtr );
    virtual std::chrono::steady_clock::time_point getLastProcessed() { return lastProcessed; };
    virtual std::chrono::steady_clock::time_point getLastReported() { return lastReported; };
    virtual void setLastProcessed(std::chrono::steady_clock::time_point t) { lastProcessed = t; };
    virtual void setLastReported(std::chrono::steady_clock::time_point t) { lastReported = t; };
    uint32_t getModuleMinSamplingInterval() { return moduleMinSamplingInterval; }
    uint32_t getModuleMaxSamplingInterval() { return moduleMaxSamplingInterval; }
protected:
    uint8_t status;
    uint8_t moduleCompName[256];
    uint8_t moduleCompNameLen;
    uint8_t moduleValueType;
    uint8_t moduleEndian;
    uint8_t moduleAdcBits;
    uint8_t moduleAdcShift;
    uint32_t moduleAdcBitMask;
    uint8_t moduleInitCmd[256];
    uint8_t moduleInitCmdLen;
    uint8_t moduleStopCmd[256];
    uint8_t moduleStopCmdLen;
    uint8_t moduleReadLength;
    uint8_t moduleReadOffset;
    uint64_t moduleMinSamplingInterval;
    uint64_t moduleMaxSamplingInterval;
    uint8_t channelCount;
    std::vector<Channel*> channels;
    std::vector<Channel*>::iterator it;
    std::chrono::steady_clock::time_point lastProcessed;
    std::chrono::steady_clock::time_point lastReported;
private:
};

// measurement channel base class
class Channel {
friend class RMSChannelProcessor;
template <class T>
friend class AVGChannelProcessor;
friend class DigitalInChannelProcessor;
friend class MotionChannelProcessor;
public:
    Channel( uint8_t **buf, uint16_t len );
    virtual ~Channel();
    virtual uint8_t begin( uint8_t *sampleData ) =0;
    virtual void addChannelProcessor( ChannelProcessor *cP ) { processors.push_back( cP ); };
    virtual bool sample() =0;
    virtual void process( char **resBufPtr, bool forceReporting ) =0;
    virtual void print();
    uint8_t getIndex() { return channelIndex; };
protected:
    uint8_t status;
    uint8_t channelIndex;
    uint8_t channelPhysicalQuantity[32];
    uint8_t channelPhysicalQuantityLen;
    uint32_t channelMinSamplingInterval;
    uint32_t channelMaxSamplingInterval;
    float   channelScalingFactor[4];
    uint8_t *dataBuffer;    // sampled data
    uint16_t dataBufferLen; // length of allocated sample buffer
    uint16_t dataBufferCnt; // index to the next free buffer location
    float thresholdRel = 0.1;
    float thresholdAbs = 1;
    std::vector<ChannelProcessor*> processors;
    std::vector<ChannelProcessor*>::iterator it;
};


class AnalogChannel: public Channel {
public:
    AnalogChannel( uint8_t **buf, uint16_t len );
    virtual ~AnalogChannel();
    virtual uint8_t begin(uint8_t *sampleData) =0;
    virtual bool sample() =0;
    virtual void process( char **resBufPtr, bool forceReporting ) =0;
    virtual void print() =0;
protected:
    int32_t channelMinValid;
    int32_t channelMaxValid;
    uint8_t channelAccuracyClass;
};

class DigitalChannel: public Channel {
public:
    DigitalChannel( uint8_t **buf, uint16_t len );
    virtual ~DigitalChannel();
    uint8_t begin(uint8_t *sampleData) =0;
    virtual bool sample()=0;
    virtual void process( char **resBufPtr, bool forceReporting )=0;
    virtual void print() =0;
protected:
    uint8_t ioPin;
};

// Channel processor base class
class ChannelProcessor {
public:
    ChannelProcessor( Channel *c ) {parent = c; };
    virtual void process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting );
protected:
    Channel *parent;
};

class RMSChannelProcessor: public ChannelProcessor {
public:
    RMSChannelProcessor( Channel *c ) : ChannelProcessor(c) {};
    void process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting );
protected:
    float currRMSValue;
};

template <class T>
class AVGChannelProcessor: public ChannelProcessor {
public:
    AVGChannelProcessor( Channel *c ) : ChannelProcessor(c) {};
    void process( T *buffer, uint16_t len, char **resBufPtr, bool forceReporting );
protected:
    float currAVGValue;
};


#endif
