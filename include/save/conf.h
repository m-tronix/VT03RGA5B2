/*
 * VT field device configuration class
 * (c) 2019 Viimatech Digital Oy
 * Author: Martti Paalanen / M-Tronix t:mi
 *
 * A VT device configuration specifies the I/O as a set of modules according
 * to the nature of the connected signal and the connection to the processing
 * unit such as I2C or direct GPIO.
 * Each module controls a number of channels that do the actual interfacing
 * For each channel a number of channel processors can be attached. Each processor
 * produces a specific measurement value such as the RMS value of the signal or
 * simple on/off value etc.
 *
 * The configuration class has the following functionality:
 * - VT field device configuration memory parser
 * - construct the configuration class hierarchy:
 *   conf
 *     module
 *       channel
 *         channel processor
 * - process the configuration to produce measurement values (returns json strings)
*/

#ifndef CONF_H
#define CONF_H

#include <stdint.h>
#include <vector>
#include <time.h>
#include <chrono>

#include "vt3r0.h"

#include "../../../spectral/spectrum_analyzer.hpp"

#define CONF_OK         0
#define CONF_ERR_VALUE  1
#define CONF_ERR_SYNTAX 2
#define CONF_ERR_STRUCT 3
#define CONF_ERR_MEM    4
#define CONF_ERR_I2C    5
#define CONF_WARN_UNK   100

#define DIGITAL_IN_TOGGLE_COUNT 20


using namespace std;
using namespace std::literals::chrono_literals; // ns, us, ms, s, h, etc.
using namespace std::chrono;
using std::chrono::steady_clock;

class Module;
class Channel;
class ChannelProcessor;

typedef union {
    uint8_t b[8];
    uint16_t uw[4];
    int16_t w[4];
    uint32_t ul[2];
    int32_t l[2];
    float f[2];
    double lf;
} cPResult;

namespace adxl372 {

// ADXK372 related stuff
class Vec3i16 {
public:
    Vec3i16();
    Vec3i16(int16_t x, int16_t y, int16_t z);
    Vec3i16(const Vec3i16 &other);
    int16_t x;
    int16_t y;
    int16_t z;

    float norm(void) const;
};

static const char RESET_DEVICE = 0x52; // Write to RESET register.
struct adxl372_status {
    /* A complete x, y, and z measurement has been made and results can be read.
       doesnt' really matter when using the fifo.
    */
    uint16_t data_rdy : 1;
    /* FIFO Ready. At least one valid sample is available in the FIFO. */
    uint16_t fifo_rdy : 1;
    /* FIFO Watermark. The FIFO watermark level, specified in FIFO_SAMPLES,
       has been reached.
    */
    uint16_t fifo_full : 1;
    /* FIFO Overrun. FIFO has overflowed, and data has been lost. */
    uint16_t fifo_ovr : 1;
    /* Nonvolatile memory (NVM) is busy programming fuses. */
    uint16_t user_nvm_busy : 1;
    uint16_t awake : 1;
    /* SEU Event. An SEU event has been detected in a user register. ("An SEU
       is a change of state caused by ions or electromagnetic radiation striking
       a sensitive node in a micro-electronic device.") The bit is high for an
       unitialized device.
    */
    uint16_t err_user_regs : 1;
    /* Number of samples available to be read in the FIFO. */
    uint16_t samples;
};
}
//----------------
using interval_ms = duration<steady_clock::rep, std::ratio<1,1000>>;

// Configuration base class
class Conf {
public:
    Conf( uint8_t i2cAddr );
    virtual ~Conf();
    uint8_t begin();
    uint8_t buildFromEEPROM();
    void process(char **resBufPtr);
    void getUUID( char **ID );
    uint8_t getStatus() { return status; }
    void print();

protected:
    uint8_t status;
    uint32_t boardModel;
    char boardUUID[16];
    uint8_t moduleCount;
    std::vector<Module*> modules;
    std::vector<Module*>::iterator it;
private:
};

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
    uint32_t moduleMinSamplingInterval;
    uint32_t moduleMaxSamplingInterval;
    uint8_t channelCount;
    std::vector<Channel*> channels;
    std::vector<Channel*>::iterator it;
    std::chrono::steady_clock::time_point lastProcessed;
    std::chrono::steady_clock::time_point lastReported;
private:
};

class AdcI2cModule: public Module {
public:
    AdcI2cModule( uint8_t **buf, uint16_t len );
    virtual ~AdcI2cModule();
    virtual uint8_t begin();
    virtual void process( char **resBufPtr );
    virtual void print();
protected:
    uint8_t i2cAddress;
    uint8_t readBuffer[6];
};

namespace adxl372 {

class ADXL372Module: public Module {    // ADXL372 motion sensor
public:
    ADXL372Module( uint8_t **buf, uint16_t len );
    virtual ~ADXL372Module();
    virtual uint8_t begin();
    void set_power_mode(enum power_mode mode, bool high_pass=false);
    void set_timing(enum output_data_rate rate);
    void set_measurement_settings(enum filter_bandwidth bw, bool low_noise = 0);
    void set_fifo(unsigned int samples = 128);
    virtual void process( char **resBufPtr );
//    virtual void print();
protected:
    bool high_pass;
    unsigned int output_data_rate = 0;
    uint8_t i2cAddress;
    uint8_t readBuffer[6];
};
}

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

class DigitalInModule: public Module {
public:
    DigitalInModule( uint8_t **buf, uint16_t len );
    virtual ~DigitalInModule();
    virtual uint8_t begin();
    virtual void process( char **resBufPtr );
    virtual void print();
protected:
};

// measurement channel base class
class Channel {
friend class RMSChannelProcessor;
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

namespace adxl372 {

class MotionI2cChannel: public AnalogChannel {
    friend class MotionChannelProcessor;
public:
    MotionI2cChannel( uint8_t **buf, uint16_t len );
    virtual ~MotionI2cChannel();
    uint8_t begin(uint8_t *sampleData);
    void set_offsets(const Vec3i16 &offsets);
    struct adxl372_status read_status(void);
    // uint16_t update(uint16_t num);
    virtual bool sample();
    void read_fifo_xyz(std::vector<Vec3i16> &buf, uint16_t num);
    int16_t combine_int12(uint8_t msb, uint8_t lsb) const;
    int16_t find_median(std::vector<int16_t> median_buf) const;
    virtual void process( char **resBufPtr, bool forceReporting );
    virtual void print();
    std::vector<Vec3i16> filtered_samples;
    uint16_t output_data_rate = 0;
protected:
    adxl372_status mStatus;
    Vec3i16 offsets;
    std::vector<int16_t> median_buf_x;
    std::vector<int16_t> median_buf_y;
    std::vector<int16_t> median_buf_z;

//    SpectrumAnalyzer spectrum_analyzer;
//    std::vector<int16_t> acceleration_magnitudes;
 };
}

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

class DigitalInChannel: public DigitalChannel {
public:
    DigitalInChannel( uint8_t **buf, uint16_t len );
    virtual ~DigitalInChannel();
    uint8_t begin(uint8_t *sampleData);
    virtual bool sample();
    virtual void process( char **resBufPtr, bool forceReporting );
    virtual void print();
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

class AVGChannelProcessor: public ChannelProcessor {
public:
    AVGChannelProcessor( Channel *c ) : ChannelProcessor(c) {};
    void process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting );
protected:
    float currAVGValue;
};

class MotionChannelProcessor: public ChannelProcessor {
public:
    MotionChannelProcessor( Channel *c ) : ChannelProcessor(c) {};
    void process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting );
protected:
    float newRMSValue = 0;
    float currRMSValue = 0;
};


class DigitalInChannelProcessor: public ChannelProcessor {
public:
    DigitalInChannelProcessor( Channel *c ) : ChannelProcessor(c) {};
    void process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting );
protected:
    uint8_t currResult;
};




// static helper instances
#endif // CONF_H
