#ifndef ADXL372_H
#define ADXL372_H

#include <stdint.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include "vt3r0.h"
#include "module.h"


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

class MotionI2cChannel: public AnalogChannel {
    friend class MotionChannelProcessor;
public:
    MotionI2cChannel( uint8_t **buf, uint16_t len );
    virtual ~MotionI2cChannel();
    uint8_t begin(uint8_t *sampleData);
    void set_offsets(const Vec3i16 &offsets);
    struct adxl372_status read_status(void);
    uint16_t update(uint16_t num);
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
 };



class MotionChannelProcessor: public ChannelProcessor {
public:
    MotionChannelProcessor( Channel *c ) : ChannelProcessor(c) {};
    void process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting );
protected:
    float newRMSValue = 0;
    float currRMSValue = 0;
};


#endif

