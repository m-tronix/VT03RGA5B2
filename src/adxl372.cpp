#include <iostream>
#include <stdio.h>
#include "vt3r0.h"
#include "adxl372.h"
#include <string.h>
#include "bcm2835.h"
#include "i2c.h"
#

// ADXL372 related auxiliary stuff

char read_register(enum register_map reg) {
    i2c_write_char(reg);
    return i2c_read_char();
}

void read_register_buf(enum register_map reg, char *buf, uint32_t len) {
    i2c_write_char(reg);
    i2c_read_buf(buf, len);
}

void write_register(enum register_map reg, uint8_t val) {
    const char buf[] = {reg, val};
    i2c_write_buf(buf, 2);
}

unsigned int enum_to_data_rate(enum output_data_rate odr) {
    /* Returns the ODR (sample rate) in Hz.
    */
    static const unsigned int rates[] = {400, 800, 1600, 3200, 6400};
    return rates[odr];
}

Vec3i16::Vec3i16() : x(0), y(0), z(0) {}
Vec3i16::Vec3i16(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
Vec3i16::Vec3i16(const Vec3i16 &other) : x(other.x), y(other.y), z(other.z) {}
float Vec3i16::norm(void) const {
    /* Calculates the 2-norm of the vector. */
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;
    return std::sqrt(xx + yy + zz);
}


ADXL372Module::ADXL372Module( uint8_t **buf, uint16_t len ) : Module( buf, len) {
uint8_t *b;
uint8_t fLen, fType, byteValue;
uint32_t bLen;
Channel *c;

#ifdef DEBUG
cout << "> ADXLModule" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
    high_pass = false;

    status = CONF_OK;
    b = *buf;
    while (b < *buf + len) {
        fLen = *b++;
        fType = *b++;
        switch ( fType ) {
            case VT_IOB_FLD_MOD_CH_COUNT: {
#ifdef DEBUG
cout << "> VT_IOB_FLD_MOD_CH_COUNT" << endl;
#endif // DEBUG
                channelCount = *b++;
                break;
            }
            case VT_IOB_CH_BLOCK_LEN: { // New channel definition
#ifdef DEBUG
cout << "> VT_IOB_CH_BLOCK_LEN" << endl;
#endif // DEBUG
                memcpy( &bLen, b, sizeof( uint32_t ) );
                b += sizeof( uint32_t);
                c = nullptr;
                c = new MotionI2cChannel( &b, bLen );
                if ( c != nullptr ) {
                    c->addChannelProcessor( new MotionChannelProcessor( c ) );
                    channels.push_back( c );
                }
                else status = CONF_ERR_MEM;
                break;
            }
            case VT_IOB_FLD_DIG_COMPONENT: {
#ifdef DEBUG
cout << "> VT_IOB_FLD_DIG_COMPONENT" << endl;
#endif // DEBUG
                memcpy( moduleCompName, b, fLen-1 );
                moduleCompNameLen = fLen-1;
                moduleCompName[fLen-1] = 0;
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_DIG_VALUE_TYPE: {
#ifdef DEBUG
cout << "> VT_IOB_FLD_DIG_VALUE_TYPE" << endl;
#endif // DEBUG
                byteValue = *b++;
                if ( byteValue <= VT_IOB_VALUE_DOUBLE ) moduleValueType = byteValue;
                else status = CONF_ERR_VALUE;
                break;
            }
            case VT_IOB_FLD_DIG_ENDIAN: {
                byteValue = *b++;
#ifdef DEBUG
cout << "> VT_IOB_FLD_DIG_ENDIAN";
printf(": %d\n",byteValue);
#endif // DEBUG
                if ( byteValue <= VT_IOB_ENDIAN_BE ) moduleEndian = byteValue;
                else status = CONF_ERR_VALUE;
                break;
            }
            case VT_IOB_FLD_I2C_ADDR: {
#ifdef DEBUG
cout << "> VT_IOB_FLD_I2C_ADDR" << endl;
#endif // DEBUG
                byteValue = *b++;
                if ( byteValue <= 0x7f ) i2cAddress = byteValue;
                else status = CONF_ERR_VALUE;
                break;
            }
            case VT_IOB_FLD_I2C_INIT_CMD: {
 #ifdef DEBUG
cout << "> VT_IOB_FLD_I2C_INIT_CMD" << endl;
#endif // DEBUG
               memcpy( moduleInitCmd, b, fLen-1);
                moduleInitCmdLen = fLen-1;
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_I2C_STOP_CMD: {
#ifdef DEBUG
cout << "> VT_IOB_FLD_I2C_STOP_CMD" << endl;
#endif // DEBUG
                memcpy( moduleStopCmd, b, fLen-1);
                moduleStopCmdLen = fLen-1;
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_I2C_READ_LEN: {
 #ifdef DEBUG
cout << "> VT_IOB_FLD_I2C_READ_LEN" << endl;
#endif // DEBUG
               moduleReadLength = *b++;
                break;
            }
            case VT_IOB_FLD_ADC_BITS: {
#ifdef DEBUG
cout << "> VT_IOB_FLD_ADC_BITS" << endl;
#endif // DEBUG
                moduleAdcBits = *b++;
                break;
            }
            case VT_IOB_FLD_ADC_SHIFT: {
#ifdef DEBUG
cout << "> VT_IOB_FLD_ADC_SHIFT" << endl;
#endif // DEBUG
                moduleAdcShift = *b++;
                break;
            }
            case VT_IOB_FLD_ADC_BITMASK: {
#ifdef DEBUG
cout << "> VT_IOB_FLD_ADC_BITMASK" << endl;
#endif // DEBUG
                memcpy( &moduleAdcBitMask, b, sizeof( uint32_t ));
                moduleStopCmdLen = fLen-1;
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_MOD_MIN_INTERVAL_MS: {    // uint32_t Maximum valid sampling interval (ms)
#ifdef DEBUG
cout << "> VT_IOB_FLD_MOD_MIN_INTERVAL_MS" << endl;
#endif // DEBUG
                memcpy( &moduleMinSamplingInterval, b, fLen-1);
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_MOD_MAX_INTERVAL_MS: {    // uint32_t Maximum valid sampling interval (ms)
#ifdef DEBUG
cout << "> VT_IOB_FLD_MOD_MAX_INTERVAL_MS" << endl;
#endif // DEBUG
                memcpy( &moduleMaxSamplingInterval, b, fLen-1);
                b += fLen-1;
                break;
            }
            default: {
#ifdef DEBUG
printf("Unknown field type: %x\n" , fType );
#endif // DEBUG
                status = CONF_WARN_UNK;
                b += fLen-1;
            }
        }
    }
    *buf = b;
    // done
#ifdef DEBUG
cout << "< AdcI2cModule" << endl;
#endif
}

ADXL372Module::~ADXL372Module() {}

void ADXL372Module::set_power_mode(enum power_mode mode, bool high_pass) {
    /* Sets the POWER_CTL register. (In effect, turns the device on or off.) See
       page 52 of the datasheet.
    */
    char value = mode;
    this->high_pass = high_pass;
    if (!high_pass) {
        // HPF_DISABLE
        value |= 0x04;
    }
    // Enable i2c high speed mode
    value |= 0x80;
    write_register(register_map::POWER_CTL, value);
}

void ADXL372Module::set_timing(enum output_data_rate rate) {
    /* Se the TIMING register: EXT_SYNC, EXT_CLK, WAKEUP_RATE, ODR.
       We only care about the ODR = output data rate = sample rate.
    */
    uint8_t wakeup_rate = 0x0; // = 52 ms, doesn't matter.
    uint8_t timing_val = (rate << 5) | (wakeup_rate << 2);
    write_register(register_map::TIMING, timing_val);
    // Save the rate in Hz as integer internally.
    output_data_rate = enum_to_data_rate(rate);
}

void ADXL372Module::set_measurement_settings(enum filter_bandwidth bw, bool low_noise) {
    uint8_t value = bw | (low_noise << 3);
    write_register(register_map::MEASURE, value);
}

void ADXL372Module::set_fifo(unsigned int samples) {
    /* Setup the fifo in xyz-samples stream mode, with the fifo full watermark
       at /samples/. Maximum samples to fit in the fifo is 170. The device must
       be in standby power mode.
    */
    samples *= 3;
    uint8_t fifo_samples_lsb = 0x00ff & samples;
    uint8_t fifo_samples_msb = (0x0100 & samples) >> 8;

    uint8_t fifo_ctl = fifo_samples_msb | 0x02;
    write_register(register_map::FIFO_CTL, fifo_ctl);
    write_register(register_map::FIFO_SAMPLES, fifo_samples_lsb);
}

void MotionI2cChannel::set_offsets(const Vec3i16 &offsets) {
    this->offsets = offsets;
}

uint8_t ADXL372Module::begin() {
uint8_t cnt;
Channel *c;
    bcm2835_i2c_setSlaveAddress(MOTIONADDR);
    write_register(register_map::RESET, RESET_DEVICE);
    set_power_mode(power_mode::STANDBY);

    set_measurement_settings(filter_bandwidth::BW_400, true);
    set_timing(output_data_rate::HZ_1600);
    set_fifo(FIFO_SIZE);

    set_power_mode(power_mode::MEASUREMENT, false);

    cnt = 0;
     for ( it = channels.begin(); it != channels.end(); it++ ) {
        c = *it;
        c->begin( &readBuffer[cnt*2] ); //TODO: korjaa vähemmän hirveäksi
        cnt++;
    }
    return CONF_OK;
}

void ADXL372Module::process(  char **resBufPtr ) {
    bcm2835_i2c_setSlaveAddress(MOTIONADDR);
    Channel *c;
#ifdef DEBUG
        cout << ">ADXL372Module.process" << endl;
#endif // DEBUG
    for ( it = channels.begin(); it != channels.end(); it++ ) {
        c = *it;
        c->sample();
    }
    Module::process(resBufPtr);
}




// multiaxis motion sensor
MotionI2cChannel::MotionI2cChannel( uint8_t **buf, uint16_t len )  : AnalogChannel( buf, len){
uint8_t fLen;
uint8_t fType;
uint8_t *b;
float tmp[3];
#ifdef DEBUG
cout << "> MotionI2cChannel" << endl;
    printBuffer( *buf, len );
#endif // DEBUG

    // parse the EEPROM block
    status = CONF_OK;
    b = *buf;
    while ( b < *buf + len ) {
        fLen = *b++;
        fType = *b++;
        #ifdef DEBUG
        printf("fLen %x, fType %x\n", fLen, fType);
        #endif
        switch ( fType ) {
            case VT_IOB_FLD_CH_IDX: {       // uint8_t Channel index, starting from 0
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_IDX" << endl;
#endif // DEBUG
                channelIndex = *b++;
                break;
            }
            case VT_IOB_FLD_CH_PHY: {       // char[32] Physical quantity
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_PHY" << endl;
#endif // DEBUG
                memcpy( channelPhysicalQuantity, b, fLen-1);
                channelPhysicalQuantityLen = fLen-1;
                channelPhysicalQuantity[fLen-1] = 0;
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_CH_SCAL: {      // float[4] Scaling parameters (RAW --> SI-unit)
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_SCAL" << endl;
#endif // DEBUG
                memcpy( channelScalingFactor, b, fLen-1);
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_CH_MIN_VAL: {   // int32_t Minimum valid output (RAW / mV)
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_MIN_VAL" << endl;
#endif // DEBUG
                memcpy( &channelMinValid, b, fLen-1);
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_CH_MAX_VAL: {   // int32_t Maximum valid output (RAW / mV)
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_MAX_VAL" << endl;
#endif // DEBUG
                memcpy( &channelMaxValid, b, fLen-1);
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_CH_ACC: {       // uint8_t Accuracy class
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_ACC" << endl;
#endif // DEBUG
                channelAccuracyClass = *b++;
                break;
            }
            case VT_IOB_FLD_CH_THRESHOLD_REL: {      // min. relative change to report
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_THRESHOLD_REL" << endl;
#endif // DEBUG
                memcpy( &thresholdRel, b, fLen-1);
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_CH_THRESHOLD_ABS: {      // min. absolute change to report
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_THRESHOLD_REL" << endl;
#endif // DEBUG
                memcpy( &thresholdAbs, b, fLen-1);
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_CH_MOTION_OFFSETS: {      // min. change to report
#ifdef DEBUG
cout << "> VT_IOB_FLD_CH_MOTION_OFFSETS" << endl;
#endif // DEBUG
                memcpy( &tmp, b, fLen-1);
                set_offsets(Vec3i16(tmp[0], tmp[1], tmp[2]));
//printf("offsets: %f, %f, %f\n", tmp[0], tmp[1], tmp[2] );
                b += fLen-1;
                break;
            }
            default: {
#ifdef DEBUG
printf("Unknown field type: %x\n" , fType );
#endif // DEBUG
                status = CONF_WARN_UNK;
                b += fLen-1;
            }
        }
    }
    *buf = b;
#ifdef DEBUG
cout << "< MotionI2cChannel" << endl;
#endif // DEBUG
}

MotionI2cChannel::~MotionI2cChannel() {}


uint8_t MotionI2cChannel::begin(uint8_t *sampleData) {
#ifdef DEBUG
        cout << ">MotionI2cChannel.begin)" << endl;
#endif // DEBUG
#ifdef DEBUG
        cout << "<MotionI2cChannel.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

struct adxl372_status MotionI2cChannel::read_status(void) {
    /* Returns the contents of the status register and the number of samples
       in the fifo as a struct.
    */
    struct adxl372_status status;
    uint8_t data[4];
    read_register_buf(register_map::STATUS, (char *)data, 4);

    status.data_rdy      = (data[0] & status::DATA_RDY) != 0;
    status.fifo_rdy      = (data[0] & status::FIFO_RDY) != 0;
    status.fifo_ovr      = (data[0] & status::FIFO_OVR) != 0;
    status.user_nvm_busy = (data[0] & status::USER_NVM_BUSY) != 0;
    status.awake         = (data[0] & status::AWAKE) != 0;
    status.err_user_regs = (data[0] & status::ERR_USER_REGS) != 0;

    status.samples = (data[2] << 8 | data[3]) / 3;
    return status;
}

uint16_t MotionI2cChannel::update(uint16_t num) {
    /* Read num samples from the fifo and pass them through the filtering chain.
       Returns the number of processed samples ready.
    */
    std::vector<Vec3i16> raw_xyz;
    raw_xyz.reserve(num);
    read_fifo_xyz(raw_xyz, num);

    for (const Vec3i16 &sample : raw_xyz) {
        median_buf_x.push_back(sample.x);
        median_buf_y.push_back(sample.y);
        median_buf_z.push_back(sample.z);
        if (median_buf_x.size() == MEDIAN_SAMPLES) {
            // There are enough samples to calculate median.
            Vec3i16 median_value(
                find_median(median_buf_x),
                find_median(median_buf_y),
                find_median(median_buf_z)
            );
            filtered_samples.push_back(median_value);

            // Shift the values in the buffers forwards and pop the first, which
            // is the last after the rotation. This is of course not very
            // efficient, a circular buffer would be better. This will probably
            // work nicely for median filters of size 3 or 5 items.
            std::rotate(median_buf_x.begin(), median_buf_x.begin() + 1, median_buf_x.end());
            std::rotate(median_buf_y.begin(), median_buf_y.begin() + 1, median_buf_y.end());
            std::rotate(median_buf_z.begin(), median_buf_z.begin() + 1, median_buf_z.end());
            median_buf_x.pop_back();
            median_buf_y.pop_back();
            median_buf_z.pop_back();
        }
    }
    return filtered_samples.size();
}

bool MotionI2cChannel::sample() {
    struct adxl372_status status = read_status();
    if (status.samples) {
        unsigned int num_filtered = update(status.samples);
        if (num_filtered > 0) return true;
        else return false;
    }
    return false;

}

void MotionI2cChannel::read_fifo_xyz(std::vector<Vec3i16> &buf, uint16_t num) {
    /* Fills the given buffer buf with num samples read from the FIFO.
    */
    std::vector<uint8_t> fifo_data(6 * num);
    read_register_buf(register_map::FIFO_DATA, (char *)&fifo_data[0], 6 * num);

    buf.clear();

    for (unsigned int i = 0; i < num; i++) {
        buf.push_back(Vec3i16(
            combine_int12(fifo_data[i * 6 + 0], fifo_data[i * 6 + 1]) + offsets.x,
            combine_int12(fifo_data[i * 6 + 2], fifo_data[i * 6 + 3]) + offsets.y,
            combine_int12(fifo_data[i * 6 + 4], fifo_data[i * 6 + 5]) + offsets.z
        ));
    }
}

int16_t MotionI2cChannel::combine_int12(uint8_t msb, uint8_t lsb) const {
    /* Combines a msb and lsb byte to a 12 bit signed value.
    */
    uint16_t combined = msb << 8 | lsb;
    return ((int16_t)combined) >> 4; // NOTE: Relying on sign extension.
}

int16_t MotionI2cChannel::find_median(std::vector<int16_t> median_buf) const {
    std::nth_element(
        median_buf.begin(),
        median_buf.begin() + median_buf.size() / 2,
        median_buf.end()
    );
    return median_buf[median_buf.size() / 2];
}

void MotionI2cChannel::process( char **resBufPtr, bool forceReporting ) {
    Channel::process(resBufPtr, forceReporting);
}

void MotionI2cChannel::print() {}



void MotionChannelProcessor::process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting ) {
uint16_t n;
#ifdef DEBUG
        cout << ">MotionChannelProcessor.process()" << endl;
#endif // DEBUG

//Mikon koodi
    /* Update the RMS vibration value from the data in filtered_samples. The
       value is smoothed with filtering with the given time constant. The
       filtered_samples buffer must be clear()ed between calls to this function!
       The filtering performed is a simple exponentially weighted moving average.
    */
    // RMS filter constant calculation
float sample_interval = 1.0f / static_cast<MotionI2cChannel*>(parent)->output_data_rate;
float alpha = 1.0f - std::exp(-sample_interval / 1.0 );

    for (const Vec3i16 &sample : static_cast<MotionI2cChannel*>(parent)->filtered_samples) {
        float norm = 0.1f * sample.norm(); // Convert to g:s from 100 milligees.
        newRMSValue = alpha * std::abs(norm - 1.0f) + (1 - alpha) * newRMSValue;
    }
    // The filtered samples buffer vector must be cleared manually!
    static_cast<MotionI2cChannel*>(parent)->filtered_samples.clear();

//-----------------------
    // If above minimum reporting limit and changed more than min. rel. change, report
    if (newRMSValue > parent->thresholdAbs || currRMSValue > parent->thresholdAbs || forceReporting) {
        if ( ( newRMSValue > currRMSValue * (1.0 + parent->thresholdRel ) ) ||
             ( currRMSValue > newRMSValue * (1.0 + parent->thresholdRel ) ) || forceReporting) {
            currRMSValue = newRMSValue;
            n = sprintf(*resBufPtr, "\"%s\": %f", parent->channelPhysicalQuantity, newRMSValue);
            *resBufPtr += n;   // ignore terminating 0
        }
    }
#ifdef DEBUG
        printf("RMS: %f \n", newRMSValue);
        cout << "<MotionhannelProcessor.process()" << endl;
#endif // DEBUG

}

