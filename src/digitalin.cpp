#include <iostream>
#include "digitalin.h"
#include <string.h>
#include <thread>
#include "bcm2835.h"


DigitalInModule::DigitalInModule( uint8_t **buf, uint16_t len ) : Module( buf, len) {
uint8_t *b;
uint8_t fLen, fType, byteValue;
uint32_t bLen;
Channel *c;

#ifdef DEBUG
cout << "> DigitalInModule" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
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
                c = new DigitalInChannel( &b, bLen );
                if ( c != nullptr ) {
                    c->addChannelProcessor( new DigitalInChannelProcessor( c ) );
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
cout << "< DigitalInModule" << endl;
#endif
}

DigitalInModule::~DigitalInModule() {
}

uint8_t DigitalInModule::begin() {
Channel *c;
#ifdef DEBUG
        cout << ">DigitalInModule.begin)" << endl;
#endif // DEBUG
    for ( it = channels.begin(); it != channels.end(); it++ ) {
        c = *it;
        c->begin( nullptr );
    }
#ifdef DEBUG
        cout << "<DigitalInModule.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

void DigitalInModule::process( char **resBufPtr ) {
uint16_t cnt;
Channel *c;
#ifdef DEBUG
        cout << ">DigitalInModule.process)" << endl;
#endif // DEBUG
    // set the sampling interval
    std::chrono::microseconds period(SAMPLEINTERVALUS);
    auto next = std::chrono::high_resolution_clock::now() + period;
    // periodically sample the digital input channels until
    // number of samples has been collected
    for ( cnt = 0; cnt < NUMMEASCOUNT_DIGITAL; cnt++ ) {
        for ( it = channels.begin(); it != channels.end(); it++ ) {
            c = *it;
            c->sample();
        }
        std::this_thread::sleep_until(next);
        next += period;
    }
    // when sample count full, process the channels
    Module::process( resBufPtr );
#ifdef DEBUG
        cout << "<DigitalInModule.process)" << endl;
#endif // DEBUG
}

void DigitalInModule::print() {
    printf("Digital input module:\n");
    Module::print();
}


DigitalChannel::DigitalChannel( uint8_t **buf, uint16_t len )  : Channel( buf, len){
#ifdef DEBUG
cout << "> DigitalChannel" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
#ifdef DEBUG
cout << "< DigitalChannel" << endl;
#endif // DEBUG
}

DigitalChannel::~DigitalChannel() {}

/*
uint8_t DigitalChannel::begin(uint8_t *sampleData) {
    return CONF_OK;
}
void DigitalChannel::sample() {}
void DigitalChannel::process() {}
void DigitalChannel::print() {}
*/



DigitalInChannel::DigitalInChannel( uint8_t **buf, uint16_t len )  : DigitalChannel( buf, len){
uint8_t fLen;
uint8_t fType;
uint8_t *b;
#ifdef DEBUG
cout << "> DigitalInChannel" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
    dataBufferLen = NUMMEASCOUNT_DIGITAL * sizeof(uint8_t); // TODO: sample datatype size should be read from conf!
    dataBuffer = new uint8_t[dataBufferLen];
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
            case VT_IOB_FLD_IO_PIN: {       // GPIO input pin of the board MCU connector
#ifdef DEBUG
cout << "> VT_IOB_FLD_IO_PIN" << endl;
#endif // DEBUG
                ioPin = *b++;
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
cout << "< DigitalInChannel" << endl;
#endif // DEBUG
}

DigitalInChannel::~DigitalInChannel() {}

// Initialize the digital input channel.
// GPIO pin is set as input
// the function argument is not used for anything
uint8_t DigitalInChannel::begin(uint8_t *sampleData) {
#ifdef DEBUG
        cout << ">DigitalInChannel.begin)" << endl;
#endif // DEBUG
    bcm2835_gpio_fsel( ioPin, BCM2835_GPIO_FSEL_INPT);
    dataBufferCnt = 0;
#ifdef DEBUG
        cout << "<DigitalInChannel.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

// Add a new sample to the channel data buffer.
// The sample is read directly from the I/O pin.
bool DigitalInChannel::sample() {
    if (dataBufferCnt < dataBufferLen ) {
        *(dataBuffer+dataBufferCnt) = bcm2835_gpio_lev( ioPin );
        dataBufferCnt++;
        if (dataBufferCnt < dataBufferLen ) return false;
        else return true;
    }
    else return true;
}

void DigitalInChannel::process( char **resBufPtr, bool forceReporting) {
#ifdef DEBUG
        cout << ">DigitalInChannel.process()" << endl;
        printf("bufsize %d \n", dataBufferLen);
        printf("samples %d \n", dataBufferCnt);
#endif // DEBUG
    Channel::process( resBufPtr, forceReporting );
    // After processsing the channel we restart the sampling
    // cycle by resetting buffer sample count to zero
    dataBufferCnt =0;
#ifdef DEBUG
        cout << "<DigitalInChannel.process()" << endl;
#endif // DEBUG
}

void DigitalInChannel::print() { Channel::print(); }


void DigitalInChannelProcessor::process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting ) {
uint16_t cnt;
uint16_t sampleSum;
uint8_t newResult;
int16_t n;
#ifdef DEBUG
        cout << ">DigitalInChannelStateProcessor.process()" << endl;
#endif // DEBUG
    sampleSum = 0;
    for ( cnt = 0; cnt < len; cnt++ ) {
        sampleSum += *(buffer+cnt);
    }
    if ( sampleSum > (len / 2)) newResult = 1;
    else newResult = 0;
    if ( ( newResult != currResult ) || forceReporting ) {
        currResult = newResult;
        n = sprintf(*resBufPtr, "\"%s\": %d", parent->channelPhysicalQuantity, newResult);
        *resBufPtr += n;   // ignore terminating 0
    }
#ifdef DEBUG
        printf("sampled %d times\n", len);
        cout << "<DigitalInChannelStateProcessor.process()" << endl;
#endif // DEBUG
}
//-----------------------------

