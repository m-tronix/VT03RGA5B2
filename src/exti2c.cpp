#include <iostream>
#include <string.h>
#include <thread>
#include "bcm2835.h"
#include "module.h"
#include "exti2c.h"

#define DEBUG

// MPa 20210630
ExtI2cModule::ExtI2cModule( uint8_t **buf, uint16_t len ) : Module( buf, len) {
uint8_t *b;
uint8_t fLen, fType, byteValue;
uint32_t bLen;
Channel *c;

#ifdef DEBUG
cout << "> ExtI2cModule" << endl;
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
                c = new ExtI2cChannel( &b, bLen );
                if ( c != nullptr ) {
                    // 4-20mA channel gets average value processor, CT's get RMS
                    if ( c->getIndex() == 2 ) c->addChannelProcessor( new AVGChannelProcessor<uint32_t>( c ) );
                    else c->addChannelProcessor( new RMSChannelProcessor( c ) );
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
            case VT_IOB_FLD_I2C_READ_OFFS: {
 #ifdef DEBUG
cout << "> VT_IOB_FLD_I2C_READ_OFFS" << endl;
#endif // DEBUG
               moduleReadOffset = *b++;
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

ExtI2cModule::~ExtI2cModule() {
}

uint8_t ExtI2cModule::begin() {
uint8_t result;
//uint8_t bufIncrement;
uint8_t cnt;
Channel *c;
#ifdef DEBUG
        cout << ">ExtI2cModule.begin)" << endl;
#endif // DEBUG
    // initialize the external module, if an initialization has been specified
    if ( moduleInitCmdLen > 0 ) {
        bcm2835_i2c_setSlaveAddress(EXPADDR);
        result = bcm2835_i2c_write((char *)moduleInitCmd, moduleInitCmdLen);
        if ( result != BCM2835_I2C_REASON_OK ) {
            status = CONF_ERR_I2C;
            return status;
        }
    }
    //Allocate a buffer to hold the data read from the converter
//    readBuffer = (uint8_t*) malloc (moduleReadLength);
//    bufIncrement = moduleReadLength / channelCount;
    //Each channel gets the address of the data element corresponding to that channel
    // Init the channel
    cnt = 0;
     for ( it = channels.begin(); it != channels.end(); it++ ) {
        c = *it;
        c->begin( &readBuffer[cnt*sizeof(uint32_t)] );
        cnt++;
    }
#ifdef DEBUG
        cout << "<ExtI2cModule.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

// Module processor periodically reads the A/D converter and distributes the samples
// to the individual channels ( the sample() function ).
// The first sample() that returns true triggers channel processing where
// each channel in turn is processed.
void ExtI2cModule::process( char **resBufPtr) {
Channel *c;
uint8_t result;
bool rdy;
char offs = 0;
#ifdef DEBUG
        cout << ">ExtI2cModule.process)" << endl;
#endif // DEBUG
    // set the sampling interval
    std::chrono::microseconds period(SAMPLEINTERVALUS);
    auto next = std::chrono::high_resolution_clock::now() + period;

    // periodically read the converter and distribute the samples
    // to each channel. Stop when any channel reports buffer full.
    rdy = false;
    bcm2835_i2c_setSlaveAddress(i2cAddress);
    while ( !rdy ) {
        // read module datasource
        //result = bcm2835_i2c_read((char *)readBuffer, moduleReadLength);
        result = bcm2835_i2c_write_read_rs(&offs, 1, (char *)readBuffer, moduleReadLength);
        if ( result != BCM2835_I2C_REASON_OK ) {
        //TODO: handle the I2C bus failure!
#ifdef DEBUG
        cout << "I2C read failure!" << endl;
        exit(1);
#endif // DEBUG
        }
        // put data in channel buffers
        for ( it = channels.begin(); it != channels.end(); it++ ) {
            c = *it;
            rdy |= c->sample();
        }
        if ( rdy ) continue;
        // delay for next sample
        std::this_thread::sleep_until(next);
        next += period;
    }

    // when channel buffers full, process channels
    Module::process(resBufPtr);

#ifdef DEBUG
        cout << "<ExtI2cModule.process)" << endl;
#endif // DEBUG

}

void ExtI2cModule::print() {
    printf("VT05 Expansion I2C module:\n");
    printf("I2C address: %x\n", i2cAddress);
    Module::print();
}


ExtI2cChannel::ExtI2cChannel( uint8_t **buf, uint16_t len )   : AnalogChannel( buf, len) {
uint8_t fLen;
uint8_t fType;
uint8_t *b;
#ifdef DEBUG
cout << "> ExtI2cChannel" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
    // set default values
    // expander channels are 32 bit and only one sample
    dataBufferLen = sizeof(uint32_t);
    dataBuffer = new uint8_t[dataBufferLen];
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
cout << "< ExtI2cChannel" << endl;
#endif // DEBUG

}

ExtI2cChannel::~ExtI2cChannel() {}

uint8_t ExtI2cChannel::begin(uint8_t *sampleData){
#ifdef DEBUG
        cout << ">ExtI2cChannel.begin)" << endl;
#endif // DEBUG
    readBuffer = sampleData;
    dataBufferCnt = 0;
#ifdef DEBUG
        cout << "<ExtI2cChannel.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

bool ExtI2cChannel::sample(){
//uint16_t sourceWord,swappedWord;
    if( dataBufferCnt <= dataBufferLen-sizeof(uint32_t) ) {
        // no swap bytes; exp module is little endian
        // memcpy(&sourceWord, readBuffer, 2);
        // swappedWord = ((sourceWord & 0xff) << 8) | ((sourceWord & 0xff00) >> 8);
        // swappedWord &= 0x0fff;  // drop 4 MS bits
        // memcpy( (dataBuffer+dataBufferCnt) , &swappedWord, 2 );
        memcpy(dataBuffer+dataBufferCnt, readBuffer, sizeof(uint32_t));
        dataBufferCnt += sizeof(uint32_t);
        // if buffer is now full, data set is ready for processing
        if (dataBufferCnt >= dataBufferLen ) return true;
        else return false;
    }
    return true;
}

void ExtI2cChannel::process( char **resBufPtr, bool forceReporting ){
#ifdef DEBUG
        cout << ">ExtI2cChannel.process()" << endl;
#endif // DEBUG
    Channel::process( resBufPtr, forceReporting );
    // After processsing the channel we restart the sampling
    // cycle by resetting buffer sample count to zero
    dataBufferCnt =0;
#ifdef DEBUG
        cout << "<ExtI2cChannel.process()" << endl;
#endif // DEBUG
}

void ExtI2cChannel::print(){ Channel::print(); }

