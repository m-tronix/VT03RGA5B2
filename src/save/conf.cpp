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
*/

#include <iostream>
#include <cstring>
#include <thread>
#include <algorithm>
#include <cmath>

#include "conf.h"
#include "bcm2835.h"

//#define PRBUF // define for printing out the EEPROM read buffer contents

using namespace std;
using namespace std::this_thread;     // sleep_for, sleep_until
using namespace adxl372;


// auxiliary function for debugging. Prints a buffer broken into fields
// buffer must start on a valid field definition
void printBuffer(uint8_t * buf, uint16_t len) {
#ifdef PRBUF
uint8_t *rbuf, fLen, fType, j;
    rbuf = buf;
    while (rbuf < buf+len) {
        fLen = *rbuf++;
        fType = *rbuf++;
        printf("Len: %2x, type: %2x | ", fLen, fType);
        for (j = 0; j < fLen-1; j++ ) printf("%2x ", *rbuf++);
        printf("\n");
    }
#endif
}

// ADXL372 related auxiliary stuff

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

void i2c_read_buf(char *buf, uint32_t len) {
    uint8_t retcode = bcm2835_i2c_read(buf, len);
    if (retcode != bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK) {
//        std::string reason_str = reasoncode_to_str((bcm2835I2CReasonCodes)retcode);
//        throw std::runtime_error("bcm2835_i2c_read failed: " +  reason_str);
    }
}

void i2c_write_buf(const char *buf, uint32_t len) {
    uint8_t retcode = bcm2835_i2c_write(buf, len);
    if (retcode != bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK) {
// TODO: hoitele jotenkin
//        std::string reason_str = reasoncode_to_str((bcm2835I2CReasonCodes)retcode);
//        throw std::runtime_error("bcm2835_i2c_write failed: " +  reason_str);
    }
}

void i2c_write_char(char c) {
    /* Helper method to write single bytes to i2c.
    */
    const char buffer[] = {c};
    i2c_write_buf(buffer, 1);
}

char i2c_read_char(void) {
    char c;
    i2c_read_buf(&c, 1);
    return c;
}

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


//-----------------------

uint8_t readFromConf(uint16_t addr, uint8_t *buf, uint16_t len) {
uint8_t result;
char addrBuf[2];
    addrBuf[0] = (uint8_t)(addr >> 8);
    addrBuf[1] = (uint8_t)(addr & 0xff);
    result = bcm2835_i2c_write(addrBuf, sizeof (uint16_t));
    if ( result != BCM2835_I2C_REASON_OK ) {
#ifdef DEBUG
        cout << "i2c write failed" << endl;
#endif // DEBUG
        return CONF_ERR_MEM;
    }

    result = bcm2835_i2c_read((char *)buf, len);
    if ( result != BCM2835_I2C_REASON_OK ) {
 #ifdef DEBUG
       cout << "i2c conf read 1 failed" << endl;
 #endif // DEBUG
        return CONF_ERR_MEM;
    }
    return CONF_OK;
}

//-----------------------------

Conf::Conf( uint8_t i2cAddr ) {
    if (!bcm2835_init()) {
        status = 1;
        return;
    }
    // I2C begin if specified
    if (!bcm2835_i2c_begin()) {
        status = 2;
        return;
    }
    bcm2835_i2c_setSlaveAddress(i2cAddr);
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
    status = 0;
}

Conf::~Conf() {
    for (it = modules.begin(); it!=modules.end(); it++) {
        delete *it;
    }
}

// configuration main parser
/*
 buildFromEEPROM creates the data structures for a new configuration
 The structure is as follows:
 conf
   module
     channel
       channelprocessor
 There may be multiple modules each with multiple channels.
 Multiple channelprocessors may be attached to each channel
*/
uint8_t Conf::buildFromEEPROM() {

uint8_t rawBuffer[4096];
uint8_t *b;
uint16_t addr, fLen;
uint8_t fType;
uint16_t readLen;
uint8_t modType;
uint32_t modLen;
Module *m;
    status = CONF_OK;
    // read the configuration block length from EEPROM address 0...5
    // @0: length (must be 0x05)
    // @1: type (must be 0x00)
    // @2..5 block length
    addr = CONFSTARTADDR;
    readFromConf( addr, rawBuffer, 6);
    if( rawBuffer[0] != 0x05 || rawBuffer[1] != 0x000) {
#ifdef DEBUG
        cerr << "got invalid field length or type!" << endl;
#endif // DEBUG
        status = CONF_ERR_SYNTAX;
        return status;
    }
    memcpy( &readLen, &rawBuffer[2], sizeof( uint32_t));
    // parse the rest of the configuration after reading it into the rawBuffer
    addr = CONFSTARTADDR + 6;
    readFromConf(addr, rawBuffer, readLen);
    b = rawBuffer;
    printBuffer( b, readLen );
    while (b < rawBuffer+readLen) {
        fLen = *b++;
        fType = *b++;
#ifdef DEBUG
        printf("Type: %2x, len %2x  ", fType, fLen);
#endif // DEBUG

        switch ( fType ) {
            case VT_IOB_MODEL: {
#ifdef DEBUG
cout << "> VT_IOB_MODEL" << endl;
#endif // DEBUG
                memcpy(&boardModel, b, sizeof( uint32_t));
                b += fLen-1;
                break;
            }
            case VT_IOB_MODULE_COUNT: {
#ifdef DEBUG
cout << ">VT_IOB_MODULE_COUNT" << endl;
#endif // DEBUG
                moduleCount = *b++;
                break;
            }
            case VT_IOB_BOARD_UUID: {
#ifdef DEBUG
cout << ">VT_IOB_BOARD_UUID" << endl;
#endif // DEBUG
                memcpy(boardUUID, b, sizeof( boardUUID ));
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_MOD_LEN: {  // create new module
#ifdef DEBUG
cout << ">VT_IOB_FLD_MOD_LEN" << endl;
#endif // DEBUG
                memcpy( &modLen, b, sizeof(uint32_t));
                b += sizeof( uint32_t);
                // get the module type from next field and create that type
                fLen = *b++;
                fType = *b++;
                modType = *b++;
                if ( fLen == 2 && fType == VT_IOB_FLD_MOD_TYPE ) {
                    // fix modLen by subtracting type field that we already read
                    modLen -= 3;
                    switch ( modType) {
                        case VT_IOB_MOD_ADC_I2C: {
                            m =  new AdcI2cModule( &b, modLen );
#ifdef DEBUG
printf("ADCI2C module %p\n" , m );
#endif // DEBUG
                            break;
                        }
                        case VT_IOB_MOD_DIG_IN: {
                            m =  new DigitalInModule( &b, modLen );
#ifdef DEBUG
printf("Digital module %p\n" , m );
#endif // DEBUG
                            break;
                        }
                        case VT_IOB_MOD_MOTION_I2C: {
                            m =  new ADXL372Module( &b, modLen );
#ifdef DEBUG
printf("I2C motion module %p\n" , m );
#endif // DEBUG
                            break;
                        }
                        case VT_IOB_MOD_TEMP_I2C: {
                            m =  new TempI2cModule( &b, modLen );
#ifdef DEBUG
printf("I2C motion module %p\n" , m );
#endif // DEBUG
                            break;
                        }
                    }
                    if ( m != nullptr ) {
#ifdef DEBUG
printf("Pushing module %p\n" , m );
#endif // DEBUG
                        modules.push_back( m );
                    }
                }
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
    return status;
}

// Initialization of the configuration after sucessful construction of
// the element hierarchy
uint8_t Conf::begin() {
Module *m;
#ifdef DEBUG
        cout << ">Conf.begin)" << endl;
#endif // DEBUG
    for ( it = modules.begin(); it != modules.end(); it++ ) {
        m = *it;
        m->begin();
    }
 #ifdef DEBUG
        cout << "<Conf.begin)" << endl;
#endif // DEBUG
   return CONF_OK;
}

void Conf::getUUID( char **ID ) {
uint8_t cnt,n;
    for ( cnt = 0; cnt < BOARD_UUID_LEN; cnt++ ) {
        n = sprintf(*ID, "%x", boardUUID[cnt] >> 4 & 0xf );
        *ID += n;
        n = sprintf(*ID,"%x", boardUUID[cnt] & 0xf);
        *ID += n;
        if ( cnt == 3 || cnt == 5 || cnt == 7 || cnt == 9 ) {
            n = sprintf(*ID, "-");
            *ID += n;
        }
    }
}

// Processing the configuration i.e. actual measurements
void Conf::process(char **resBufPtr) {
char *b, *tmpP, *tmp2P;
//uint64_t now;
std::chrono::steady_clock::time_point now;
Module *m;
#ifdef DEBUG
        cout << ">Conf.process()" << endl;
#endif // DEBUG
    b = *resBufPtr;
    *b++ = '[';         // opening bracket.
#ifdef DEBUG
        printf("Result buffer at: %p\n", resBufPtr );
#endif // DEBUG
    tmpP = b;
    tmp2P = b;
    for ( it = modules.begin(); it != modules.end(); it++ ) {
        m = *it;
        now = std::chrono::steady_clock::now();
        if ( now > (m->getLastProcessed() + interval_ms{m->getModuleMinSamplingInterval()})) {   // minimum processing interval elapsed?
            m->setLastProcessed(now);
//        now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
//        if ( now > m->getNextProcessTimeStamp() ) {
//            m->setNextProcessTimeStamp(now + m->getModuleMinSamplingInterval());
            m->process( &b );
            if ( b > tmp2P ) {  // if module inserted a json block then add separator
                *b++ = ',';
                tmp2P = b;
            }
        }
    }
    if ( *(b-1) == ',' ) b--;  // remove last (extra) ','
    if ( b > tmpP ) {
        *b++ = ']';         // closing bracket and terminal 0.
        *b++ = 0;
    }
    else {
        **resBufPtr = 0;    // .. or empty message (not sent)
    }

#ifdef DEBUG
        printf("Result buffer at: %p\n", resBufPtr );
        cout << "<Conf.process()" << endl;
#endif // DEBUG
}

// Configuration printout for testing purposes. Absolutely do not use
// in production!
void Conf::print() {
uint8_t cnt;
Module *m;
    printf("\n\n== Configuration begins: ==\n\n");
    printf("Conf status: %d\n", status);
    printf("BoardModel: %x\n", boardModel);
    printf( "UUID: ");
    for ( cnt = 0; cnt < BOARD_UUID_LEN; cnt++ ) {
        printf("%x", boardUUID[cnt] >> 4 & 0xf );
        printf("%x", boardUUID[cnt] & 0xf);
        if ( cnt == 3 || cnt == 5 || cnt == 7 || cnt == 9 ) printf("-");
    }
    printf("\n");
    cnt = 0;
    for ( it = modules.begin(); it != modules.end(); it++ ) {
        printf("\nModule %d:\n", cnt);
        m = *it;
        m->print();
        cnt++;
    }
    printf("\n==Configuration ends.==\n\n");
}

//-----------------------------
// Module level

Module::Module( uint8_t **buf, uint16_t len ) {
}

Module::~Module() {

}

uint8_t Module::begin() {
#ifdef DEBUG
        cout << "<Module.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
#ifdef DEBUG
        cout << "<Module.begin)" << endl;
#endif // DEBUG
}

// Virtual parent processor method for the module base class
// A specific child module will first run its own processor to prepare the module and then
// call this base method to iterate the channels.
void Module::process( char **resBufPtr) {
Channel *c;
int16_t n;
bool forceReporting;
char *tmpP, *tmpP2;
std::chrono::steady_clock::time_point now;

#ifdef DEBUG
        cout << ">Module.process()" << endl;
        printf("Result buffer at: %p\n", resBufPtr );
#endif // DEBUG
    tmpP = *resBufPtr;
    using namespace std::chrono;
    milliseconds ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch());
    // print timestamp for the results of this module
    n = sprintf(*resBufPtr, "{\"time\": %lli, ", ms);
    *resBufPtr += n;   // ignore terminating 0

    forceReporting = false;
    now = std::chrono::steady_clock::now();
    if ( now > (lastReported + interval_ms{moduleMaxSamplingInterval}) ) {   // minimum processing interval elapsed?
        forceReporting = true;
    }

    tmpP2 = *resBufPtr;
    for ( it = channels.begin(); it != channels.end(); it++ ) {
        c = *it;
        c->process( resBufPtr, forceReporting );
    }
    if ( *resBufPtr > tmpP2 ) {  // remove the last (xtra) ", " if there
        *resBufPtr -= 2;
        **resBufPtr =0;
        n = sprintf(*resBufPtr, "}");
        *resBufPtr += n;   // ignore terminating 0
        lastReported =now;
    }
    else {                  // no text; remove entire header
        *resBufPtr = tmpP;
        **resBufPtr = 0;
    }
#ifdef DEBUG
        printf("Result buffer at: %p\n", resBufPtr );
        cout << "<Module.process()" << endl;
#endif // DEBUG
}

void Module::print() {
uint8_t cnt;
Channel *c;
    printf("\nModule status: %d\n", status);
    printf("Module component: %s\n", moduleCompName);
    // module value type
    printf( "Module value type: ");
    switch (moduleValueType) {
        case VT_IOB_VALUE_UNDEFINED: printf("undefined"); break;
        case VT_IOB_VALUE_UINT8_T: printf("UINT8_T"); break;
        case VT_IOB_VALUE_INT8_T: printf("INT8_T"); break;
        case VT_IOB_VALUE_CHAR: printf("CHAR"); break;
        case VT_IOB_VALUE_UINT16_T: printf("UINT16_T"); break;
        case VT_IOB_VALUE_INT16_T: printf("INT16_T"); break;
        case VT_IOB_VALUE_UINT32_T: printf("UINT32_T"); break;
        case VT_IOB_VALUE_INT32_T: printf("INT32_T"); break;
        case VT_IOB_VALUE_FLOAT: printf("FLOAT"); break;
        case VT_IOB_VALUE_DOUBLE: printf("DOUBLE"); break;
   }
    printf("\n");
    printf("Module is %s endian\n", moduleEndian ? "big" : "little" );
    printf("ADC bits: %d \n", moduleAdcBits);
    printf("ADC shift: %d \n", moduleAdcShift);
    printf("ADC bitmask: %x \n", moduleAdcBitMask);
    printf("Module init cmd: ");
    for ( cnt = 0; cnt < moduleInitCmdLen; cnt++) {
        printf("%x",moduleInitCmd[cnt] >> 4 & 0x0f);
        printf("%x ", moduleInitCmd[cnt] & 0x0f);
    }
    printf("\n");
    printf("Module stop cmd: ");
    for ( cnt = 0; cnt < moduleStopCmdLen; cnt++) {
        printf("%x",moduleStopCmd[cnt] >> 4 & 0x0f);
        printf("%x ", moduleStopCmd[cnt] & 0x0f);
    }
    printf("\n");
    printf("Module read length: %d\n", moduleReadLength);
    cnt = 0;
    printf("Channels: %d\n",  channelCount );
    for ( it = channels.begin(); it != channels.end(); it++ ) {
        printf("\nChannel %d:\n", cnt);
        c = *it;
        c->print();
        cnt++;
    }

}


AdcI2cModule::AdcI2cModule( uint8_t **buf, uint16_t len ) : Module( buf, len) {
uint8_t *b;
uint8_t fLen, fType, byteValue;
uint32_t bLen;
Channel *c;

#ifdef DEBUG
cout << "> AdcI2cModule" << endl;
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
                c = new AdcI2cChannel( &b, bLen );
                if ( c != nullptr ) {
                    // 4-20mA channel gets average value processor, CT's get RMS
                    if ( c->getIndex() == 2 ) c->addChannelProcessor( new AVGChannelProcessor( c ) );
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
                memcpy( &moduleMinSamplingInterval, b, sizeof( uint32_t ));
                b += fLen-1;
                break;
            }
            case VT_IOB_FLD_MOD_MAX_INTERVAL_MS: {    // uint32_t Maximum valid sampling interval (ms)
#ifdef DEBUG
cout << "> VT_IOB_FLD_MOD_MAX_INTERVAL_MS" << endl;
#endif // DEBUG
                memcpy( &moduleMaxSamplingInterval, b, sizeof( uint32_t ));
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

AdcI2cModule::~AdcI2cModule() {
}

uint8_t AdcI2cModule::begin() {
uint8_t result;
uint8_t bufIncrement;
uint8_t cnt;
Channel *c;
#ifdef DEBUG
        cout << ">AdcI2cModule.begin)" << endl;
#endif // DEBUG
    // initialize the A/D converter
    bcm2835_i2c_setSlaveAddress(ADCADDR);
    result = bcm2835_i2c_write((char *)moduleInitCmd, moduleInitCmdLen);
    if ( result != BCM2835_I2C_REASON_OK ) {
        status = CONF_ERR_I2C;
        return status;
    }
    //Allocate a buffer to hold the data read from the converter
//    readBuffer = (uint8_t*) malloc (moduleReadLength);
    bufIncrement = moduleReadLength / channelCount;
    //Each channel gets the address of the data element corresponding to that channel
    // Init the channel
    cnt = 0;
     for ( it = channels.begin(); it != channels.end(); it++ ) {
        c = *it;
        c->begin( &readBuffer[cnt*2] ); //TODO: korjaa vähemmän hirveäksi
        cnt++;
    }
#ifdef DEBUG
        cout << "<AdcI2cModule.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

// Module processor periodically reads the A/D converter and distributes the samples
// to the individual channels ( the sample() function ).
// The first sample() that returns true triggers channel processing where
// each channel in turn is processed.
void AdcI2cModule::process( char **resBufPtr) {
Channel *c;
uint8_t result;
bool rdy;
#ifdef DEBUG
        cout << ">AdcI2cModule.process)" << endl;
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
        result = bcm2835_i2c_read((char *)readBuffer, moduleReadLength);
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
        cout << "<AdcI2cModule.process)" << endl;
#endif // DEBUG

}

void AdcI2cModule::print() {
    printf("Analog I2C module:\n");
    printf("I2C address: %x\n", i2cAddress);
    Module::print();
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

TempI2cModule::TempI2cModule( uint8_t **buf, uint16_t len ) : Module( buf, len) {
uint8_t *b;
uint8_t fLen, fType, byteValue;
uint32_t bLen;
Channel *c;

#ifdef DEBUG
cout << "> TempI2cModule" << endl;
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
                c = new TempI2cChannel( &b, bLen );
                if ( c != nullptr ) {
                    // 4-20mA channel gets average value processor, CT's get RMS
                    c->addChannelProcessor( new AVGChannelProcessor( c ) );
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
cout << "< TempI2cModule" << endl;
#endif
}

TempI2cModule::~TempI2cModule() {
}

uint8_t TempI2cModule::begin() {
uint8_t result;
uint8_t bufIncrement;
uint8_t cnt;
Channel *c;
#ifdef DEBUG
        cout << ">TempI2cModule.begin)" << endl;
#endif // DEBUG
    // initialize the A/D converter
    bcm2835_i2c_setSlaveAddress(i2cAddress);
    result = bcm2835_i2c_write((char *)moduleInitCmd, moduleInitCmdLen);
    if ( result != BCM2835_I2C_REASON_OK ) {
        status = CONF_ERR_I2C;
        return status;
    }
    //Allocate a buffer to hold the data read from the converter
//    readBuffer = (uint8_t*) malloc (moduleReadLength);
    bufIncrement = moduleReadLength / channelCount;
    //Each channel gets the address of the data element corresponding to that channel
    // Init the channel
    cnt = 0;
     for ( it = channels.begin(); it != channels.end(); it++ ) {
        c = *it;
        c->begin( &readBuffer[cnt*2] ); //TODO: korjaa vähemmän hirveäksi
        cnt++;
    }
#ifdef DEBUG
        cout << "<TempI2cModule.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

void TempI2cModule::process( char **resBufPtr) {
Channel *c;
uint8_t result;
bool rdy;
char reg = 0x00;
#ifdef DEBUG
        cout << ">TempI2cModule.process)" << endl;
#endif // DEBUG
    bcm2835_i2c_setSlaveAddress(i2cAddress);
    // set register to read (temp)
    result = bcm2835_i2c_write(&reg, 1);
    if ( result != BCM2835_I2C_REASON_OK ) {
        return;
    }
    // read module datasource
    result = bcm2835_i2c_read((char *)readBuffer, moduleReadLength);
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

    // when channel buffers full, process channels
    Module::process(resBufPtr);

#ifdef DEBUG
        cout << "<TempI2cModule.process)" << endl;
#endif // DEBUG

}

void TempI2cModule::print() {
    printf("Temp I2C module:\n");
    printf("I2C address: %x\n", i2cAddress);
    Module::print();
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

    set_measurement_settings(adxl372::filter_bandwidth::BW_400, true);
    set_timing(adxl372::output_data_rate::HZ_1600);
    set_fifo(FIFO_SIZE);

    set_power_mode(adxl372::power_mode::MEASUREMENT, false);

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

//-----------------------------

Channel::Channel( uint8_t **buf, uint16_t len ) {
#ifdef DEBUG
cout << "> Channel" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
#ifdef DEBUG
cout << "< Channel" << endl;
#endif // DEBUG
}

Channel::~Channel() {}
/*
uint8_t Channel::begin(uint8_t *sampleData) {
    return CONF_OK;
}
*/

void Channel::process( char **resBufPtr, bool forceReporting) {
ChannelProcessor *cP;
int16_t n;
char *tmpP;
#ifdef DEBUG
        cout << ">Channel.process()" << endl;
#endif // DEBUG
    tmpP = *resBufPtr;
    for ( it = processors.begin(); it != processors.end(); it++ ) {
        cP = *it;
        cP->process( dataBuffer, dataBufferLen, resBufPtr, forceReporting );
        if ( *resBufPtr > tmpP ) {
            n = sprintf(*resBufPtr, ", ");
            *resBufPtr += n;
            tmpP = *resBufPtr;
        }
    }
#ifdef DEBUG
        cout << "<Channel.process()" << endl;
#endif // DEBUG

}

void Channel::print() {
    printf("Channel status: %d\n", status);
//    printf("I/O pin: %d\n", ioPin);
    printf("Physical quantity: %s\n", channelPhysicalQuantity);
/*    printf(" Channel scaling factors: %f, %f, %f, %f\n",
        channelScalingFactor[0],
        channelScalingFactor[1],
        channelScalingFactor[2],
        channelScalingFactor[3]);
    printf("Minimum valid meas: %d\n", channelMinValid);
    printf("Maximum valid meas: %d\n", channelMaxValid);
*/
    printf("Minimum sampling freq: %d\n", channelMinSamplingInterval);
    printf("Maximum sampling freq: %d\n", channelMaxSamplingInterval);
}

AnalogChannel::AnalogChannel( uint8_t **buf, uint16_t len )  : Channel( buf, len){
#ifdef DEBUG
cout << "> AnalogChannel" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
#ifdef DEBUG
cout << "< AnalogChannel" << endl;
#endif // DEBUG
}

AnalogChannel::~AnalogChannel() {}

/*
uint8_t AnalogChannel::begin(uint8_t *sampleData) {
    return CONF_OK;
}
void AnalogChannel::sample() {}
void AnalogChannel::process() {}
void AnalogChannel::print() {}
*/

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

// Construct an AdcI2c type channel from the buffered EEPROM data
/*
 The constructor parses the buffer into fields according to the specification in
 VT documentation.
 By default the constructor attaches an RMS value processor to the channel. This
 may change in later specifications if the description is expanded to include specific
 processors in the configuration data.
*/
AdcI2cChannel::AdcI2cChannel( uint8_t **buf, uint16_t len )  : AnalogChannel( buf, len){
uint8_t fLen;
uint8_t fType;
uint8_t *b;
#ifdef DEBUG
cout << "> AdcI2cChannel" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
    // set default values
    dataBufferLen = NUMMEASCOUNT * sizeof(uint16_t); // TODO: sample datatype size should be read from conf!
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
cout << "< AdcI2cChannel" << endl;
#endif // DEBUG
}

AdcI2cChannel::~AdcI2cChannel() {}

uint8_t AdcI2cChannel::begin(uint8_t *sampleData) {
#ifdef DEBUG
        cout << ">AdcI2cChannel.begin)" << endl;
#endif // DEBUG
    readBuffer = sampleData;
    dataBufferCnt = 0;
#ifdef DEBUG
        cout << "<AdcI2cChannel.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

// Add a new sample to the channel data buffer.
// the read buffer address has been set in the begin() method
// TODO: properly handle the data element size. Now assumed to be 2
// i.e. an uint16_t
bool AdcI2cChannel::sample() {
uint16_t sourceWord,swappedWord;
    if( dataBufferCnt <= dataBufferLen-2 ) {
        // swap bytes; ADC is big endian
        memcpy(&sourceWord, readBuffer, 2);
        swappedWord = ((sourceWord & 0xff) << 8) | ((sourceWord & 0xff00) >> 8);
        swappedWord &= 0x0fff;  // drop 4 MS bits
        memcpy( (dataBuffer+dataBufferCnt) , &swappedWord, 2 );
        dataBufferCnt += 2;
        // if buffer is now full, data set is ready for processing
        if (dataBufferCnt <= dataBufferLen-2 ) return false;
        else return true;
    }
    return true;

}

void AdcI2cChannel::process( char **resBufPtr, bool forceReporting) {
#ifdef DEBUG
        cout << ">AdcI2cChannel.process()" << endl;
#endif // DEBUG
    Channel::process( resBufPtr, forceReporting );
    // After processsing the channel we restart the sampling
    // cycle by resetting buffer sample count to zero
    dataBufferCnt =0;
#ifdef DEBUG
        cout << "<AdcI2cChannel.process()" << endl;
#endif // DEBUG
}

void AdcI2cChannel::print() { Channel::print(); }

TempI2cChannel::TempI2cChannel( uint8_t **buf, uint16_t len )  : AnalogChannel( buf, len){
uint8_t fLen;
uint8_t fType;
uint8_t *b;
#ifdef DEBUG
cout << "> TempI2cChannel" << endl;
    printBuffer( *buf, len );
#endif // DEBUG
    // set default values (for temp we process 1 sample at a time, so 2 byte buffer
    dataBufferLen = sizeof(uint16_t); // TODO: sample datatype size should be read from conf!
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
cout << "< TempI2cChannel" << endl;
#endif // DEBUG
}

TempI2cChannel::~TempI2cChannel() {}

uint8_t TempI2cChannel::begin(uint8_t *sampleData) {
#ifdef DEBUG
        cout << ">TempI2cChannel.begin)" << endl;
#endif // DEBUG
    readBuffer = sampleData;
    dataBufferCnt = 0;
#ifdef DEBUG
        cout << "<TempI2cChannel.begin)" << endl;
#endif // DEBUG
    return CONF_OK;
}

bool TempI2cChannel::sample() {
uint16_t sourceWord,swappedWord;
    if( dataBufferCnt <= dataBufferLen-2 ) {
        // swap bytes; ADC is big endian
        memcpy(&sourceWord, readBuffer, 2);
        swappedWord = ((sourceWord & 0xff) << 8) | ((sourceWord & 0xff00) >> 8);
        swappedWord = (swappedWord >> 4) & 0x0fff;  // drop 4 MS bits
        memcpy( (dataBuffer+dataBufferCnt) , &swappedWord, 2 );
        dataBufferCnt += 2;
        // if buffer is now full, data set is ready for processing
        if (dataBufferCnt <= dataBufferLen-2 ) return false;
        else return true;
    }
    return true;

}

void TempI2cChannel::process( char **resBufPtr, bool forceReporting) {
#ifdef DEBUG
        cout << ">TempI2cChannel.process()" << endl;
#endif // DEBUG
    Channel::process( resBufPtr, forceReporting );
    // After processsing the channel we restart the sampling
    // cycle by resetting buffer sample count to zero
    dataBufferCnt =0;
#ifdef DEBUG
        cout << "<TempI2cChannel.process()" << endl;
#endif // DEBUG
}

void TempI2cChannel::print() { Channel::print(); }

// multiaxis motion sensor
MotionI2cChannel::MotionI2cChannel( uint8_t **buf, uint16_t len )  : AnalogChannel( buf, len){
uint8_t fLen;
uint8_t fType;
uint8_t *b;
uint16_t samples;
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
                set_offsets(adxl372::Vec3i16(tmp[0], tmp[1], tmp[2]));
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

bool MotionI2cChannel::sample() {
    // If the fifo has samples available, perform a read.
    struct adxl372::adxl372_status status = read_status();
    if (status.samples) {
        std::vector<Vec3i16> raw_xyz;
        raw_xyz.reserve(status.samples);
        read_fifo_xyz(raw_xyz, status.samples);


        for (const Vec3i16 &sample : raw_xyz) {
            float magnitude = sample.norm();
            acceleration_magnitudes.push_back((int16_t)magnitude);
        }

        printf("num saples in buffer %u\n", acceleration_magnitudes.size());

        if (acceleration_magnitudes.size() >= MAX_FFT_SIZE) {
            // truncate to 4096, perform spectrum analysis and clear buffer
            if (spectrum_analyzer.process(acceleration_magnitudes.data(), MAX_FFT_SIZE)) {
                printf("spectrum_analyzer.process ok\n");
                char str[256] = "";
                if (spectrum_analyzer.getPeaks("", str, sizeof(str))) {
                    // ok
                    printf("Peaks %s\n", str);
                } else {
                    printf("no peaks\n");
                }
            } else {
                printf("spectrum_analyzer.process failed\n");
            }
            acceleration_magnitudes.clear();
        }

        /*
        unsigned int num_filtered = update(status.samples);
        if (num_filtered > 0) return true;
        else return false;
        */
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

//---------------------------
// Channel processors

void ChannelProcessor::process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting ) {
#ifdef DEBUG
        cout << "><ChannelProcessor.process()" << endl;
#endif // DEBUG
}

void RMSChannelProcessor::process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting ) {
uint16_t val, minVal, maxVal, cnt;
float valPp, pwrRMSValue, tmpRMSValue, newRMSValue;
int16_t n;
#ifdef DEBUG
        cout << ">RMSChannelProcessor.process()" << endl;
#endif // DEBUG
    // Trivial RMS value calculation: locate peak values and divide by sqrt(2)
    // Scan sample buffer and find min and max values
    minVal = 0xffff;
    maxVal = 0;
    for ( cnt = 0; cnt < len; cnt +=2 ) {
        val = *(uint16_t *)(buffer+cnt);
        if ( val < minVal ) minVal = val;
        if ( val > maxVal ) maxVal = val;
    }
    // Calcualte peak-to-peak value and from that, RMS
    valPp = maxVal - minVal;
    tmpRMSValue = valPp / 1.41421356237;
    pwrRMSValue = tmpRMSValue;
    newRMSValue = parent->channelScalingFactor[3] + pwrRMSValue * parent->channelScalingFactor[2];
    pwrRMSValue *= tmpRMSValue;
    newRMSValue += pwrRMSValue * parent->channelScalingFactor[1];
    pwrRMSValue *= tmpRMSValue;
    newRMSValue += pwrRMSValue * parent->channelScalingFactor[0];
//    printf("min: %4d max: %4d rms: %.2f\n", minVal, maxVal, newRMSValue);
    // If changed more than thresholdRel, report. But not if abs value is less than thresholdAbs
    if (newRMSValue > parent->thresholdAbs || currRMSValue > parent->thresholdAbs || forceReporting ) {
        if ( ( newRMSValue > currRMSValue * (1.0 + parent->thresholdRel ) ) ||
             ( currRMSValue > newRMSValue * (1.0 + parent->thresholdRel ) ) || forceReporting ) {
            currRMSValue = newRMSValue;
            n = sprintf(*resBufPtr, "\"%s\": %f", parent->channelPhysicalQuantity, newRMSValue);
            *resBufPtr += n;   // ignore terminating 0
        }
    }
#ifdef DEBUG
        printf("RMS: %f \n", newRMSValue);
        cout << "<RMSChannelProcessor.process()" << endl;
#endif // DEBUG

}

void AVGChannelProcessor::process( uint8_t *buffer, uint16_t len, char **resBufPtr, bool forceReporting ) {
uint16_t val, minVal, maxVal, cnt;
float valPp, pwrAVGValue, tmpAVGValue, newAVGValue;
int16_t n;
#ifdef DEBUG
        cout << ">AVGChannelProcessor.process()" << endl;
#endif // DEBUG
    // Trivial AVG value calculation: mean of samples
    // Scan sample buffer and find min and max values
    minVal = 0xffff;
    maxVal = 0;
    tmpAVGValue = 0;
    for ( cnt = 0; cnt < len; cnt +=2 ) {
        val = *(uint16_t *)(buffer+cnt);
        if ( val < minVal ) minVal = val;
        if ( val > maxVal ) maxVal = val;
        tmpAVGValue += val;
    }
    tmpAVGValue /= (len / 2);
    // Calcualte peak-to-peak value and from that, RMS
    pwrAVGValue = tmpAVGValue;
    newAVGValue = parent->channelScalingFactor[3] + pwrAVGValue * parent->channelScalingFactor[2];
    pwrAVGValue *= tmpAVGValue;
    newAVGValue += pwrAVGValue * parent->channelScalingFactor[1];
    pwrAVGValue *= tmpAVGValue;
    newAVGValue += pwrAVGValue * parent->channelScalingFactor[0];
    // If changed more than thresholdRel, report. But not if abs value is less than thresholdAbs
    if (newAVGValue > parent->thresholdAbs || currAVGValue > parent->thresholdAbs || forceReporting) {
        if ( ( newAVGValue > currAVGValue * (1.0 + parent->thresholdRel ) ) ||
             ( currAVGValue > newAVGValue * (1.0 + parent->thresholdRel ) ) || forceReporting) {
            currAVGValue = newAVGValue;
            n = sprintf(*resBufPtr, "\"%s\": %f", parent->channelPhysicalQuantity, newAVGValue);
            *resBufPtr += n;   // ignore terminating 0
        }
    }
#ifdef DEBUG
        printf("AVG: %f \n", newAVGValue);
        cout << "<AVGChannelProcessor.process()" << endl;
#endif // DEBUG

}


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
    if ( (newResult != currResult) || forceReporting ) {
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



