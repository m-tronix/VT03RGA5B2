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
#include <time.h>
#include <chrono>

#include "bcm2835.h"
#include "conf.h"
#include "module.h"
#include "adci2c.h"
#include "adxl372.h"
#include "digitalin.h"
#include "exti2c.h"
#include "tempi2c.h"


#define DIGITAL_IN_TOGGLE_COUNT 20
#define MAGIC1 0xa5f0693c

using namespace std;
using namespace std::this_thread;     // sleep_for, sleep_until





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
int8_t writeToConf(uint16_t addr, uint8_t *buf, uint16_t len) {
uint8_t result;
char Buf[34];
    Buf[0] = (uint8_t)(addr >> 8);
    Buf[1] = (uint8_t)(addr & 0xff);
    memcpy( &Buf[2], buf, len );
    result = bcm2835_i2c_write(Buf, len + sizeof (uint16_t));
    if ( result != BCM2835_I2C_REASON_OK ) {
        cout << "i2c write failed" << endl;
        return CONF_ERR_MEM;
    }

/*
    result = bcm2835_i2c_write((char *)buf, len);
    if ( result != BCM2835_I2C_REASON_OK ) {
       cout << "i2c conf write 1 failed" << endl;
        return CONF_ERR_MEM;
    }
*/
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
uint32_t magic1;
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
                // old style UUID in configuration
                // if a new style exists, ignore this
                readFromConf( 0, (uint8_t *)&magic1, sizeof( uint32_t ) );
                if ( magic1 != MAGIC1 ) {
                    memcpy(boardUUID, b, sizeof( boardUUID ));
                    magic1 = MAGIC1; // copy to new location for next time
                    writeToConf(0, (uint8_t *)&magic1, sizeof( uint32_t) );
                    bcm2835_delay(100);
                    writeToConf(16, (uint8_t *)&boardUUID, sizeof( boardUUID ) );
                }
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
                        case VT_IOB_MOD_VT03EXP: {
                            m =  new ExtI2cModule( &b, modLen );
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

    // check new style UUID and use if it is there
    readFromConf( 0, (uint8_t *)&magic1, sizeof( uint32_t ) );
    if ( magic1 == MAGIC1 ) {
        readFromConf(16, (uint8_t *)&boardUUID, sizeof( boardUUID) );
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
#ifdef SINGLE_PROCESS
bool processed;
#endif
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
#ifdef SINGLE_PROCESS
    processed = false;
#endif
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
#ifdef SINGLE_PROCESS
                processed = true;
#endif
            }
        }
#ifdef SINGLE_PROCESS
        if ( processed ) break;
#endif
    }
    if ( *(b-1) == ',' ) b--;  // remove last (extra) ','
    if ( b > tmpP ) {
        *b++ = ']';             // closing bracket and terminal 0.
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






