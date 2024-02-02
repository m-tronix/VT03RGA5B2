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

#include "vt3r0.h"
#include "module.h"
#include "adci2c.h"




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

#endif // CONF_H
