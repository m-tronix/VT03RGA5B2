#include <iostream>
#include "module.h"
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


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
    /*
    minVal = 0xffff;
    maxVal = 0;
    for ( cnt = 0; cnt < len; cnt +=2 ) {
        val = *(uint16_t *)(buffer+cnt);
        if ( val < minVal ) minVal = val;
        if ( val > maxVal ) maxVal = val;
    }*/

    // Copy the buffer and sort as uint16_t
    uint16_t *buf_copy = (uint16_t*) malloc(len);
    memcpy(buf_copy, buffer, len);
    sort(buf_copy, buf_copy+len/2);

    // Pick the third-largest and third-smalles values to filter out anomalies
    minVal = buf_copy[2];
    maxVal = buf_copy[(len/2)-3];
    free(buf_copy);


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

template <class T>
void AVGChannelProcessor<T>::process( T *buffer, uint16_t len, char **resBufPtr, bool forceReporting ) {
T val, minVal, maxVal;
uint16_t cnt;
float pwrAVGValue, tmpAVGValue, newAVGValue;
int16_t n;
#ifdef DEBUG
        cout << ">AVGChannelProcessor.process()" << endl;
#endif // DEBUG
    if ( len > 1 ) {
        // Trivial AVG value calculation: mean of samples
        // Scan sample buffer and find min and max values
        minVal = 0xffff;
        maxVal = 0;
        tmpAVGValue = 0;
        for ( cnt = 0; cnt < len; cnt++ ) {
            val = *(T *)(buffer+cnt);
            if ( val < minVal ) minVal = val;
            if ( val > maxVal ) maxVal = val;
            tmpAVGValue += val;
        }
        tmpAVGValue /= (len / 2);
    }
    else {  // if only 1 sample, no calculations needed
        tmpAVGValue = *(T *)buffer;
    }
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
