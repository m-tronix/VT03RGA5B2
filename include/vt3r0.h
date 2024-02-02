/*
 * VT3R0 telemetry daemon
 * (c) 2018 Viimatech Digital Oy
 * Author: Martti Paalanen / M-Tronix t:mi
 */

#ifndef VT3R0_H
#define VT3R0_H

#include <stdint.h>

//#define DEBUG         // define for extended debug printouts
#define NET_DEBUG       // debug printouts for networking events
#define MSG_DEBUG       // debug printouts for data messages
#define SINGLE_PROCESS  // process only 1 module per iteration to minimize buffer length
//#define DIYMALL         // simplified GPRS module control

#define CONF_OK         0
#define CONF_ERR_VALUE  1
#define CONF_ERR_SYNTAX 2
#define CONF_ERR_STRUCT 3
#define CONF_ERR_MEM    4
#define CONF_ERR_I2C    5
#define CONF_WARN_UNK   100


void *mqtttask( void *param );
void *diagtask( void *param );

/* Physical I/O pins */
//Fona module connection pins on Raspi
#define FONA_RESET          05   // Raspi chip pin #
#define FONA_KEY            19
#define FONA_POWERSTATUS    13
#define FONA_NETWORKSTATUS  12

//Digital inputs on Raspi
#define D_IN1               22
#define D_IN2               23
#define D_IN3               24
#define D_IN4               25

#define LED_R               26
#define LED_G               20
#define LED_B               21

//MPa 2022-03-01
#define MAX_REGISTRATION_ATTEMPTS 20
//#define MAX_REGISTRATION_ATTEMPTS 300
#define RESTART_INTERVAL_MS (12*60*60*1000)
#define WATCHDOG_UPDATE_INTERVAL_MS 10000

/* MQTT message queue info */
#define MQTT_BROKER_IP "34.248.247.75"
#define MQTT_BROKER_PORT "1883"
#define MQTT_BROKER_UNAME "vt3_0000"
#define MQTT_BROKER_PW "CCuSi}OpzT1@X<xU"
#define DEV_TYPE "vt03"
//#define DEV_UUID "815e62e6-4e2a-4557-8a0d-acefe78af249"

/* Configuration EEPROM control */
#define CONFMEM_ADDR 0x50
#define EEPROMADDR          0x50        // I2C address of EEPROM
#define CONFSTARTADDR       256         // configuration start address in eeprom mem space

/* Motion sensor */
#define MOTIONADDR          0x53


/* Analog measurement (ADCI2C) control */
#define ADCADDR             0x34        // I2C address of ADC
#define ADC_NUM_DATA_CHANNELS 3        // number of 32-bit measurement values
#define NUMMEASCOUNT        100         // no of measurements in one cycle
#define SAMPLEINTERVALUS    1000        // interval between individual samples (microseconds)
#define CYCLEINTERVALMS     1000        // interval between measurement cycles (milliseconds)


/* Digital input measurement control */
#define D_IN1 22                        // Raspi chip pin numbers for digital I/O
#define D_IN2 23
#define D_IN3 24
#define D_IN4 25
#define D_TIMERIND 18
#define NUMMEASCOUNT_DIGITAL 50         // no of samples for a digital input per period

/* Expansion module measuremnent values */
#define EXPADDR             0x11        // I2C address of expansion module
#define EXP_NUM_DATA_CHANNELS 8        // number of 32-bit measurement values


/* Top Level Block descriptors */
#define VT_IOB_BOARD_LEN			0x00		// uint32_t Length of the data in memory in bytes
#define VT_IOB_MODEL				0x01		// uint32_t Model and revision of the expansion board
#define VT_IOB_BOARD_UUID			0x02		// uint8_t[16] IO Board serial number
#define VT_IOB_MODULE_COUNT			0x03		// uint8_t Number of modules in the IO Board
#define BOARD_UUID_LEN 				16

/* For ALL module type descriptors */
#define VT_IOB_FLD_MOD_LEN			0x10	// uint32_t Module block identifier, indicates the start of a module block and length of the block in bytes
#define VT_IOB_FLD_MOD_TYPE			0x11	// uint8_t Module type
#define VT_IOB_FLD_MOD_CH_COUNT		0x12	// uint8_t Number of channels in the module

/* For any ANALOG MODULE type */
#define VT_IOB_FLD_IO_PIN			0x13	// uint8_t Connector Pin connected to the analog Vout

/* For any DIGITAL MODULE type */
#define VT_IOB_FLD_DIG_COMPONENT	0x14	// char[254] Component name/identifier
#define VT_IOB_FLD_DIG_VALUE_TYPE	0x15	// uint8_t Data format of the digital output
#define VT_IOB_FLD_DIG_ENDIAN		0x16	// uint8_t Output data endianness

/* For any I2C MODULE type */
#define VT_IOB_FLD_I2C_ADDR			0x17	// uint8_t 7-bit I2C slave address
#define VT_IOB_FLD_I2C_INIT_CMD		0x18	// uint8_t[254] Command issued at power on to start measurements
#define VT_IOB_FLD_I2C_STOP_CMD		0x19	// uint8_t[254] Command issued to stop measurements
#define VT_IOB_FLD_I2C_READ_LEN		0x1a	// uint8_t Number of bytes requested when reading output
#define VT_IOB_FLD_I2C_READ_OFFS	0x25	// uint8_t Offset of bytes requested when reading output

/* For any SPI MODULE type */
#define VT_IOB_FLD_SPI_SS_PIN		0x1b	// uint8_t Connector pin connected to the slave select pin

/* For any ADC MODULE type */
#define VT_IOB_FLD_ADC_BITS			0x20	// uint8_t Number of bits in the ADC
#define VT_IOB_FLD_ADC_SHIFT		0x21	// int8_t Required bit shift for the value
#define VT_IOB_FLD_ADC_BITMASK		0x22	// uint32_t Bitmask for the output
#define VT_IOB_FLD_MOD_MIN_INTERVAL_MS 0x23	// uint32_t Minimum valid sampling interval (ms)
#define VT_IOB_FLD_MOD_MAX_INTERVAL_MS 0x24	// uint32_t Maximum valid sampling interval (ms)

/* For all channels */
#define VT_IOB_CH_BLOCK_LEN			0x80	// uint32_t Channel block identifier
											// indicates the start of a channel sub-block and length of the block in bytes
#define VT_IOB_FLD_CH_IDX			0x81	// uint8_t Channel index, starting from 0
#define VT_IOB_FLD_CH_PHY			0x82	// char[32] Physical quantity
#define VT_IOB_FLD_CH_SCAL			0x83	// float[4] Scaling parameters (RAW --> SI-unit)
#define VT_IOB_FLD_CH_MIN_VAL		0x84	// int32_t Minimum valid output (RAW / mV)
#define VT_IOB_FLD_CH_MAX_VAL		0x85	// int32_t Maximum valid output (RAW / mV)
#define VT_IOB_FLD_CH_ACC			0x88	// uint8_t Accuracy class
#define VT_IOB_FLD_CH_THRESHOLD_REL 0x89    // minimum change of signal to report (0.1 == 10%)
#define VT_IOB_FLD_CH_THRESHOLD_ABS 0x8a    // minimum change of signal to report (0.1 == 10%)
#define VT_IOB_FLD_CH_MOTION_OFFSETS 0x8b   // motion sensor static offset compensation


/* Values for VT_IOB_FLD_MOD_TYPE (0x011) */
#define VT_IOB_MOD_ANALOG			0x00	// Analog signal
#define VT_IOB_MOD_ADC_I2C			0x01	// External ADC in I2C bus
#define VT_IOB_MOD_ADC_SPI			0x02	// External ADC in SPI bus
#define VT_IOB_MOD_I2C_UDF			0x03	// Undefined I2C module
#define VT_IOB_MOD_SPI_UDF			0x04	// Undefined SPI module
#define VT_IOB_MOD_UART_UDF			0x05	// Undefined UART module
#define VT_IOB_MOD_DIG_IN           0x06    // Digital input module
#define VT_IOB_MOD_DIG_OUT          0x07    // Digital output module
#define VT_IOB_MOD_MOTION_I2C       0x08    // multiaxis motion sensor i2c
#define VT_IOB_MOD_TEMP_I2C         0x09    // Digital temperature sensor i2c
#define VT_IOB_MOD_VT03EXP          0x0a    // VT03 expansion module (I2C bus)

/* Values for VT_IOB_FLD_DIG_VALUE_TYPE (0x15) */
#define VT_IOB_VALUE_UNDEFINED		0x00
#define VT_IOB_VALUE_UINT8_T		0x01
#define VT_IOB_VALUE_INT8_T			0x02
#define VT_IOB_VALUE_CHAR			0x03
#define VT_IOB_VALUE_UINT16_T		0x04
#define VT_IOB_VALUE_INT16_T		0x05
#define VT_IOB_VALUE_UINT32_T		0x06
#define VT_IOB_VALUE_INT32_T		0x07
#define VT_IOB_VALUE_FLOAT			0x08
#define VT_IOB_VALUE_DOUBLE			0x09


/* Values for VT_IOB_FLD_DIG_ENDIAN (0x16) */
#define VT_IOB_ENDIAN_LE			0x00	// little endian
#define VT_IOB_ENDIAN_BE			0x01	// big endian

/* Values for VT_IOB_FLD_CH_ACC (0x88) */
#define VT_IOB_CH_ACC_UNDEF			0x00	// accuracy not defined

/*================================*/
/*  Diagnostic definitions.
    Diagnostic messages are sent from the field device to indicate
    exceptions, abnormal conditions or errors in processing or hardware.
    The diagnostic message contains the following parts (fields):
    1. severity ( 0 ... 4 )
    2. diagnostic classification code
    3. optional cleartext message for additional information where relevant.

    The exception severity (code) is classified as follows

    0:  informative - the diagnostic has no impact on the field device functionality
    1:  cosmetic - a formal exception with no or insignificant impact
    2:  minor - an exception impacting some nonessential functionality
    3:  major - an exception disabling a key feature or functionality
    4:  fatal - an exception that prevents the field device from functioning.
*/

/* Diagnostic codes */

/* Codes 0001 ... 1999: Field device functionality diagnostics */
#define DIAG_DEV_AUX_PWR    1001    // supply voltage problem
#define DIAG_DEV_HWERR      1002    // generic hardware element unresponsive
#define DIAG_DEV_I2C        1003    // I2C bus relatred error
#define DIAG_DEV_SPI        1004    // SPI bus related error
#define DIAG_DEV_ADC        1005    // directly connected ADC related error

/* codes 2000 ... 2999: Configuration issues */
#define DIAG_CONF_MEMFAULT  2001    // configuration memory unreadable
#define DIAG_CONF_NOCONF    2002    // no configuration found
#define DIAG_CONF_STRUCT    2003    // broken configuration structure
#define DIAG_CONF_NOHW      2004    // irrelevant/incorrect definition (no such hardware)
#define DIAG_CONF_ELEM      2005    // invalid conf element typecode (unknown item)
#define DIAG_CONF_VALUE     2006    // invalid conf element value (out of range)

/* codes 3000 ... 3999: Measurement and process related diagnostics */

/* codes 4000 ... 4999: Local analysis and computation related diagnostics */


/* ADXL372 motion sensor related definitions */

#define FIFO_SIZE 170

// Number of samples for the median filtering. Must be odd!
static const int MEDIAN_SAMPLES = 5;

enum register_map : char {
    DEVID_AD = 0x00,
    DEVID_MST = 0x01,
    PARTID = 0x02,
    REVID = 0x03,
    STATUS = 0x04,
    STATUS2 = 0x05,
    FIFO_ENTRIES2 = 0x06,
    FIFO_ENTRIES = 0x07,
    XDATA_H = 0x08,
    XDATA_L = 0x09,
    YDATA_H = 0x0A,
    YDATA_L = 0x0B,
    ZDATA_H = 0x0C,
    ZDATA_L = 0x0D,
    MAXPEAK_X_H = 0x15,
    MAXPEAK_Y_H = 0x17,
    MAXPEAK_Z_H = 0x19,
    MAXPEAK_Z_L = 0x1A,
    OFFSET_X = 0x20,
    OFFSET_Y = 0x21,
    OFFSET_Z = 0x22,
    THRESH_ACT_X_H = 0x23,
    THRESH_ACT_X_L = 0x24,
    THRESH_ACT_Y_H = 0x25,
    THRESH_ACT_Y_L = 0x26,
    THRESH_ACT_Z_H = 0x27,
    THRESH_ACT_Z_L = 0x28,
    TIME_ACT = 0x29,
    THRESH_INACT_X_H = 0x2A,
    THRESH_INACT_X_L = 0x2B,
    THRESH_INACT_Y_H = 0x2C,
    THRESH_INACT_Y_L = 0x2D,
    THRESH_INACT_Z_H = 0x2E,
    THRESH_INACT_Z_L = 0x2F,
    TIME_INACT_H = 0x30,
    TIME_INACT_L = 0x31,
    THRESH_ACT2_X_H = 0x32,
    THRESH_ACT2_X_L = 0x33,
    THRESH_ACT2_Y_H = 0x34,
    THRESH_ACT2_Y_L = 0x35,
    THRESH_ACT2_Z_H = 0x36,
    THRESH_ACT2_Z_L = 0x37,
    HPF = 0x38,
    FIFO_SAMPLES = 0x39,
    FIFO_CTL = 0x3A,
    INT1_MAP = 0x3B,
    INT2_MAP = 0x3C,
    TIMING = 0x3D,
    MEASURE = 0x3E,
    POWER_CTL = 0x3F,
    SELF_TEST = 0x40,
    RESET = 0x41,
    FIFO_DATA = 0x42
};

enum output_data_rate : char {
    HZ_400 = 0x00,
    HZ_800,
    HZ_1600,
    HZ_3200,
    HZ_6400
};

enum filter_bandwidth : char {
    BW_200 = 0x00,
    BW_400,
    BW_800,
    BW_1600,
    BW_3200,
};

enum power_mode : char {
    STANDBY = 0x0,
    WAKE_UP,
    INSTANT_ON,
    MEASUREMENT,
};

/* Status register bit masks */
enum status : uint8_t {
    DATA_RDY = 0x01,
    FIFO_RDY = 0x02,
    FIFO_FULL = 0x04,
    FIFO_OVR = 0x08, // fifo overflow
    USER_NVM_BUSY = 0x20,
    AWAKE = 0x40,
    ERR_USER_REGS = 0x80
};


#endif
