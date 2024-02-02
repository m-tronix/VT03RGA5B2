/*
Viimatech VT03-RGA5B2 main
Author: Martti Paalanen
        23.8.2019

*/
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <cstring>
#include <array>
#include <csignal>
#include <cstdlib>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <systemd/sd-daemon.h>

#include "bcm2835.h"
#include "conf.h"
#include "messenger.h"


volatile std::sig_atomic_t signal_received = 0;
bool netDebug, msgDebug;

void signal_handler(int signal) {
    signal_received = signal;
}


void GPIO_init( void){
    bcm2835_gpio_fsel( FONA_RESET, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel( FONA_KEY, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel( FONA_POWERSTATUS, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel( FONA_NETWORKSTATUS, BCM2835_GPIO_FSEL_INPT);

    bcm2835_gpio_fsel( LED_R, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel( LED_G, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel( LED_B, BCM2835_GPIO_FSEL_OUTP);
}

Conf conf(0x50);        // configuration and measurement class instance
messenger *diag;        // MQTT publisher
messenger *values;

char msgBuf[2049];
char *message;

// check Fona network status
// 0 - powered off
// 1 - running but not registered to network
// 2 - registered
// 3 - GPRS data connection active
uint8_t fonaNetStatus() {
milliseconds falling, rising, low;
if (netDebug) {
        printf( "fonaNetStatus ");
}
    if ( bcm2835_gpio_lev( FONA_POWERSTATUS ) == false ) {
if (netDebug) {
        printf( "0\n");
}
        return 0;
    }
    while (bcm2835_gpio_lev( FONA_NETWORKSTATUS ) == false);
    while (bcm2835_gpio_lev( FONA_NETWORKSTATUS ) == true);
    falling = duration_cast< milliseconds >( steady_clock::now().time_since_epoch());
    while (bcm2835_gpio_lev( FONA_NETWORKSTATUS ) == false);
    rising = duration_cast< milliseconds >( steady_clock::now().time_since_epoch());
    low = rising - falling;
    if ( low < (milliseconds)400 ) {
if (netDebug) {
        printf( "3\n");
}
        return 3;  // GPRS connection active
    }
    if (low < (milliseconds)1500 ) {
if (netDebug) {
        printf( "1\n");
}
        return 1;  // running, not registered
    }
if (netDebug) {
        printf( "2\n");
}
    return 2;
}

void stopGPRSModem() {
if (netDebug) {
        printf( "Stopping Fona\n" );
}
    bcm2835_gpio_clr(LED_G);
    bcm2835_gpio_clr(LED_B);
    bcm2835_gpio_set(LED_R);


    if ( bcm2835_gpio_lev( FONA_POWERSTATUS ) == true ) {
        bcm2835_gpio_clr(FONA_RESET);
        bcm2835_delay(2000);
        bcm2835_gpio_set(FONA_RESET);
        bcm2835_delay(5000);
    }
    /*
    bcm2835_gpio_clr(FONA_RESET);
    bcm2835_delay(100);
    bcm2835_gpio_set(FONA_RESET);
    */
}

bool startGPRSModem() {
uint16_t registrationCounter;
    if (netDebug) {
            printf( "Starting Fona\n" );
    }
    bcm2835_gpio_set(FONA_RESET);
    while ( fonaNetStatus() < 2 ) {
        if (netDebug) {
                printf( "init Fona\n" );
        }
        if ( bcm2835_gpio_lev( FONA_POWERSTATUS ) == false ) {
            do {
                bcm2835_gpio_clr(FONA_RESET);
                bcm2835_delay(2000);
                bcm2835_gpio_set(FONA_RESET);
                bcm2835_delay(5000);
                if (netDebug) {
                            printf( "Fona KEY cycled\n" );
                }
            } while ( bcm2835_gpio_lev( FONA_POWERSTATUS ) == false );
        }
        if (netDebug) {
                printf( "Fona power ON\n" );
        }

    //bcm2835_gpio_clr(FONA_RESET);

        // Init Fona and verify power status ON
        // Verify network status ON
        if (netDebug) {
                    printf( "Waiting for network...\n" );
        }
        registrationCounter = 0;
        do {
            if (netDebug) {
                        printf( "." ); fflush( stdout);
            }
            registrationCounter++;
            sd_notify( 0, "WATCHDOG=1");
            if ( registrationCounter >= MAX_REGISTRATION_ATTEMPTS ) {
                if (netDebug) {
                            printf( "FONA_RESET CLR " ); fflush( stdout);
                }
                bcm2835_gpio_clr(FONA_RESET);
                bcm2835_delay(2000);
                bcm2835_gpio_set(FONA_RESET);
                if (netDebug) {
                            printf( "- FONA_RESET SET\n" );
                }
                registrationCounter = 0;
                if (netDebug) {
                                printf( "Reset Fona\n" );
                }
            }
                bcm2835_delay(500);
        } while ( fonaNetStatus() < 2 );
        if (netDebug) {
                printf( "\nFona network registration successful!\n" );
        }
    }

    return true;
}

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

void sigpipe_handler(int sig) {
    printf("SIGPIPE: Broken pipe, continuing...\n");
}

int main( int argc, char **argv ) {
std::string pppStatus;
char c[10];
char devUUID[40];
char *devUUIDPtr;
char startText[256];
char *wd_millisecondsStr;
uint32_t wd_milliseconds;
milliseconds ms;
milliseconds lastFonaRestart;
milliseconds lastWatchdogUpdate;

//    std::signal( SIGTERM, signal_handler) ;
//    std::signal( SIGHUP, signal_handler );
//    std::signal( SIGINT, signal_handler );
std::signal( SIGPIPE, sigpipe_handler );

/* Initialize device hardware */
    sd_notify( 0, "WATCHDOG=1");
    netDebug = true;
    msgDebug = false;
    if ( argc > 1 ) {
        if (strcmp(argv[1], "1") == 0 ) netDebug = true;
        else if (strcmp(argv[1], "2") == 0 ) {
            netDebug = true;
            msgDebug = true;
        }
    }

    if ( !bcm2835_init() ){
        printf( "BCM init failed!\n" );
    }

    GPIO_init();


    // TODO: Initialize GPIO pins (Fona ctrl, digital inputs)
    // TODO: Initialize I2C bus
    // TODO: Initialize A/D onverter

/* Check environment for watchdog timeout */
    wd_millisecondsStr = getenv("WATCHDOG_USEC");
    if ( wd_millisecondsStr != nullptr ) wd_milliseconds = std::stoi(wd_millisecondsStr) /1000;
    else wd_milliseconds = 0;
    if (netDebug) {
        printf("Watchdog delay %d ms\n", wd_milliseconds);
     }

    sd_notify( 0, "WATCHDOG=1");

/* Read device configuration */
    conf.buildFromEEPROM();
//    conf.print();
    sd_notify( 0, "WATCHDOG=1");
    conf.begin();
    devUUIDPtr = &devUUID[0];
    conf.getUUID( &devUUIDPtr );
    sd_notify( 0, "WATCHDOG=1");

/* Initiate 3G point to point connection using Adafruit Fona control script */

    stopGPRSModem();


    startGPRSModem();
    sd_notify( 0, "WATCHDOG=1");
    bcm2835_delay(5000);
    sd_notify( 0, "WATCHDOG=1");
    // --- lisätty 2019-08-30
    do {
        if (netDebug) {
            printf("pppd on\n");
        }
        system("sudo pon fona");
        for ( uint8_t waitCnt = 0; waitCnt < 5; waitCnt++ ) {
            sd_notify( 0, "WATCHDOG=1");
            bcm2835_delay(3000);
            pppStatus = exec("ip addr | grep ppp0 | wc -l");
            std::strcpy(c,pppStatus.c_str());
            if (netDebug) {
                printf("pppd status %c\n", c[0]);
            }
            if ( c[0] != '0' ) break;
        }
        if ( c[0] == '0' ) {
            sd_notify( 0, "WATCHDOG=1");
            system("sudo poff fona");
            bcm2835_delay(5000);
            sd_notify( 0, "WATCHDOG=1");
            if (netDebug) {
                printf("pppd off\n");
            }
            sd_notify( 0, "WATCHDOG=1");
            bcm2835_delay(5000);
       }
    } while ( c[0] == '0' );

    bcm2835_gpio_clr(LED_R);
    bcm2835_gpio_clr(LED_G);
    bcm2835_gpio_set(LED_B);
    // ---

    // Force NTP clock update
    /*
    system( "sudo timedatectl set-ntp false" );
    system( "sudo timedatectl set-ntp true" );
    */
    system( "sudo systemctl restart systemd-timesyncd.service" );

    // TODO: verify successful ppp connection is open
/*    milliseconds ms1 = duration_cast< milliseconds >( steady_clock::now().time_since_epoch());
printf("time first: %lli\n", ms1);
    system( "sudo sntp -S time.mikes.fi");
//    system( "timedatectl set-ntp true" );
    milliseconds ms2 = duration_cast< milliseconds >( steady_clock::now().time_since_epoch());
printf("time second: %lli\n", ms2);
*/
    bcm2835_delay(1000);
    sd_notify( 0, "WATCHDOG=1");

    diag = new messenger("diag", devUUID);


    bcm2835_delay(5000);
    sd_notify( 0, "WATCHDOG=1");

/* Send MQTT diagnostic message to indicate system boot-up */

    ms = duration_cast< milliseconds >( steady_clock::now().time_since_epoch());
    lastFonaRestart = ms;
    lastWatchdogUpdate = ms;
    sprintf(startText, "time %lli: Device %s, built %s at %s: System restart", ms, devUUID, __DATE__, __TIME__);
    if (netDebug) {
        printf("%s\n", startText);
     }
    diag->send( startText );
    sd_notify( 0, "WATCHDOG=1");

    bcm2835_delay(1000);
    values = new messenger("values", devUUID);
    bcm2835_delay(5000);


/* Measurement loop */
    for ( ;; ) {

        message = msgBuf;
        conf.process( &message );
        if (strlen(message) > 0 ) {
            if (msgDebug) {
                printf("%s\n", msgBuf);
            }
;
            bcm2835_gpio_set(LED_R);
            values->send( msgBuf );
        }
        bcm2835_delay(500);     // wait a while for the Fona input buffer to recover
        bcm2835_gpio_clr(LED_R);
        ms = duration_cast< milliseconds >( steady_clock::now().time_since_epoch());
        if ( ms > lastFonaRestart + ( milliseconds ) RESTART_INTERVAL_MS  ) {
            lastFonaRestart = ms;
            exit(EXIT_SUCCESS);
            /*
            sd_notify( 0, "WATCHDOG=1");
            system("sudo poff fona");
            sd_notify( 0, "WATCHDOG=1");

            bcm2835_delay(2000);
            stopGPRSModem();
            sd_notify( 0, "WATCHDOG=1");

            startGPRSModem();
            sd_notify( 0, "WATCHDOG=1");
            bcm2835_delay(1000);
            system("sudo pon fona");
            sd_notify( 0, "WATCHDOG=1");
            bcm2835_delay(1000);

            bcm2835_gpio_clr(LED_R);
            bcm2835_gpio_clr(LED_G);
            bcm2835_gpio_set(LED_B);
            */
        }
        // pat the watchdog
        if ( ms > lastWatchdogUpdate + ( milliseconds ) WATCHDOG_UPDATE_INTERVAL_MS ) {
            lastWatchdogUpdate = ms;
            sd_notify( 0, "WATCHDOG=1");
        }

    }
}



