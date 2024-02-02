#ifndef MESSENGER_H
#define MESSENGER_H

#include <stdio.h>
#include <cstdlib>
#include <time.h>
#include <unistd.h>
extern "C" {
#include "mqtt.h"
#include "posix_sockets.h"
}
#include <pthread.h>
#include "vt3r0.h"
#include "bcm2835.h"

enum msgType { msgTypeNone, msgTypeDiagnostic, msgTypeMeasurement };

void publish_callback(void** unused, struct mqtt_response_publish *published);
void reconnect_client(struct mqtt_client* client, void **reconnect_state_vptr);
void* client_refresher(void* client);
void exit_service(int status, int sockfd, pthread_t *client_daemon);

struct reconnect_state_t {
    uint8_t txBuf[2048];
    uint8_t rxBuf[2048];

    const char *mqttIPAddr;
    const char *mqttIPPort;
    char mqttTopic[80];
    const char *mqttPass;
    const char *klDevType;
    const char *mqttUID;
    const char *klUUID;
    const char *klMsgType;
    char klTimeStamp[20];
    char klMessage[2048];
};

class messenger
{
public:
    messenger();
    messenger( const char *topic,  char *UUID  );
    virtual ~messenger();
    void begin( char *UUID);
    void send( const char *msg );
    MQTTErrors disconnect();
    bool reconnect();

protected:
    struct mqtt_client client;
	struct reconnect_state_t reconnect_state;
    pthread_t client_daemon;
	/*
    int msgSocket;
    uint8_t txBuf[2048];
    uint8_t rxBuf[2048];

    const char *mqttIPAddr;
    const char *mqttIPPort;
    char mqttTopic[80];
    const char *mqttPass;
    const char *klDevType;
    const char *mqttUID;
    const char *klUUID;
    const char *klMsgType;
    char klTimeStamp[20];
    char klMessage[2048];
	*/
};


#endif // MESSENGER_H
