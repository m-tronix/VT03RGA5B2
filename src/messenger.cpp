#include "messenger.h"

#define NET_DEBUG


messenger::messenger()
{
    reconnect_state.mqttIPAddr = MQTT_BROKER_IP;        //"34.248.247.75";
    reconnect_state.mqttIPPort = MQTT_BROKER_PORT;      //"1883";
    reconnect_state.mqttPass = MQTT_BROKER_PW;          //"CCuSi}OpzT1@X<xU";
    reconnect_state.mqttUID = MQTT_BROKER_UNAME;        //"vt3_0000";
    reconnect_state.klDevType = DEV_TYPE;               //"vt03";
// removed; we get this as the argument of begin method
//    klUUID = DEV_UUID;                  //"815e62e6-4e2a-4557-8a0d-acefe78af249";
    reconnect_state.klMsgType = "";

   mqtt_init_reconnect(&client, reconnect_client, &reconnect_state, publish_callback);

#ifdef NET_DEBUG
        printf("Starting client daemon\n");
#endif

    if(pthread_create(&client_daemon, NULL, client_refresher, &client)) {
#ifdef NET_DEBUG
        printf("Failed to start client daemon!\n");
#endif
        exit_service(1, -1, NULL);
    }
    else {
#ifdef NET_DEBUG
        printf("Started MQTT refresher daemon!\n");
#endif
    }
}

messenger::messenger( const char *topic,  char *UUID ) : messenger() {
    reconnect_state.klMsgType = topic;
    reconnect_state.klUUID = UUID;
}

messenger::~messenger()
{
    //dtor
}

void messenger::send( const char *msg ) {
    snprintf(reconnect_state.klMessage, sizeof( reconnect_state.klMessage ), "%s", msg);
    MQTTErrors err = mqtt_publish(&client, reconnect_state.mqttTopic, reconnect_state.klMessage, strlen( reconnect_state.klMessage ), MQTT_PUBLISH_QOS_1);
#ifdef NET_DEBUG
        if (err != MQTT_OK) {
            printf("Publish error: ");
            printf("\"%s\"\n", mqtt_error_str(err) );
            fflush(stdout);
        }

#endif
}



void publish_callback(void** unused, struct mqtt_response_publish *published) {
#ifdef NET_DEBUG
    printf("Publish Callback invoked.\n");
#endif
}

void exit_service(int status, int sockfd, pthread_t *client_daemon) {
    printf("Exit service!");
    if (sockfd != -1) close(sockfd);
    if (client_daemon != NULL) pthread_cancel(*client_daemon);
    exit(status);
}

void* client_refresher(void* client) {
    while(1) {
        MQTTErrors err = mqtt_sync((struct mqtt_client*) client);
#ifdef NET_DEBUG
        if (err != MQTT_OK) {
            printf("Client refesher error: ");
            printf("\"%s\"\n", mqtt_error_str(err) );
            fflush(stdout);
        }

#endif
        usleep(200000U);
    }
    return NULL;
}

void reconnect_client(struct mqtt_client* client, void **reconnect_state_vptr)
{
#ifdef NET_DEBUG
    printf("Reconnect_client called\n");
    fflush(stdout);
#endif // NET_DEBUG
	struct reconnect_state_t *reconnect_state = *((struct reconnect_state_t**) reconnect_state_vptr);

	/* Close the clients socket if this isn't the initial reconnect call */
	if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
		close(client->socketfd);
	}

	/* Perform error handling here. */
	if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
		printf("reconnect_client: called while client was in error state \"%s\"\n",
			mqtt_error_str(client->error)
		);
	}

	/* Open a new socket. */
	int sockfd = open_nb_socket(reconnect_state->mqttIPAddr, reconnect_state->mqttIPPort);
	if (sockfd == -1) {
		perror("Failed to open socket: ");
		exit_service(EXIT_FAILURE, sockfd, NULL);
	}

	/* Reinitialize the client. */
    mqtt_reinit(client, sockfd, reconnect_state->txBuf, sizeof(reconnect_state->txBuf), reconnect_state->rxBuf, sizeof(reconnect_state->rxBuf));

	/* Ensure we have a clean session */
	uint8_t connect_flags = MQTT_CONNECT_CLEAN_SESSION;

	/* Send connection request to the broker. */
    snprintf(reconnect_state->mqttTopic, sizeof( reconnect_state->mqttTopic ), "vt03/%s/%s", reconnect_state->klUUID, reconnect_state->klMsgType);

    if (mqtt_connect(client, reconnect_state->mqttTopic, NULL, NULL, 0, reconnect_state->mqttUID, reconnect_state->mqttPass, connect_flags, 120) == MQTT_OK) {

#ifdef NET_DEBUG
        printf ("MQTT connect OK\n");
#endif
            bcm2835_gpio_clr(LED_R);
            bcm2835_gpio_set(LED_G);
            bcm2835_gpio_clr(LED_B);
    }
    else {
#ifdef NET_DEBUG
        printf ("MQTT connect Failed!\n");
#endif
            bcm2835_gpio_set(LED_R);
            bcm2835_gpio_clr(LED_G);
            bcm2835_gpio_set(LED_B);
    }
}



MQTTErrors messenger::disconnect() {
MQTTErrors e;
#ifdef NET_DEBUG
        printf ("MQTT disconnect ");
#endif
    e = mqtt_disconnect( &client );
    close( client.socketfd );
#ifdef NET_DEBUG
        printf ("status %d\n", e );
#endif
    return e;
};

/***************** DEPRECIATED FUNCTIONS ***************************/

void messenger::begin( char * UUID ) {
    if ( UUID ) {
        reconnect_state.klUUID = UUID;
    }
    snprintf(reconnect_state.mqttTopic, sizeof( reconnect_state.mqttTopic ), "vt03/%s/%s", reconnect_state.klUUID, reconnect_state.klMsgType);
    if (mqtt_connect(&client, reconnect_state.mqttTopic, NULL, NULL, 0, reconnect_state.mqttUID, reconnect_state.mqttPass, 0, 90) == MQTT_OK) {
#ifdef NET_DEBUG
        printf ("MQTT connect OK\n");
#endif
    }
    else {
#ifdef NET_DEBUG
        printf ("MQTT connect Failed!\n");
#endif
    }
}



/*
bool messenger::reconnect() {
    msgSocket = open_nb_socket(reconnect_state.mqttIPAddr, reconnect_state.mqttIPPort);
    if ( msgSocket == -1 ) return false;
    mqtt_reinit(&client, msgSocket, reconnect_state.txBuf, sizeof(reconnect_state.txBuf), reconnect_state.rxBuf, sizeof(reconnect_state.rxBuf) );
    if (mqtt_connect(&client, reconnect_state.mqttTopic, NULL, NULL, 0, reconnect_state.mqttUID, reconnect_state.mqttPass, 0, 90) == MQTT_OK) {
#ifdef NET_DEBUG
        printf ("MQTT connect OK\n");
#endif
        return true;
    }
    else {
#ifdef NET_DEBUG
        printf ("MQTT connect Failed!\n");
#endif
        return false;
    }
}
*/
