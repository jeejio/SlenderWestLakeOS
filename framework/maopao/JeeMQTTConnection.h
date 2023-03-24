#ifndef _H_JEE_MQTT_CONNECTION_
#define _H_JEE_MQTT_CONNECTION_ (1)

#include <stdint.h>

#include "bsp_define.h"
#include "JeeMQTTTransaction.h"
#include "JeeMQTTMessage.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

// #include "MQTTPacket.h"
#include "paho_mqtt.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define PUBLISH_TOPIC "/iot_server/2"
#define SUBSCRIBE_TOPIC "/iot_client/2"

#define MQTT_SUBTOPIC "/mqtt/test"
#define MQTT_PUBTOPIC "/mqtt/test"
#define MQTT_WILLMSG  "\
{\
\"id\":\"666\",\
\"method\":\"onoffline\",\
\"type\":\"rpc\",\
\"jsonrpc\":\"1.2\",\
\"params\":{\
\"clientId\":2,\
\"status\":0\
}\
}"

    typedef struct jee_mqtt_server_info
    {
        char host[128];
        // uint16_t port;
        char userName[64];
        char password[1024];
        char clientId[64];
        char will_info[256];
        char subtopic[64];
        char pubtopic[64];
    } JeeMQTTServerInfo_t, *PJeeMQTTServerInfo_t;

    typedef struct jee_mqtt_connection_st
    {
        /* Public */
        // Match
        struct jee_mqtt_transaction_manager_st *transactionManager;
        // Send
        struct jee_mqtt_transaction_node_st *transactionHead;

        int8_t (*cOpen)(struct jee_mqtt_connection_st *thiz);
        void (*vClose)(struct jee_mqtt_connection_st *thiz);

        struct jee_mqtt_transaction_st *(*pxCreateTransaction)(struct jee_mqtt_connection_st *thiz,
                                                               struct jee_mqtt_message_st *request, struct jee_mqtt_message_st *response);

        /* Private */
        PJeeMQTTServerInfo_t __serverInfo;
        uint8_t *__sendBuffer;
        uint8_t *__readBuffer;
        JEE_MUTEX __transactionManagerMutex;
        // Network __mqttNetwork;
        MQTTClient __mqttClient;
        int32_t __event_fd;

        void (*vSendMessage)(struct jee_mqtt_connection_st *thiz,
                             struct jee_mqtt_transaction_st *transaction);
        void (*__vRecvMessage)(struct jee_mqtt_transaction_st *transaction);
    } JeeMQTTConnection_t, *PJeeMQTTConnection_t;

    extern PJeeMQTTConnection_t conn;

    extern PJeeMQTTConnection_t pxJeeMQTTConnection_Create(PJeeMQTTServerInfo_t info);

#define CONN_EVENT_INIT (0)
#define CONN_EVENT_KNOCK (1)
#define CONN_EVENT_QUIT (2)

    typedef int32_t (*lMethod_DevApiProcess_t)(const char *method, const char *prams, char *result, int32_t resultLen);
    extern int32_t lRegister_DevApiProcess(lMethod_DevApiProcess_t apiProcess);
    extern int32_t lMqtt_is_connected(PJeeMQTTConnection_t thiz);

    typedef void (*JeeMQTTconnect_callback_st)(void *);
    /*
    **Solution provider :liyongen
    **Code editor       :yulei
    */
    extern int lJeeMQTTConnect_Register(PJeeMQTTConnection_t conn, \
                        JeeMQTTconnect_callback_st connect, \
                        JeeMQTTconnect_callback_st online, \
                        JeeMQTTconnect_callback_st offline);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _H_JEE_MQTT_CONNECTION_
