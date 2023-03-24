#ifndef _H_JEE_MQTT_TRANSACTION_
#define _H_JEE_MQTT_TRANSACTION_ (1)

#include "JeeMQTTMessage.h"
#include "JeeMQTTConnection.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//限制时间内未收到有效的respond，清理request环境，限制时间单位秒
#define JEE_TRANSACTION_MANAGER_TIMEOUT_S 5
typedef struct jee_mqtt_transaction_st
{
    struct jee_mqtt_connection_st *connection;
    struct jee_mqtt_message_st *request;
    struct jee_mqtt_message_st *response;
    void *originData;
    uint64_t timeout;   //发送request后，超过规定时间未收到有效response，则销毁无意义的req环境

    void (*vSendRequest)(struct jee_mqtt_transaction_st *transaction);
    void (*vSendResponse)(struct jee_mqtt_transaction_st *transaction);
    void (*vOnTransactionReceive)(struct jee_mqtt_transaction_st *transaction);
    void (*vDispose)(struct jee_mqtt_transaction_st *transaction);
} JeeMQTTTransaction_t, *PJeeMQTTTransaction_t;

typedef struct jee_mqtt_transaction_node_st
{
    PJeeMQTTTransaction_t transaction;
    struct jee_mqtt_transaction_node_st *prev;
    struct jee_mqtt_transaction_node_st *next;
} JeeMQTTTransactionNode_t, *PJeeMQTTTransactionNode_t;

typedef struct jee_mqtt_transaction_manager_st
{
    PJeeMQTTTransactionNode_t head;
    // PJeeMQTTTransactionNode tail;

    void (*add)(struct jee_mqtt_transaction_st *transaction);
    struct jee_mqtt_transaction_st *(*take)(struct jee_mqtt_connection_st *connection, \
        struct jee_mqtt_message_st *response);
    void (*timeout_clean)(struct jee_mqtt_connection_st *conn);
} JeeMQTTTransactionManager_t, *PJeeMQTTTransactionManager_t;

extern void vJeeMQTTTransactionManagerInit(struct jee_mqtt_connection_st *connection);
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _H_JEE_MQTT_TRANSACTION_
