#ifndef _H_JEE_MQTT_MESSAGE_
#define _H_JEE_MQTT_MESSAGE_ (1)

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus
#if 0
typedef struct jee_mqtt_request_st
{
    char id[128];
    char method[128];
    char *params;
} JeeMQTTRequest_t, *PJeeMQTTRequest_t;

typedef struct jee_mqtt_response_st
{
    char id[128];
    int32_t code;
    const char *msg;
    char *result;
}JeeMQTTResponse_t, *PJeeMQTTResponse_t;
#endif
typedef struct jee_mqtt_message_st
{
    char id[128];         // Request Response
    char method[64];      // Request
    char jsonrpc[16];     // Request Response
    uint32_t code;        // Response
    char *msg;            // Respose
    char *payload;        // Request Response
} JeeMQTTMessage_t, *PJeeMQTTMessage_t;

extern PJeeMQTTMessage_t pxJeeMQTTMessage_Creator(void);

extern void vJeeMQTTMessageReq_Destroy(PJeeMQTTMessage_t message);
extern void vJeeMQTTMessageResp_Destroy(PJeeMQTTMessage_t message);
extern void vJeeMQTTMessage_Destroy(PJeeMQTTMessage_t message);

extern void vJeeMQTTMessage_SetPayload(PJeeMQTTMessage_t message, const char *payload);
extern void vJeeMQTTMessage_SetMessage(PJeeMQTTMessage_t message, const char *msg);

/*
*解析RPC消息，获取消息类型，返回JERRNO_REQUEST or JERRNO_RESPONSE
*/
extern int32_t lJeeGetRPCJsonMsgType(char *rpc);

extern int32_t lJeeMQTTMessage_ToJson(PJeeMQTTMessage_t message, char *buf, const int32_t length);
extern int32_t lJeeJson_ToMQTTMessage(PJeeMQTTMessage_t mqttMsg, char *buf, const int32_t length);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _H_JEE_MQTT_MESSAGE_
