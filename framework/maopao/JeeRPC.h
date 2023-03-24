#ifndef _H_JEE_RPC_
#define _H_JEE_RPC_ (1)

#include <stdint.h>
#include "bsp_define.h"
#include "JeeErrno.h"
#include "JeeMQTTConnection.h"
#include "JeeMQTTTransaction.h"
#include "JeeMQTTMessage.h"
#include "mson.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus


/* Public */

#define REGISTER_API(methodName, apiProcess, objName) {.method = methodName, .lJeeDevApiProcess = apiProcess, .objString = MSON_MEMBERS_TO_STRING(objName) }


typedef void (*vDevSendEventResult_t) (const char *result);
typedef int32_t (*lDevApiProcess_t)(void *args, char *result, int32_t resultLen);

typedef struct JeeDevApi_st
{
    char *method;
    lDevApiProcess_t lJeeDevApiProcess;
    char *objString;
    struct JeeDevApi_st *prev;
    struct JeeDevApi_st *next;
} JeeDevApi_t, *PJeeDevApi_t;

/**
 * 注册 RPC
 */
extern int32_t JeeRPC_Register(PJeeDevApi_t pJeeDevApi);

/**
 * 反注册 RPC
 */
extern void JeeRPC_UnRegister(const char *method);

/**
 * 设备向客户端上报事件
 */
extern void JeeRPC_SendEvent(struct jee_mqtt_connection_st *conn,
                                vDevSendEventResult_t cb_result, const char *payload);

/* Protected */

extern int32_t JeeRPC_Call(const char *method, const char *params,
                            char *result, int32_t resultLen);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _H_JEE_RPC_
