#ifndef _H_JEE_ERRNO_
#define _H_JEE_ERRNO_ (1)

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Request 没有 code，0x80 表示 code 的无效值
#define JERRNO_REQUEST (0x80)
#define JERRNO_RESPONSE (0X70)

// 成功
#define JERRNO_SUCCESS (0x00)

#define JERRNO_NOMEM (0x01)
#define JERRNO_INVAL (0x02)

#define JERRNO_RPC_SUCCESS (200)
#define JERRNO_RPC_FAILED (500)

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _H_JEE_ERRNO_
