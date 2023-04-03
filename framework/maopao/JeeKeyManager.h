#ifndef _JEE_KEY_MANAGER_H_
#define _JEE_KEY_MANAGER_H_ (1)

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define SECRET_KEY_BIN "/data/secret_key.bin"
#define DEVICE_ID_BYTES 8

extern int32_t lGetIntDeviceID(int64_t *data);
extern int32_t lGetIntDeviceIDFromFlash(int64_t *data);


#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _JEE_KEY_MANAGER_H_
