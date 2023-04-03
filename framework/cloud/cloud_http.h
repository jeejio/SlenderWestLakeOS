
#ifndef __CLOUD_HTTP_H__
#define __CLOUD_HTTP_H__
#include "webclient.h"
#include "string.h"
#include "cJSON.h"
#define DEV  (0)

#define TOKEN_LENGTH (512)

extern char token[TOKEN_LENGTH];
extern SemaphoreHandle_t cloudMutex;

int webclient_post_cloud(void);
static int webclient_post_comm(const char *uri, const void *post_data, unsigned int data_len);



#endif