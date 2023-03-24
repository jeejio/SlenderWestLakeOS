#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_steering_gear.h"

/************SteeringGear 舵机*************/
MSON_DEFOBJ(SteeringGearRotationDirection,
            char *rotationDirection;);

MSON_DEFOBJ(SteeringGearRotationAngle,
            int rotationAngle;);

// 0:Clockwise
// 1:Counterclockwise
int32_t lGetRotationDirectionProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int32_t ret = lTalSteeringGearGetRotationDirction();
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "Counterclockwise" : "Clockwise"), "");
    return ret;
}

/*返回值： int型，取值范围：0-180*/
int32_t lGetRotationAngleProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int32_t ret = lTalSteeringGearGetRotationAngle();
    sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, (int)ret, "");
    return ret;
}

// set direction, 0: Clockwise, 1:Counterclockwise
int32_t lSetRotationDirectionnProcess(void *obj, char *result, int32_t resultLen)
{
    printf("[%s] is running\r\n", __FUNCTION__);
    CHECK_API_OBJS(obj, result, resultLen);

    int32_t ret = 0;
    int setDir = 0;
    SteeringGearRotationDirection *steeringGearRotationDirection = (SteeringGearRotationDirection *)obj;

    if (!steeringGearRotationDirection->rotationDirection)
    {
        printf("Parse json rotationDirection error\r\n");
        ret = -1;
        goto exit;
    }
    printf("Set rotation direction:%s\r\n", steeringGearRotationDirection->rotationDirection);
    if (!strcmp(steeringGearRotationDirection->rotationDirection, "Clockwise"))
    {
        setDir = 1;
    }
    else if (!strcmp(steeringGearRotationDirection->rotationDirection, "Counterclockwise"))
    {
        setDir = 0;
    }
    else
    {
        printf("Set rotation direction value error\r\n");
        ret = -1;
        goto exit;
    }
    vTalSteeringGearSetRotationDirction(setDir);
    sprintf(result, "{\"code\":%d, \"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

exit:
    return ret;
}

// set angle：取值范围0-180
int32_t lSetRotationAngleProcess(void *obj, char *result, int32_t resultLen)
{
    printf("[%s] is running\r\n", __FUNCTION__);
    CHECK_API_OBJS(obj, result, resultLen);
    int ret;
    SteeringGearRotationAngle *steeringGearRotationAngle = (SteeringGearRotationAngle *)obj;

    printf("Set rotation angle:%d\r\n", steeringGearRotationAngle->rotationAngle);
    if (steeringGearRotationAngle->rotationAngle < 0 || steeringGearRotationAngle->rotationAngle > 180)
    {
        printf("Set rotation angle value error\r\n");
        ret = -1;
        goto exit;
    }
    vTalSteeringGearSetRotationAngle(steeringGearRotationAngle->rotationAngle);
    sprintf(result, "{\"code\":%d, \"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

exit:
    return ret;
}

/************SteeringGear_API*************/
JeeDevApi_t SteeringGear_API[] =
    {
        REGISTER_API("getRotationDirection", lGetRotationDirectionProcess, NONE_OBJ),
        REGISTER_API("getRotationAngle", lGetRotationAngleProcess, NONE_OBJ),
        REGISTER_API("setRotationDirection", lSetRotationDirectionnProcess, SteeringGearRotationDirection),
        REGISTER_API("setRotationAngle", lSetRotationAngleProcess, SteeringGearRotationAngle),
        {
            .method = NULL,
        },
};

int32_t SteeringGear_Object_Init(void *args)
{
    printf("SteeringGear init\r\n");
    vTalSteeringGearInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    return 0;
}

DevObject_t SteeringGearObject =
    {
        .lDevObjectInit = SteeringGear_Object_Init,
        .devApi = SteeringGear_API,
};
