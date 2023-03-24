#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_motor.h"

/****************MOTOR********************/
MSON_DEFOBJ(MotorRotationDirection,
            char *rotationDirection;);

MSON_DEFOBJ(MotorSpinVelocityLevel,
            char *spinVelocityLevel;);

MSON_DEFOBJ(MotorOnoff,
            char *Onoff;);

int32_t lMotorGetRotationDirectionProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int32_t ret = lTalMotorGetRotationDirection();
    char *direction = "Forward";
    if (MOTOR_DIR_FORWARD == ret)
    {
        direction = "Forward";
    }
    else if (MOTOR_DIR_BACKWARD == ret)
    {
        direction = "Backward";
    }
    else if (MOTOR_DIR_LEFT == ret)
    {
        direction = "Left";
    }
    else if (MOTOR_DIR_RIGHT == ret)
    {
        direction = "Right";
    }
    else if (MOTOR_DIR_FORWARD_RIGHT == ret)
    {
        direction = "ForwardRight";
    }
    else if (MOTOR_DIR_FORWARD_LEFT == ret)
    {
        direction = "ForwardLeft";
    }
    else if (MOTOR_DIR_BACKWARD_RIGHT == ret)
    {
        direction = "BackwardRight";
    }
    else if (MOTOR_DIR_BACKWARD_LEFT == ret)
    {
        direction = "BackwardLeft";
    }
    else
    {
        printf("GetRotationDirection return unknow:%d\r\n", ret);
        direction = "";
    }
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            strlen(direction) ? JERRNO_RPC_SUCCESS : JERRNO_RPC_FAILED, direction, "");
    return ret;
}

int32_t lMotorGetSpinVelocityLevelProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    char *speed = "Fast";
    int32_t ret = lTalMotorGetSpinVelocityLevel();
    if (MOTOR_SPEED_SLOW == ret)
    {
        speed = "Slow";
    }
    else if (MOTOR_SPEED_MIDDLE == ret)
    {
        speed = "Middle";
    }
    else if (MOTOR_SPEED_FAST == ret)
    {
        speed = "Fast";
    }
    else
    {
        printf("GetSpinVelocityLevel return unknow:%d\r\n", ret);
        speed = "";
    }
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            strlen(speed) ? JERRNO_RPC_SUCCESS : JERRNO_RPC_FAILED, speed, "");
    return ret;
}

int32_t lMotorGetOnOffProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int32_t ret = lTalMotorGetOnoff();
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");
    return 0;
}

int32_t lMotorSetRotationDirectionProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);

    int32_t ret = 0;
    MotorDirection_t setDir = MOTOR_INIT;
    MotorRotationDirection *motorRotationDirection = (MotorRotationDirection *)obj;
    if (!motorRotationDirection->rotationDirection)
    {
        printf("Parse json rotationDirection error\r\n");
        ret = -1;
        goto exit;
    }
    if (!strcmp(motorRotationDirection->rotationDirection, "Forward"))
    {
        setDir = MOTOR_DIR_FORWARD;
    }
    else if (!strcmp(motorRotationDirection->rotationDirection, "Backward"))
    {
        setDir = MOTOR_DIR_BACKWARD;
    }
    else if (!strcmp(motorRotationDirection->rotationDirection, "Left"))
    {
        setDir = MOTOR_DIR_LEFT;
    }
    else if (!strcmp(motorRotationDirection->rotationDirection, "Right"))
    {
        setDir = MOTOR_DIR_RIGHT;
    }
    else if (!strcmp(motorRotationDirection->rotationDirection, "ForwardLeft"))
    {
        setDir = MOTOR_DIR_FORWARD_LEFT;
    }
    else if (!strcmp(motorRotationDirection->rotationDirection, "ForwardRight"))
    {
        setDir = MOTOR_DIR_FORWARD_RIGHT;
    }
    else if (!strcmp(motorRotationDirection->rotationDirection, "BackwardLeft"))
    {
        setDir = MOTOR_DIR_BACKWARD_LEFT;
    }
    else if (!strcmp(motorRotationDirection->rotationDirection, "BackwardRight"))
    {
        setDir = MOTOR_DIR_BACKWARD_RIGHT;
    }
    else
    {
        printf("Set rotation direction value error:%s\r\n", motorRotationDirection->rotationDirection);
        ret = -1;
        goto exit;
    }
    printf("Set rotation direction:%s->%d\r\n", motorRotationDirection->rotationDirection, setDir);
    vTalMotorSetRotationDirection(setDir);
    sprintf(result, "{\"code\":%d, \"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

exit:
    return ret;
}

int32_t lMotorSetSpinVelocityLevelProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);

    int32_t ret = 0;
    MotorSpeed_t speed = MOTOR_SPEED_SLOW;
    MotorSpinVelocityLevel *motorSpinVelocityLevel = (MotorSpinVelocityLevel *)obj;

    if (!motorSpinVelocityLevel->spinVelocityLevel)
    {
        printf("Parse json spinVelocityLevel error\r\n");
        ret = -1;
        goto exit;
    }
    if (!strcmp(motorSpinVelocityLevel->spinVelocityLevel, "Fast"))
    {
        speed = MOTOR_SPEED_FAST;
    }
    else if (!strcmp(motorSpinVelocityLevel->spinVelocityLevel, "Middle"))
    {
        speed = MOTOR_SPEED_MIDDLE;
    }
    else if (!strcmp(motorSpinVelocityLevel->spinVelocityLevel, "Slow"))
    {
        speed = MOTOR_SPEED_SLOW;
    }
    else
    {
        printf("Set spinVelocityLevel value error\r\n");
        ret = -1;
        goto exit;
    }
    printf("Set spinVelocityLevel:%s->%d\r\n", motorSpinVelocityLevel->spinVelocityLevel, speed);
    vTalMotorSetSpinVelocityLevel(speed);
    sprintf(result, "{\"code\":%d, \"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

exit:
    return ret;
}

int32_t lMotorSetOnOffProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);

    int32_t ret = 0;
    uint8_t onoff = 0;
    MotorOnoff *motorOnoff = (MotorOnoff *)obj;

    if (!motorOnoff->Onoff)
    {
        printf("Parse json Onoff error\r\n");
        ret = -1;
        goto exit;
    }
    if (!strcmp(motorOnoff->Onoff, "True"))
    {
        onoff = 1;
    }
    else if (!strcmp(motorOnoff->Onoff, "False"))
    {
        onoff = 0;
    }
    else
    {
        printf("Set motor onoff value error\r\n");
        ret = -1;
        goto exit;
    }
    printf("Set motor on off:%s->%d\r\n", motorOnoff->Onoff, onoff);
    vTalMotorSetOnoff(onoff);
    sprintf(result, "{\"code\":%d, \"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

exit:
    return ret;
}

/************MOTOR_API*************/
JeeDevApi_t Motor_API[] =
    {
        REGISTER_API("getRotationDirection", lMotorGetRotationDirectionProcess, NONE_OBJ),
        REGISTER_API("getSpinVelocityLevel", lMotorGetSpinVelocityLevelProcess, NONE_OBJ),
        REGISTER_API("getOnOff", lMotorGetOnOffProcess, NONE_OBJ),
        REGISTER_API("setRotationDirection", lMotorSetRotationDirectionProcess, MotorRotationDirection),
        REGISTER_API("setSpinVelocityLevel", lMotorSetSpinVelocityLevelProcess, MotorSpinVelocityLevel),
        REGISTER_API("setOnOff", lMotorSetOnOffProcess, MotorOnoff),
        {
            .method = NULL,
        },
};

int32_t Motor_Object_Init(void *args)
{
    printf("Motor init\r\n");
    lTalMotorInit();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return 0;
}

DevObject_t MotorObject =
    {
        .lDevObjectInit = Motor_Object_Init,
        .devApi = Motor_API,
};
