#ifndef MSON_H
#define MSON_H

#include "cJSON.h"

#define MSON_OK 0
#define MSON_ERROR 1

// #define MEMBER2(TYPE, NAME) TYPE NAME

#define MSON_DEFOBJ(NAME, MEMBERS...)            \
    typedef struct NAME                          \
    {                                            \
        MEMBERS                                  \
    } NAME, *P##NAME;                            \
    const char *const NAME##_Members = #MEMBERS; \
    const int NAME##_SIZE = sizeof(NAME);
#define MSON_MEMBERS_TO_STRING(NAME) NAME##_Members
#define MSON_MEMBERS_SIZE(NAME) NAME##_SIZE
#define NONE_OBJ_Members NULL

typedef struct varType_t
{
    const char *typeName; // 类型名称
    int typeSize;         // 类型大小
    int typeParam;        // 类型相关属性
} VarType;

typedef struct memberInfoType_t
{
    struct memberInfoType_t *prev;
    struct memberInfoType_t *next;

    struct memberInfoType_t *child;

    int memberSize;        // 成员大小
    int memberOffset;      // 偏移位置（相对于父节点）
    int memberArrayLength; // 数组长度（仅类型为array时有值）
    int memberMaxSize;     // 最大成员的大小，若为struct即计算struct内最大成员大小（用于内存对齐计算）

    int memberType;            // 成员类型，见宏定义MEMBER_TYPE_* (如普通变量,array,struct)
    VarType *memberTypeDetail; // 成员类型详情（仅普通变量时有值）

    const char *memberName; // 成员名
} MemberInfoType;

/**
 * 解析结构体中成员信息
 * @param structString 结构体成员字符串
 * @return 结构体信息
 */
MemberInfoType *mson_pParseStructMember(const char *structString);

/**
 * 删除结构体成员信息并释放空间
 * @param memberInfo
 */
void mson_vDeleteMemberInfo(MemberInfoType *memberInfo);

/**
 * 将json字符串转换为结构体
 * @param json json字符串
 * @param obj 目标结构体（已分配空间）
 * @param structInfo 结构体信息
 * @return 转换状态(成功:0, 失败:1)
 */
void *__mson_pJsonToObj(const char *json, const char *structInfo, MemberInfoType **structMemberInfo);

void __mson_release(void *obj, MemberInfoType *structMemberInfo);

#define mson_jsonToObj(__json, __structInfo, __obj) \
    MemberInfoType *__msonMemberInfo;               \
    void *__obj = __mson_pJsonToObj(__json, __structInfo, &__msonMemberInfo)
#define mson_vRelease(obj) __mson_release(obj, __msonMemberInfo)

#endif
