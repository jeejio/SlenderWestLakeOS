#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "mson.h"
#include "jeedef.h"

#define MAX(A, B) ((A) > (B) ? (A) : (B))
#define MIN(A, B) ((A) < (B) ? (A) : (B))

#define DEFTYPE(TYPE, PARAM)         \
    {                                \
#TYPE, sizeof(TYPE), (PARAM) \
    }

#define TYPE_PARAM_DECIMALS (1 << 1) // 小数
#define TYPE_PARAM_UNSIGNED (1 << 2) // 无符号数据
#define TYPE_PARAM_MALLOC (1 << 8)   // 需要申请空间

VarType supportTypeList[] = {
    // 所有struct内用到的普通类型都要在这列出来。
    DEFTYPE(int, 0),
    DEFTYPE(char *, 0),
    DEFTYPE(char*, 0),
    DEFTYPE(char, 0),
    DEFTYPE(jee_int8_t, 0),
    DEFTYPE(jee_int16_t, 0),
    DEFTYPE(jee_int32_t, 0),
    DEFTYPE(jee_int64_t, 0),
    DEFTYPE(jee_uint8_t, TYPE_PARAM_UNSIGNED),
    DEFTYPE(jee_uint16_t, TYPE_PARAM_UNSIGNED),
    DEFTYPE(jee_uint32_t, TYPE_PARAM_UNSIGNED),
    DEFTYPE(jee_uint64_t, TYPE_PARAM_UNSIGNED),
    DEFTYPE(jee_size_t, 0),
    DEFTYPE(float, TYPE_PARAM_DECIMALS),
    DEFTYPE(double, TYPE_PARAM_DECIMALS)
};
const int supportTypeNum = sizeof(supportTypeList) / sizeof(VarType);

#define MEMBER_TYPE_UNKNOWN cJSON_Invalid
#define MEMBER_TYPE_ARRAY cJSON_Array   // 数组
#define MEMBER_TYPE_STRUCT cJSON_Object // 结构体
#define MEMBER_TYPE_VAR (1 << 8)        // 普通变量，如int, char*

const int addressLength = sizeof(int *);

/**
 * 新建memberInfo节点
 */
MemberInfoType *pNewMemberInfoNode(void)
{
    MemberInfoType *memberInfoNode = malloc(sizeof(MemberInfoType));
    if (!memberInfoNode)
    {
        printf("error: %s malloc for memberInfoNode fail.", __FUNCTION__);
        return NULL;
    }
    memberInfoNode->prev = NULL;
    memberInfoNode->next = NULL;
    memberInfoNode->child = NULL;
    memberInfoNode->memberSize = 0;
    memberInfoNode->memberOffset = 0;
    memberInfoNode->memberType = MEMBER_TYPE_UNKNOWN;
    memberInfoNode->memberTypeDetail = NULL;
    memberInfoNode->memberName = NULL;

    return memberInfoNode;
}

/**
 * 链接相邻两个memberInfo节点
 * @param prevMemberInfoNode 前结点
 * @param nextMemberInfoNode 后结点
 */
void vLinkMemberInfoNode(MemberInfoType *prevMemberInfoNode, MemberInfoType *nextMemberInfoNode)
{
    if (prevMemberInfoNode)
    {
        prevMemberInfoNode->next = nextMemberInfoNode;
    }
    if (nextMemberInfoNode)
    {
        nextMemberInfoNode->prev = prevMemberInfoNode;
    }
}

/**
 * 分离字符串中的类型与变量名
 * @param[in] memberString 含有类型和变量名的字符串
 * @param[out] singleMemberStringLength
 * @param[out] typeStringStartPos 类型字符串的起始位置
 * @param[out] typeStringLength 类型字符串的长度
 * @param[out] nameStringStartPos 变量名称字符串的起始位置
 * @param[out] nameStringLength 变量名称字符串的长度
 * @param[out] childStructStringStartPos 子结构体字符串的起始位置
 * @param[out] childStructStringLength 子结构体字符串的长度
 * @param[out] arrayLengthStringStartPos 数组长度字符串的起始位置
 * @param[out] arrayLengthStringLength 数组长度字符串的长度
 * @return 是否分割成功。0:成功，-1:失败
 *
 *
 * @example1
 * "int code;"
 * 类型字符串:"int"
 * 变量名字符串:"code"
 * 额外信息字符串:""
 * 数组长度字符串:""
 * 返回类型: MEMBER_TYPE_VAR
 *
 * @example2
 * "struct{int a;char *msg;}value;"
 * 类型字符串:"struct"
 * 变量名称字符串:"value"
 * 子结构体字符串:"{int a;char *msg;}"
 * 数组长度字符串:""
 * 返回类型: MEMBER_TYPE_STRUCT
 *
 * @example3
 * "int value[10];"
 * 类型字符串:"int"
 * 变量名称字符串:"value"
 * 子结构体字符串:""
 * 数组长度字符串:"[10]"
 * 返回类型: MEMBER_TYPE_ARRAY
 *
 * @example4
 * "struct{int a;char *msg;}result[10];"
 * 类型字符串:"struct"
 * 变量名称字符串:"result"
 * 子结构体字符串:"{int a;char *msg;}"
 * 数组长度字符串:"[10]"
 * 返回类型: MEMBER_TYPE_STRUCT| MEMBER_TYPE_ARRAY
 */
int iSplitTypeAndMember(const char *memberString, int *singleMemberStringLength,
                        int *typeStringStartPos, int *typeStringLength,
                        int *nameStringStartPos, int *nameStringLength,
                        int *childStructStringStartPos, int *childStructStringLength,
                        int *arrayLengthStringStartPos, int *arrayLengthStringLength)
{
    char *ch = (char *)memberString;
    int memberStringLength = 0; // 从memberString开始，类型+变量名+;的长度
    char *nameEndChar, *typeEndChar;
    char *structStartChar, *structEndChar;
    char *arrayLengthStartChar, *arrayLengthEndChar;
    int memberType = 0;

    if ((!ch) || (*ch) == '\0' || (*ch) == ';')
    {
        return MEMBER_TYPE_UNKNOWN;
    }

    int bracesDepth = 0;
    while ((*ch) != '\0' && ((*ch) != ';' || bracesDepth > 0))
    {
        if (*ch == '{')
        {
            bracesDepth++;
            memberType |= MEMBER_TYPE_STRUCT;
        }
        else if (*ch == '}')
        {
            bracesDepth--;
        }

        memberStringLength++;
        ch++;
    }
    if ((*ch) == ';')
    {
        memberStringLength++;
    }

    ch = ch - 1;
    while ((*ch) == ' ' && ch > memberString)
    {
        ch--; // 过滤分号前空格
    }
    if ((*ch) == ']')
    {

        memberType |= MEMBER_TYPE_ARRAY;
        bracesDepth = 0;
        arrayLengthEndChar = ch;
        while (bracesDepth > 0 || (*ch) == '[' || (*ch) == ']' || (*ch) == ' ')
        {
            if (*ch == ']')
            {
                bracesDepth++;
            }
            else if (*ch == '[')
            {
                bracesDepth--;
            }
            ch--;
        }

        arrayLengthStartChar = ch + 1;
        while ((*arrayLengthStartChar) != '[')
        {
            arrayLengthStartChar++;
        }
        *arrayLengthStringStartPos = arrayLengthStartChar - memberString;
        *arrayLengthStringLength = arrayLengthEndChar - arrayLengthStartChar + 1;
    }

    nameEndChar = ch;
    while ((*ch) != ' ' && (*ch) != '*' && (*ch) != '}' && ch > memberString)
    {
        ch--;
    }
    // 此时ch为类型和变量名之间空格的位置，或指针*的位置
    if (ch <= memberString)
    {
        return MEMBER_TYPE_UNKNOWN;
    }
    *nameStringStartPos = ch + 1 - memberString;
    *nameStringLength = nameEndChar - ch;

    if (memberType & MEMBER_TYPE_STRUCT)
    {
        // 如果是结构体，找出最外层大括号
        while ((*ch) != '}' && ch > memberString)
        {
            ch--;
        }
        structEndChar = ch;
        ch = (char *)memberString;
        while ((*ch) != '{' && ch < structEndChar)
        {
            ch++;
        }
        structStartChar = ch;

        *childStructStringStartPos = structStartChar - memberString;
        *childStructStringLength = structEndChar - structStartChar + 1;

        ch--;
    }

    typeEndChar = ch;
    while ((*typeEndChar) == ' ')
    {
        typeEndChar--; // 过滤类型与变量名间多余
    }
    ch = (char *)memberString;
    while ((*ch) == ' ')
    {
        ch++; // 过滤最前面的空格
    }
    *typeStringLength = typeEndChar - ch + 1;
    *typeStringStartPos = ch - memberString;

    if ((*typeStringStartPos) > (*nameStringStartPos))
    {
        return MEMBER_TYPE_UNKNOWN;
    }

    *singleMemberStringLength = memberStringLength;
    if (memberStringLength <= 0)
    {
        return MEMBER_TYPE_UNKNOWN;
    }

    if (memberType)
    {
        return memberType;
    }
    return MEMBER_TYPE_VAR;
}

/**
 * 根据类型字符串获取对应类型的指针
 * @param typeString 类型字符串(example: "int")
 * @return 对应类型信息指针
 */
VarType *pGetVarType(const char *typeString)
{
    for (int i = 0; i < supportTypeNum; ++i)
    {
        VarType *pSupportType = &supportTypeList[i];
        if (strcmp(pSupportType->typeName, typeString) == 0)
        {
            return pSupportType;
        }
    }
    return NULL;
}

/**
 * 获取数组的长度
 * @param arrayLengthString 长度相关字符串
 * @return 是否转换成功
 *
 * @example
 * 输入:"[3]"
 * 输出:3
 */
int iGetArrayLength(const char *arrayLengthString, char **endChar)
{
    if (!arrayLengthString || *(arrayLengthString) == '\0')
    {
        return 0;
    }

    while (((*arrayLengthString) < '0' || (*arrayLengthString) > '9') && (*arrayLengthString) != '\0')
    {
        arrayLengthString++;
    }
    int result = strtol(arrayLengthString, endChar, 10);
    return result;
}

/**
 * 生成新的普通变量类型的memberInfo节点
 * @param varTypeString 类型字符串
 * @return 新节点
 */
MemberInfoType *pNewMemberInfoNodeVar(const char *varTypeString)
{
    MemberInfoType *newMemberInfoNode = pNewMemberInfoNode();
    if (!newMemberInfoNode)
    {
        return NULL;
    }

    newMemberInfoNode->memberTypeDetail = pGetVarType(varTypeString);
    if (!newMemberInfoNode->memberTypeDetail)
    {
        newMemberInfoNode->memberType = MEMBER_TYPE_UNKNOWN;
        printf("\033[31merror: The variable type %s is not registered!\033[m\n", varTypeString);
        free(newMemberInfoNode);
        return NULL;
    }
    newMemberInfoNode->memberType = MEMBER_TYPE_VAR;
    newMemberInfoNode->memberSize = newMemberInfoNode->memberTypeDetail->typeSize;
    newMemberInfoNode->memberMaxSize = newMemberInfoNode->memberSize;
    return newMemberInfoNode;
}

/**
 * 生成新的数组类型memberInfo节点
 * @param childMemberInfoNode 子类型
 * @param arrayLength 数组长度
 * @return 新节点
 */
MemberInfoType *pNewMemberInfoNodeArray(MemberInfoType *childMemberInfoNode, const char *arrayLengthString)
{
    if (!childMemberInfoNode)
    {
        return NULL;
    }

    char *nextArrayLengthChar = (char *)arrayLengthString;

    int arrayLength = iGetArrayLength(arrayLengthString, &nextArrayLengthChar);

    if (arrayLength == 0)
    {
        return childMemberInfoNode;
    }

    MemberInfoType *newMemberInfoNode = pNewMemberInfoNode();
    if (!newMemberInfoNode)
    {
        return NULL;
    }

    newMemberInfoNode->memberType = MEMBER_TYPE_ARRAY;
    newMemberInfoNode->memberArrayLength = arrayLength;
    newMemberInfoNode->child = pNewMemberInfoNodeArray(childMemberInfoNode, nextArrayLengthChar);
    if (newMemberInfoNode->child)
    {
        newMemberInfoNode->memberSize = newMemberInfoNode->child->memberSize * newMemberInfoNode->memberArrayLength;
        newMemberInfoNode->memberMaxSize = newMemberInfoNode->child->memberMaxSize;
    }

    return newMemberInfoNode;
}

/**
 * 解析结构体中成员信息
 * @param structString 结构体成员字符串
 * @return 结构体信息
 */
MemberInfoType *mson_pParseStructMember(const char *structString)
{
    int structStringLength = strlen(structString);
    int restMemberStringLength = structStringLength;
    int memberMaxSize = 0;
    MemberInfoType *rootMemberNode = pNewMemberInfoNode();
    MemberInfoType *currentMemberNode = NULL;

    if (!rootMemberNode)
    {
        return NULL;
    }

    char *currentMemberString = (char *)structString;

    while (currentMemberString && restMemberStringLength > 0)
    {
        int singleMemberLength;
        int typeStringStartPos, typeLength;
        int nameStringStartPos, nameLength;
        int childStructStringStartPos, childStructStringLength;
        int arrayLengthStringStartPos, arrayLengthStringLength;
        int splitResult;

        char typeString[256];
        char nameString[256];

        splitResult = iSplitTypeAndMember(currentMemberString, &singleMemberLength,
                                          &typeStringStartPos, &typeLength,
                                          &nameStringStartPos, &nameLength,
                                          &childStructStringStartPos, &childStructStringLength,
                                          &arrayLengthStringStartPos, &arrayLengthStringLength);
        if (splitResult == MEMBER_TYPE_UNKNOWN)
        {
            break;
        }

        MemberInfoType *newMemberInfoNode = NULL;

        //        printf("%s\n", currentMemberString);
        //        printf("%d %d %d %d %d\n", singleMemberLength, typeStringStartPos, typeLength, nameStringStartPos, nameLength);
        //        printf("%d %d %d %d\n", childStructStringStartPos, childStructStringLength, arrayLengthStringStartPos,
        //               arrayLengthStringLength);

        strncpy(typeString, currentMemberString + typeStringStartPos, typeLength);
        typeString[typeLength] = '\0';
        strncpy(nameString, currentMemberString + nameStringStartPos, nameLength);
        nameString[nameLength] = '\0';

        //        printf("<%s> <%s>\n", typeString, nameString);

        switch (splitResult)
        {
        case MEMBER_TYPE_VAR:
        {
            newMemberInfoNode = pNewMemberInfoNodeVar(typeString);
            break;
        }
        case MEMBER_TYPE_STRUCT:
        {
            //                char childStructString[1024];//拷贝出子结构体内容，去除大括号
            //                strncpy(childStructString, currentMemberString + childStructStringStartPos + 1,
            //                        childStructStringLength - 2);
            //                childStructString[childStructStringLength] = '\0';

            char *childStructString;
            childStructString = strndup(currentMemberString + childStructStringStartPos + 1,
                                        childStructStringLength - 2);

            newMemberInfoNode = mson_pParseStructMember(childStructString);

            free(childStructString);

            break;
        }
        case MEMBER_TYPE_ARRAY:
        {
            char arrayLengthString[64];
            strncpy(arrayLengthString, currentMemberString + arrayLengthStringStartPos, arrayLengthStringLength);
            arrayLengthString[arrayLengthStringLength] = '\0';

            MemberInfoType *childNode = pNewMemberInfoNodeVar(typeString);
            newMemberInfoNode = pNewMemberInfoNodeArray(childNode, arrayLengthString);
            break;
        }
        case MEMBER_TYPE_STRUCT | MEMBER_TYPE_ARRAY:
        {
            char arrayLengthString[64];
            strncpy(arrayLengthString, currentMemberString + arrayLengthStringStartPos, arrayLengthStringLength);
            arrayLengthString[arrayLengthStringLength] = '\0';
            char *childStructString;
            childStructString = strndup(currentMemberString + childStructStringStartPos + 1,
                                        childStructStringLength - 2);

            MemberInfoType *childNode = mson_pParseStructMember(childStructString);
            free(childStructString);
            newMemberInfoNode = pNewMemberInfoNodeArray(childNode, arrayLengthString);
            break;
        }

        default:
            printf("error: unknown member type!\n");
        }

        if (newMemberInfoNode)
        {
            if (!currentMemberNode)
            {
                rootMemberNode->memberType = MEMBER_TYPE_STRUCT;
                rootMemberNode->child = newMemberInfoNode;
            }
            else
            {
                vLinkMemberInfoNode(currentMemberNode, newMemberInfoNode);
                newMemberInfoNode->memberOffset = currentMemberNode->memberOffset + currentMemberNode->memberSize;

                int alignSize = MIN(newMemberInfoNode->memberMaxSize, addressLength);
                if (newMemberInfoNode->memberOffset % alignSize > 0)
                {
                    int paddingSize = alignSize - newMemberInfoNode->memberOffset % alignSize;
                    newMemberInfoNode->memberOffset += paddingSize;
                    rootMemberNode->memberSize += paddingSize;
                }
                //                printf("%s maxsize=%d offset=%d\n", nameString, alignSize, newMemberInfoNode->memberOffset);
            }
            newMemberInfoNode->memberName = strdup(nameString);
            rootMemberNode->memberSize += newMemberInfoNode->memberSize;
            memberMaxSize = MAX(memberMaxSize, newMemberInfoNode->memberMaxSize);
            currentMemberNode = newMemberInfoNode;
        }

        restMemberStringLength -= singleMemberLength;
        currentMemberString = currentMemberString + singleMemberLength;
    }

    if (!rootMemberNode->child)
    {
        free(rootMemberNode);
        return NULL;
    }

    rootMemberNode->memberMaxSize = memberMaxSize;
    int alignSize = MIN(memberMaxSize, addressLength);
    if (rootMemberNode->memberSize % alignSize > 0)
    {
        rootMemberNode->memberSize += alignSize - rootMemberNode->memberSize % alignSize;
    }

    return rootMemberNode;
}

/**
 * 根据成员名从指定结构体中获取成员信息
 * @param memberInfo 结构体成员信息
 * @param memberName 成员名字符串
 * @return 成员信息
 */
MemberInfoType *pGetMemberTypeByMemberName(MemberInfoType *rootMemberInfo, const char *memberName)
{
    if (!rootMemberInfo)
    {
        return NULL;
    }
    MemberInfoType *memberInfo = rootMemberInfo->child;
    while (memberInfo)
    {
        if (strcmp(memberInfo->memberName, memberName) == 0)
        {
            return memberInfo;
        }
        memberInfo = memberInfo->next;
    }
    return NULL;
}

/**
 * 删除结构体成员信息并释放空间
 * @param memberInfo
 */
void mson_vDeleteMemberInfo(MemberInfoType *memberInfo)
{
    MemberInfoType *nextMemberInfo;
    while (memberInfo)
    {
        nextMemberInfo = memberInfo->next;
        if (memberInfo->child)
        {
            mson_vDeleteMemberInfo(memberInfo->child);
        }
        if (memberInfo->memberName)
        {
            free((void *)memberInfo->memberName);
        }
        free(memberInfo);
        memberInfo = nextMemberInfo;
    }
}

/**
 * 删除结构体成员信息并释放空间
 * @param memberInfo
 */
void mson_vDeleteMemberInfoWithObj(MemberInfoType *memberInfo, void *obj)
{
    MemberInfoType *nextMemberInfo;
    while (memberInfo)
    {
        nextMemberInfo = memberInfo->next;
        if (memberInfo->child)
        {
            mson_vDeleteMemberInfoWithObj(memberInfo->child, (void *)obj + memberInfo->memberOffset);
        }

        if (memberInfo->memberTypeDetail && (memberInfo->memberTypeDetail->typeParam & TYPE_PARAM_MALLOC) > 0)
        {
            char *strAddress = NULL;
            memcpy(&strAddress, (void *)obj + memberInfo->memberOffset, addressLength);

            //            printf("free address:%X\n", strAddress);
            //            printf("%s:free string %s\n", __FUNCTION__, strAddress);

            free(strAddress);
        }
        if (memberInfo->memberName)
        {
            free((void *)memberInfo->memberName);
        }
        free(memberInfo);
        memberInfo = nextMemberInfo;
    }
}

/**
 * 向obj写入cJson Number型数据
 * @param item cJson数字项
 * @param obj 目的地址
 * @param memberInfo 成员信息
 */
void vWriteCJsonNumberToObj(cJSON *item, void *obj, MemberInfoType *memberInfo)
{
    if (!item || !memberInfo)
    {
        return;
    }
    if (item->type != cJSON_Number)
    {
        return;
    }
    printf("%s: write number \"%d\" \n", __FUNCTION__, item->valueint);
    if (memberInfo->memberType != MEMBER_TYPE_VAR)
    {
        printf("error: The member is not an integer variable!\n");
        return;
    }
    memberInfo->memberTypeDetail->typeParam &= ~TYPE_PARAM_MALLOC;
    if (memberInfo->memberTypeDetail->typeParam & TYPE_PARAM_DECIMALS)
    {
        // 小数
        switch (memberInfo->memberTypeDetail->typeSize)
        {
        case 4:
            *(float *)(obj) = (float)item->valuedouble;
            break;
        case 8:
            *(double *)(obj) = (double)item->valuedouble;
            break;
        default:
            printf("error: Recognition of decimal variables of size %dBytes is not supported!",
                   memberInfo->memberTypeDetail->typeSize);
        }
    }
    else if (memberInfo->memberTypeDetail->typeParam & TYPE_PARAM_UNSIGNED)
    {
        // 无符号整型
        switch (memberInfo->memberTypeDetail->typeSize)
        {
        case 1:
            *(unsigned char *)(obj) = (unsigned char)item->valueint;
            break;
        case 2:
            *(unsigned short *)(obj) = (unsigned short)item->valueint;
            break;
        case 4:
            *(unsigned int *)(obj) = (unsigned int)item->valueint;
            break;
        case 8:
            *(unsigned long long int *)(obj) = (unsigned long long int)item->valueint;
            break;
        default:
            printf("error: Unsigned integer variables of size %dBytes are not recognized",
                   memberInfo->memberTypeDetail->typeSize);
        }
    }
    else
    {
        // 有符号整型
        switch (memberInfo->memberTypeDetail->typeSize)
        {
        case 1:
            *(char *)(obj) = (char)item->valueint;
            break;
        case 2:
            *(short *)(obj) = (short)item->valueint;
            break;
        case 4:
            *(int *)(obj) = (int)item->valueint;
            break;
        case 8:
            *(long long int *)(obj) = (long long int)item->valueint;
            break;
        default:
            printf("error: Integer variables of size %dBytes are not recognized",
                   memberInfo->memberTypeDetail->typeSize);
        }
    }
}

/**
 * 向obj写入cJson String型数据
 * @param item cJson字符串项
 * @param obj 目的地址
 * @param memberInfo 成员信息
 */
void vWriteCJsonStringToObj(cJSON *item, void *obj, MemberInfoType *memberInfo)
{
    if (!item || !memberInfo)
    {
        return;
    }
    if (item->type != cJSON_String)
    {
        return;
    }

    printf("%s: write string \"%s\" \n", __FUNCTION__, item->valuestring);

    if (memberInfo->memberType == MEMBER_TYPE_VAR)
    {
        // 认为这是一个字符指针
        if (memberInfo->memberSize == addressLength)
        {
            char *strNew = strdup(item->valuestring);
            char debugStr[100];
            sprintf(debugStr, "strdup address:%X\n", strNew);
            printf("%s", debugStr);
            memcpy(((void *)obj), &strNew, addressLength);
            sprintf(debugStr, "obj char* address:%X\n", *(unsigned long long int *)obj);
            printf("%s", debugStr);

            memberInfo->memberTypeDetail->typeParam |= TYPE_PARAM_MALLOC;
            //            char *strAddress = NULL;
            //            switch (addressLength) {
            //                case 1:
            //                    strAddress = (char *) *(unsigned char *) obj;
            //                    break;
            //                case 2:
            //                    strAddress = (char *) *(unsigned short *) obj;
            //                    break;
            //                case 4:
            //                    strAddress = (char *) *(unsigned int *) obj;
            //                    break;
            //                case 8:
            //                    strAddress = (char *) *(unsigned long long int *) obj;
            //                    break;
            //            }
            //            printf("%s: string address:%X\n", __FUNCTION__, strAddress);
            //            if (strAddress) {
            //                strcpy(strAddress, item->valuestring);
            //            }
        }
        else
        {
            printf("error: The member is not a String variable\n");
        }
    }
    else if (memberInfo->memberType == MEMBER_TYPE_ARRAY)
    {
        // 认为这是一个字符数组
        if (memberInfo->child && memberInfo->child->memberSize == 1)
        {
            int copyLength = MIN(strlen(item->valuestring), memberInfo->memberArrayLength - 1);
            strncpy(obj, item->valuestring, copyLength);
        }
        else
        {
            printf("error: The member is not a String variable\n");
        }
    }
    else
    {
        printf("error: The member is not a String variable\n");
        return;
    }

    //    printf("[%s]target address:%X\n", item->string, ((void *) obj) + memberInfo->memberOffset);
    //    printf("[%s]string offset:%d\n", item->string, memberInfo->memberOffset);
}

/**
 * 向obj写入cJson True/False数据
 * @param item cJson布尔项
 * @param obj 目的地址
 * @param memberInfo 成员信息
 */
void vWriteCJsonBoolToObj(cJSON *item, void *obj, MemberInfoType *memberInfo)
{
    if (!item || !memberInfo)
    {
        return;
    }

    if (item->type != cJSON_True && item->type != cJSON_False)
    {
        return;
    }

    if (memberInfo->memberType != MEMBER_TYPE_VAR)
    {
        printf("error: The member is not a Boolean variable!\n");
        return;
    }

    int value = (item->type == cJSON_True) ? 1 : 0;

    if (memberInfo->memberTypeDetail->typeParam & TYPE_PARAM_DECIMALS)
    {
        // 小数
        printf("error: A Boolean is written to the decimal variable!\n");
    }
    else if (memberInfo->memberTypeDetail->typeParam & TYPE_PARAM_UNSIGNED)
    {
        // 无符号整型
        switch (memberInfo->memberTypeDetail->typeSize)
        {
        case 1:
            *(unsigned char *)(obj) = (unsigned char)value;
            break;
        case 2:
            *(unsigned short *)(obj) = (unsigned short)value;
            break;
        case 4:
            *(unsigned int *)(obj) = (unsigned int)value;
            break;
        case 8:
            *(unsigned long long int *)(obj) = (unsigned long long int)value;
            break;
        default:
            printf("error: Unsigned integer variables of size %dBytes are not recognized",
                   memberInfo->memberTypeDetail->typeSize);
        }
    }
    else
    {
        // 有符号整型
        switch (memberInfo->memberTypeDetail->typeSize)
        {
        case 1:
            *(char *)(obj) = (char)value;
            break;
        case 2:
            *(short *)(obj) = (short)value;
            break;
        case 4:
            *(int *)(obj) = (int)value;
            break;
        case 8:
            *(long long int *)(obj) = (long long int)value;
            break;
        default:
            printf("error: Integer variables of size %dBytes are not recognized",
                   memberInfo->memberTypeDetail->typeSize);
        }
    }
}

/**
 * 向obj写入cJson NULL数据
 * @param item cJson项
 * @param obj 目的地址
 * @param memberInfo 成员信息
 */
void vWriteCJsonNullToObj(cJSON *item, void *obj, MemberInfoType *memberInfo)
{
    if (!item || !memberInfo)
    {
        return;
    }

    if (item->type != cJSON_NULL)
    {
        return;
    }

    memset(obj, 0, memberInfo->memberSize);
}

/**
 * cJSON格式转obj
 * @param root cJSON
 * @param obj 结构体object
 * @param structInfo 结构体信息
 */
void cJsonToObj(cJSON *root, void *obj, MemberInfoType *structInfo)
{
    if (!root || !structInfo)
    {
        return;
    }

    switch (root->type)
    {
    /*
    #define cJSON_Invalid (0)
    #define cJSON_False  (1 << 0)
    #define cJSON_True   (1 << 1)
    #define cJSON_NULL   (1 << 2)
    #define cJSON_Number (1 << 3)
    #define cJSON_String (1 << 4)
    #define cJSON_Array  (1 << 5)
    #define cJSON_Object (1 << 6)
    #define cJSON_Raw    (1 << 7)
    */
    case cJSON_NULL:
    {
        vWriteCJsonNullToObj(root, obj, structInfo);
        break;
    }
    case cJSON_False:
    case cJSON_True:
    {
        vWriteCJsonBoolToObj(root, obj, structInfo);
        break;
    }
    case cJSON_Number:
    {
        vWriteCJsonNumberToObj(root, obj, structInfo);
        break;
    }
    case cJSON_String:
    {
        vWriteCJsonStringToObj(root, obj, structInfo);
        break;
    }
    case cJSON_Array:
    {
        if (structInfo->memberType != MEMBER_TYPE_ARRAY)
        {
            return;
        }
        int stepSize = structInfo->child->memberSize;
        int offset = 0;

        cJSON *current_item = root->child;
        while (current_item && offset < structInfo->memberSize)
        {
            //              printf("[%s]object step into offset:%X\n", current_item->string, targetMemberInfo->memberOffset);
            cJsonToObj(current_item, ((void *)obj) + offset, structInfo->child);

            offset += stepSize;
            current_item = current_item->next;
        }
        break;
    }
    case cJSON_Object:
    {
        if (structInfo->memberType != MEMBER_TYPE_STRUCT)
        {
            return;
        }

        cJSON *current_item = root->child;
        while (current_item)
        {
            MemberInfoType *targetMemberInfo = pGetMemberTypeByMemberName(structInfo, current_item->string);
            if (!targetMemberInfo)
            {
                printf("error: not found member \"%s\" in structInfo!\n", current_item->string);
                goto next_node;
            }

            cJsonToObj(current_item, ((void *)obj) + targetMemberInfo->memberOffset, targetMemberInfo);

        next_node:
            current_item = current_item->next;
        }
        break;
    }
    default:
        printf("error: unSupport json type[%d].\n", root->type);
    }
}

void __mson_release(void *obj, MemberInfoType *structMemberInfo)
{
    mson_vDeleteMemberInfoWithObj(structMemberInfo, obj);
    free(obj);
}

void *__mson_pJsonToObj(const char *json, const char *structInfo, MemberInfoType **structMemberInfo)
{
    printf("%s: start.\n",__FUNCTION__ );
    if (!json || !structInfo)
    {
        *structMemberInfo = NULL;
        return NULL;
    }
    printf("%s\n", structInfo);
    cJSON *root = cJSON_Parse(json);
    if (!root)
    {
        printf("[%s] parse json failed\r\n", __FUNCTION__);
        *structMemberInfo = NULL;
        return NULL;
    }
    *structMemberInfo = mson_pParseStructMember(structInfo);

    void *obj = malloc((*structMemberInfo)->memberSize);
    memset(obj, 0, (*structMemberInfo)->memberSize);
    printf("obj address:%X\n", obj);

    if (!obj)
    {
        mson_vDeleteMemberInfo(*structMemberInfo);
        *structMemberInfo = NULL;
        goto exit;
    }

    cJsonToObj(root, obj, *structMemberInfo);

exit:
    cJSON_Delete(root);
    printf("%s: end.\n",__FUNCTION__ );
    return obj;
}
