

#include "SampleUtils.h"
#include "ZGlobals.h"
#include "NLMEDE.h"
/*********************************************************************
 * @fn      SampleApp_Uint16ToStr
 *
 * @brief   将16位数据转换成字符串
 *
 * @param   addrCA
 *
 * @return  none
 */
void SampleApp_Uint16ToStr(uint16 guid, uint8* guid_str)
{
    uint8 ch;
    ch = guid / 256 / 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    guid_str[0] = ch;
    ch = guid / 256 % 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    guid_str[1] = ch;
    ch = guid % 256 / 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    guid_str[2] = ch;
    ch = guid % 256 % 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    guid_str[3] = ch;
}


/*********************************************************************
 * @fn      SampleApp_GetShortAddrStr
 *
 * @brief   point to point.
 *
 * @param   addrCA
 *
 * @return  none
 */
void SampleApp_GetShortAddrStr(uint8* addrCA)
{
    uint8 ch;
    uint16 _shortAddr;
    _shortAddr = NLME_GetShortAddr();
    ch = _shortAddr / 256 / 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    addrCA[0] = ch;
    ch = _shortAddr / 256 % 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    addrCA[1] = ch;
    ch = _shortAddr % 256 / 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    addrCA[2] = ch;
    ch = _shortAddr % 256 % 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    addrCA[3] = ch;
}
/*********************************************************************
 * @fn      SampleApp_GetShortAddrStr
 *
 * @brief   从4字节字符串中获取16bit的短地址
 *
 * @param   addrCA
 *
 * @return  none
 */
uint8 _SampleApp_CharToHex(uint8 ch)
{
    if( ch >= '0' && ch <= '9' )
    {
        return ch - '0';
    }
    else if( ch >= 'A' && ch <= 'F' )
    {
        return ch - 'A' + 10;
    }
    else
    {
        return 0;
    }
}
uint16 SampleApp_GetShortAddrUint16(uint8* addrCA)
{
    uint16 _shortAddr = 0;
    uint8 i = 0;
    for( i=0;i<4;i++ )
    {
        _shortAddr *= 16;
        _shortAddr += _SampleApp_CharToHex(addrCA[i]);
    }
    return _shortAddr;
}

/*********************************************************************
 * @fn      SampleUtil_CompareUint8Array
 *
 * @brief   
 *
 * @param   addrCA
 *
 * @return  none
 */
uint8 SampleUtil_CompareUint8Array(uint8* dest, uint8* src,uint8 length)
{
    uint8 i;
    for( i=0; i<length; i++ )
    {
        if( dest[i] != src[i] )
        {
            return 0x01;
        }
    }
    return 0x00;
}

/*********************************************************************
 * @fn      SampleApp_GetShortAddrStr
 *
 * @brief   从4位十六进制字符数组中获取shortAddress
 *
 * @param   addrCA
 *
 * @return  none
 */
uint16 SampleUtil_GetShortAddrFromU8Array(uint8* array)
{
    uint16 short_addr = 0x00;
    uint8 i = 0x00;
    for( i=0; i<4; i++ )
    {
        short_addr *= 16;
        if( array[i] >= '0' && array[i] <= '9' )
        {
            short_addr += array[i] - '0';
        }
        else
        {
            short_addr +=  array[i] - 'A' + 10;
        }
    }
    return short_addr;
}

/*********************************************************************
 * @fn      SampleApp_GetShortAddrStr
 *
 * @brief   从n位十进制字符数组中获取Value
 *
 * @param   addrCA
 *
 * @return  none
 */
uint16 SampleUtil_GetValueFromU8Array(uint8* array, uint8 len)
{
    uint16 value = 0x00;
    uint8 i = 0x00;
    for( i=0; i<len; i++ )
    {
        value *= 10;
        value += array[i] >= 'A' ? (array[i]-'A'+10):(array[i]-'0');
    }
    return value;
}

/*********************************************************************
 * @fn      SampleApp_GetShortAddrStr
 *
 * @brief   验证GUID格式是否正确
 *
 * @param   addrCA
 *
 * @return  none
 */
uint8 SampleUtil_ValidateGuidFormat(uint8* pGuid)
{
    uint8 i = 0x00;
    for( i=0; i<4; i++ )
    {
        if( !( (pGuid[i] >= '0' && pGuid[i] <= '9')
            || (pGuid[i] >= 'A' && pGuid[i] <= 'F')
             ) )
        {
            return 0x01;
        }
    }
    return 0x00;
}