
#ifndef _SAMPLE_UTILS_H_
#define _SAMPLE_UTILS_H_

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
    void SampleApp_GetGUIDStr(uint8* str);
#endif //终端设备
    void SampleApp_Uint16ToStr(uint16 guid, uint8* guid_str);
    void SampleApp_GetShortAddrStr(uint8* addrCA);
    uint8 _SampleApp_CharToHex(uint8 ch);
    uint16 SampleApp_GetShortAddrUint16(uint8* addrCA);

    uint8  SampleUtil_CompareUint8Array(uint8* dest, uint8* src, uint8 length);
    uint16 SampleUtil_GetShortAddrFromU8Array(uint8* array);
    uint16 SampleUtil_GetValueFromU8Array(uint8* array, uint8 len);
    uint8  SampleUtil_ValidateGuidFormat(uint8* pGuid);


#endif // _SAMPLE_UTILS_H_