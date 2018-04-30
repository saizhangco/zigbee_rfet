/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"
#include "SampleRF.h"
#include "SampleUtils.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "hal_lcd.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
    SAMPLEAPP_PERIODIC_CLUSTERID,
    SAMPLEAPP_FLASH_CLUSTERID,
    SAMPLEAPP_P2P_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
    SAMPLEAPP_ENDPOINT,              //  int Endpoint;
    SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
    SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
    SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
    SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
    SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
    (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
    SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
    (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr; //广播
afAddrType_t SampleApp_Flash_DstAddr;    //组播
afAddrType_t SampleApp_P2P_DstAddr;      //点播

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

//应用层变量
endDevMsg_t localEndDevMsg;
appMsg_t    localAppMsg;
uint8 SampleAppAckEvent = NOT_EXIST;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_Send_P2P_Message(void);
void SampleApp_GetShortAddrStr(uint8* addrCA);
uint8 _SampleApp_CharToHex(uint8 ch);
uint16 SampleApp_GetShortAddrUint16(uint8* addrCA);
void SampleApp_Uint16ToStr(uint16 guid, uint8* guid_str);

#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
//串口处理函数
void SampleApp_ProcessUartData(mtOSALSerialData_t *MSGpkt); 
//通过无线发送数据到终端设备
void SampleApp_Send_C2E_Message( uint8* msg );              
#endif //协调器

#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
void SampleApp_GetGUIDStr(uint8* str);
// 终端设备给协调器回复
void SampleApp_Send_E2C_Ack( uint8* value, uint8 length );
// 终端设备主动发送PushButton事件
void SampleApp_Send_E2C_PushBtn_Evt( uint8 evt );
#endif  //终端设备

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{ 
    uint16 _guid = GUID;
    uint8  _guid_str[9];
    SampleApp_TaskID = task_id;
    SampleApp_NwkState = DEV_INIT;
    SampleApp_TransID = 0;
  
    MT_UartInit();                  //串口初始化
    MT_UartRegisterTaskID(task_id); //注册串口任务
  
    // Device hardware initialization can be added here or in main() (Zmain.c).
    // If the hardware is application specific - add it here.
    // If the hardware is other parts of the device add it in main().

#if defined ( BUILD_ALL_DEVICES )
    // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
    // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
    // together - if they are - we will start up a coordinator. Otherwise,
    // the device will start as a router.
    if ( readCoordinatorJumper() )
        zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
    else
        zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
    // HOLD_AUTO_START is a compile option that will surpress ZDApp
    //  from starting the device and wait for the application to
    //  start the device.
    ZDOInitDevice(0);
#endif

    // Setup for the periodic message's destination address
    // Broadcast to everyone
    SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;//广播
    SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

    // Setup for the flash command's destination address - Group 1
    SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;//组播
    SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
    SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播 
    SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 

#if 0
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//如果是协调器
    SampleApp_P2P_DstAddr.addr.shortAddr = 0x0001;            //发给终端设备
#else
    SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器
#endif
#endif // 0

    // Fill out the endpoint description.
    SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_epDesc.task_id = &SampleApp_TaskID;
    SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
    SampleApp_epDesc.latencyReq = noLatencyReqs;

    // Register the endpoint description with the AF
    afRegister( &SampleApp_epDesc );

    // Register for all key events - This app will handle all key events
    RegisterForKeys( SampleApp_TaskID );

    // By default, all devices start out in Group 1
    SampleApp_Group.ID = 0x0001;
    osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
    aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
    _guid_str[0] = 'G';
    _guid_str[1] = 'u';
    _guid_str[2] = 'i';
    _guid_str[3] = 'd';
    _guid_str[4] = ':';
    SampleApp_Uint16ToStr(_guid,&_guid_str[5]);
    HalLcdWriteString( (char*)_guid_str, HAL_LCD_LINE_4 );
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
    afIncomingMSGPacket_t *MSGpkt;
    (void)task_id;  // Intentionally unreferenced parameter

    if ( events & SYS_EVENT_MSG )
    {
        MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
        while ( MSGpkt )
        {
            switch ( MSGpkt->hdr.event )
            {
                // Received when a key is pressed
                case KEY_CHANGE:
                    SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
                    break;

                // Received when a messages is received (OTA) for this endpoint
                case AF_INCOMING_MSG_CMD:
                    SampleApp_MessageMSGCB( MSGpkt );
                    break;

                // Received whenever the device changes state in the network
                case ZDO_STATE_CHANGE:
                    SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                    if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
                         //(SampleApp_NwkState == DEV_ROUTER)   || 
			 (SampleApp_NwkState == DEV_END_DEVICE) )
                    {
                        // Start sending the periodic message in a regular interval.
                        osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                    }
                    else
                    {
                        // Device is no longer in the network
                    }
                    break;
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
                case CMD_SERIAL_MSG:
#ifdef ZS_DEBUG_UART
                    HalUARTWrite(0,"CMD_SERIAL_MSG\n", sizeof("CMD_SERIAL_MSG\n"));
#endif
                    SampleApp_ProcessUartData((mtOSALSerialData_t *)MSGpkt);
                    break;
#endif
                default:
                    break;
            }

            // Release the memory
            osal_msg_deallocate( (uint8 *)MSGpkt );

            // Next - if one is available
            MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    // Send a message out - This event is generated by a timer
    //  (setup in SampleApp_Init()).
    if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
    {
        // Send the periodic message
        //SampleApp_SendPeriodicMessage();
        SampleApp_Send_P2P_Message();

        // Setup to send message again in normal period (+ a little jitter)
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
            (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

        // return unprocessed events
        return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
    }
    if( events & SAMPLEAPP_SEND_RESPONSE_MSG_EVT )
    {
        SampleApp_Send_P2P_Message();
	return (events ^ SAMPLEAPP_SEND_RESPONSE_MSG_EVT);
    }
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
    //触发按键确认事件
    if( events & SAMPLEAPP_KEY_ACK_EVT )
    {
#if 1 //调试-确认事件触发
        HalUARTWrite(0x00, "SAMPLEAPP_KEY_ACK_EVT\n", 22);       
#endif
        //HalLcdInit();
        //1 判断是否存在确认事件
        if( SampleAppAckEvent != NOT_EXIST )
        {
            //2 根据localEndDevMsg中的内容发送确认事件
            if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"TKME",4) )
            {
                // Take Medicines
                SampleApp_Send_E2C_PushBtn_Evt(SampleAppAckEvent);
            }
            else if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"ADME",4) )
            {
                // Add Medicines
                SampleApp_Send_E2C_PushBtn_Evt(SampleAppAckEvent);
            }
        }
        return ( events ^ SAMPLEAPP_KEY_ACK_EVT );
    }
    //触发按键查询事件
    if( events & SAMPLEAPP_KEY_QUERY_EVT )
    {
#if 1 //查询事件触发调试
        HalUARTWrite(0x00, "SAMPLEAPP_KEY_QUERY_EVT\n", 24);       
#endif
        //1 判断是否存在确认事件
        if( SampleAppAckEvent == NOT_EXIST )
        {
            // Query Medicines
            SampleApp_Send_E2C_PushBtn_Evt(QUERY_MEDIC_EVT);
        }
        return ( events ^ SAMPLEAPP_KEY_QUERY_EVT );
    }
#endif  //终端设备
    // Discard unknown events
    return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
#if 0
    uint8 str4lcd[6];
#endif
    (void)shift;  // Intentionally unreferenced parameter
  
    if ( keys & HAL_KEY_SW_1 )
    {
        /* This key sends the Flash Command is sent to Group 1.
         * This device will not receive the Flash Command from this
         * device (even if it belongs to group 1).
         */
        //SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
        //替换成我的程序，进行按键处理
        HalUARTWrite(0x00, "HAL_KEY_SW_1\n", 13);       //按键1触发调试
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
        
#elif ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备                                                   //终端设备
	//发送按键 确认动作 事件
	osal_set_event( SampleApp_TaskID, SAMPLEAPP_KEY_ACK_EVT );
#endif
    }

    if ( keys & HAL_KEY_SW_2 )
    {
        /* The Flashr Command is sent to Group 1.
         * This key toggles this device in and out of group 1.
         * If this device doesn't belong to group 1, this application
         * will not receive the Flash command sent to group 1.
         */
        /*
        aps_Group_t *grp;
        grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
        if ( grp )
        {
            // Remove from the group
            aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
        }
        else
        {
            // Add to the flash group
            aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
        }
        */
        HalUARTWrite(0x00, "HAL_KEY_SW_2\n", 13);       //按键2串口调试
    }
    if ( keys & HAL_KEY_SW_6 )
    {
        HalUARTWrite(0x00, "HAL_KEY_SW_6\n", 13);       //按键6串口调试
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
	    //设置按键 查询动作 事件
	    osal_set_event( SampleApp_TaskID, SAMPLEAPP_KEY_QUERY_EVT );
#endif
//==================================================================
#if 0
        //获取设备短地址，并通过发送到串口
        SampleApp_GetShortAddrStr(str4lcd);
        HalUARTWrite(0x00, str4lcd, 4);
        HalUARTWrite(0x00, "\n", 1);
        str4lcd[4] = ':';
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
        str4lcd[5] = 'C';
#elif ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ROUTER )	//路由器
        str4lcd[5] = 'R';
#elif ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
        str4lcd[5] = 'E';
	    //设置按键 查询动作 事件
	    osal_set_event( SampleApp_TaskID, SAMPLEAPP_KEY_QUERY_EVT );
#else 
        str4lcd[5] = 'U'; 
#endif
	    //将短地址在LCD上显示
        HalLcdWriteString( (char*)str4lcd, HAL_LCD_LINE_3 );
#endif // if 0
//===================================================================
    }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
    uint16 flashTime;
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
    //uint16 _shortAddr = 0x01;/* 保存短地址 */
    uint8 i = 0x00;  //用来计数的
    uint8 format = 0x00;
    uint8 tmp[4] = {0};
    uint8 *value;
#else                                                   //终端设备
    uint8 *lcdData;
    uint8 i = 0x00;
    uint16 tmp = 0x00;
    uint16 num = 0x00;
    uint8  format = 0x00;
    uint8 clearScreen[16] = {0x20};
#endif

    switch ( pkt->clusterId )
    {
        case SAMPLEAPP_P2P_CLUSTERID:

#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
#if 0 //是否启用串口调试
            HalUARTWrite(0, "Coord:[", 7);       //提示接收到数据
            HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //串口输出接收到的数据
            HalUARTWrite(0, "]\n", 2);         // 回车换行
#endif //是否启用串口调试
#elif ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
	    HalUARTWrite(0,   "End:[", 5);       //提示接收到数据
            HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //串口输出接收到的数据
            HalUARTWrite(0, "]\n", 2);         // 回车换行
#endif
                    
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
	    /*
            //返回数据到终端设备
            _shortAddr = SampleApp_GetShortAddrUint16(pkt->cmd.Data);
            SampleApp_P2P_DstAddr.addr.shortAddr = _shortAddr;            //发给终端设备
            //SampleApp_Send_P2P_Message();
            osal_set_event( SampleApp_TaskID, SAMPLEAPP_SEND_RESPONSE_MSG_EVT );
            */
            /*
             * 0~3 id
             * 4~7 command
             * 8   length - n
             * 9~n+9 value
             */
            //1.将接收到的数据保存在全局变量中。收到数据的长度必须大于9，否则丢弃。
            if( pkt->cmd.DataLength >= 9 )
            {
                // id
                for( i=0; i<4; i++ )
                {
                    localEndDevMsg.guid[i] = pkt->cmd.Data[i];
                }
                // command
                for( i=0; i<4; i++ )
                {
                    localEndDevMsg.command[i] = pkt->cmd.Data[i+4];
                }
                localEndDevMsg.length = pkt->cmd.Data[8];
                if( localEndDevMsg.length > 0 )
                {
                    value = (uint8*)osal_msg_allocate(localEndDevMsg.length);
                    if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"PING",4) )
                    {
                        if( pkt->cmd.DataLength >= 13)
                        {
#if 0
                            for( i=0; i<4; i++ )
                            {
                                localEndDevMsg.value.short_addr[i] = pkt->cmd.Data[i+9];
                            }
#endif
                            for( i=0; i<localEndDevMsg.length; i++ )
                            {
                                value[i] = pkt->cmd.Data[i+9];
                            }
                        }
                        else
                        {
                            //格式错误
                            format = 0x01;
                        }
                    }
                    else if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"TKME",4) ||
                             0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"ADME",4)
                            )
                    {
                        if( pkt->cmd.DataLength >= (9+localEndDevMsg.length) )
                        {
                            for( i=0; i<localEndDevMsg.length; i++ )
                            {
                                value[i] = pkt->cmd.Data[i+9];
                            }
                        }
                        else
                        {
                            //格式错误
                            format = 0x01;
                        }
                    }
                    else  //扩展
                    {
                        if( pkt->cmd.DataLength >= (9+localEndDevMsg.length) )
                        {
                            for( i=0; i<localEndDevMsg.length; i++ )
                            {
                                value[i] = pkt->cmd.Data[i+9];
                            }
                        }
                        else
                        {
                            //格式错误
                            format = 0x01;
                        }
                    }
                }
            }
            else
            {
                //格式错误
                format = 0x02;
            }
            //2 通过串口发送到上位机
            if( format == 0x01 )
            { 
                //格式错误
                
            }
            else if( format == 0x02 )
            {
                //格式错误
            }
            else if( format == 0x00 )
            {
                /*
                 * sof
                 * length
                 * guid
                 * command
                 * value
                 */
                HalUARTWrite(0x00, "#", 1);
                tmp[0] = localEndDevMsg.length;
                HalUARTWrite(0x00, tmp, 1);
                HalUARTWrite(0x00, localEndDevMsg.guid, 4);
                HalUARTWrite(0x00, localEndDevMsg.command, 4);
                //HalUARTWrite(0x00, localEndDevMsg.value.short_addr, 4);
                if( localEndDevMsg.length > 0 )
                {
                    tmp[0] = localEndDevMsg.length;
                    HalUARTWrite(0x00, value, localEndDevMsg.length);
                    osal_msg_deallocate(value);
                }
#if 1   //调试时，控制台换行
                HalUARTWrite(0x00, "\n", 1);
#endif
            }
#elif ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
            /*
             * 0~3 id
             * 4~7 command
             * 8   length - n
             * 9~n+9 value
             */
            //1.将接收到的数据保存在全局变量中。收到数据的长度必须大于9，否则丢弃。
            if( pkt->cmd.DataLength >= 9 )
            {
                // id
                for( i=0; i<4; i++ )
                {
                    localEndDevMsg.guid[i] = pkt->cmd.Data[i];
                }
                // command
                for( i=0; i<4; i++ )
                {
                    localEndDevMsg.command[i] = pkt->cmd.Data[i+4];
                }
                // length
                localEndDevMsg.length = pkt->cmd.Data[8];
                // value
                if( localEndDevMsg.length > 0 )
                {
                    if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"TKME",4)   // TKME
                       || 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"ADME",4) // ADME
                       || 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"LTME",4) // LTME
                         )
                    {
                        if( pkt->cmd.DataLength >= (localEndDevMsg.length+9))
                        {
                            localEndDevMsg.value.number = SampleUtil_GetValueFromU8Array(&pkt->cmd.Data[9], localEndDevMsg.length);
                        }
                        else
                        {
                            //格式错误2，value的数据长度与length不匹配
                            format = 0x02;
                        }
                    }
                }
            }
            else
            {
                //格式错误1，数据长度小于9个字节
                format = 0x01;
            }
            //2 通过串口发送到上位机
            if( format == 0x01 )
            { 
                //格式错误1，数据长度小于9个字节
                //无线回复 "FEEG" "Generic Error"
                SampleApp_Send_E2C_Ack("FEEG",4);
            }
            else if( format == 0x02 )
            {
                //格式错误2，value的数据长度与length不匹配
                //无线回复 "FEEV" value数据长度不匹配
                SampleApp_Send_E2C_Ack("FEEV",4);
            }
            else if( format == 0x00 )
            {
                /*
                 * sof
                 * length
                 * guid
                 * command
                 * value
                 */
#if 1
                //1 验证ID格式，错误无线回复 "FEEI"，并break
                if( 0x00 != SampleUtil_ValidateGuidFormat(localEndDevMsg.guid) )
                {
                    //无线回复 "FEEI"
                    SampleApp_Send_E2C_Ack("FEEI",4); 
                    break;
                }
                //2 验证CMD命令格式，错误无线回复 "FEEC"，并break
                if( ! ( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"TKME",4)   // TKME
                       || 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"ADME",4) // ADME
                       || 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"AKTK",4) // AKTK
                       || 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"AKAD",4) // AKAD
                       || 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"LTME",4) // LTME
                      ) )
                {
                    //无线回复 "FEEC"
                    SampleApp_Send_E2C_Ack("FEEC",4); 
                    break;
                }
                //3 验证ID和当前设备FW写入的ID是否匹配，不匹配无线回复 "IDNM"，并break
                if( GUID != SampleUtil_GetShortAddrFromU8Array(localEndDevMsg.guid) )
                {
                    //无线回复 "IDNM"
                    SampleApp_Send_E2C_Ack("IDNM",4); 
                    break;
                }

                //4.1 在LCD上显示，并点亮指示灯，无线回复 "OK"
                if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"TKME",4)   // TKME
                       || 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"ADME",4) // ADME
                       || 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"LTME",4) // LTME
                  )
                {
                    lcdData = (uint8*)osal_msg_allocate(7+localEndDevMsg.length);
                    for( i=0;i<4;i++ )
                    {
                        lcdData[i] = localEndDevMsg.command[i];
                    }
                    lcdData[4] = '[';
                    num = localEndDevMsg.value.number;
                    for( i=0;i<localEndDevMsg.length;i++ )
                    {
                        tmp = num % 10;
                        lcdData[localEndDevMsg.length+4-i] = tmp+'0';
                        num /= 10;
                    }
                    lcdData[5+localEndDevMsg.length] = ']';
                    lcdData[6+localEndDevMsg.length] = '\0';
                    HalLcdWriteString( (char*)lcdData, HAL_LCD_LINE_3 );
                    osal_msg_deallocate((uint8*)lcdData);
                    // 点亮指示灯
                    if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"TKME",4))   // TKME
                    {
                        HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);  
                        HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);
                    }
                    else if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"ADME",4))   // ADME
                    {
                        HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);
                        HalLedSet (HAL_LED_2, HAL_LED_MODE_ON);
                    }
                    else if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"LTME",4) )    // LTME
                    {
                        HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);
                        HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);
                        SampleAppAckEvent = NOT_EXIST;
                    }
                    //无线回复 "OK"
                    SampleApp_Send_E2C_Ack("OK",2);
                    
                    if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"TKME",4)   // TKME
                      )
                    {
                        SampleAppAckEvent = TAKE_MEDIC_EVT;
                    }
                    else if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"ADME",4) // ADME
                           )
                    {
                        SampleAppAckEvent = ADD_MEDIC_EVT;
                    }
                }
                //4.2 在LCD上显示，并关闭指示灯，无线回复 "OK"
                else if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"AKTK",4) // AKTK
                       )
                {
                    SampleAppAckEvent = NOT_EXIST;
                    HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF); 
                    HalLcdWriteString( (char*)clearScreen, HAL_LCD_LINE_3 );
                    //无线回复 "OK"
                    SampleApp_Send_E2C_Ack("OK",2);
                }
                //4.2 在LCD上显示，并关闭指示灯，无线回复 "OK"
                else if( 0x00 == SampleUtil_CompareUint8Array(localEndDevMsg.command,"AKAD",4) // AKAD
                       )
                {
                    SampleAppAckEvent = NOT_EXIST;
                    HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);
                    HalLcdWriteString( (char*)clearScreen, HAL_LCD_LINE_3 );
                    //无线回复 "OK"
                    SampleApp_Send_E2C_Ack("OK",2);
                }
#endif
#if 0
                HalLcdWriteStringValue("TAKE:",localEndDevMsg.value.number,10,HAL_LCD_LINE_3);
#endif
            }
#endif
            break;    
        case SAMPLEAPP_PERIODIC_CLUSTERID:
            break;

        case SAMPLEAPP_FLASH_CLUSTERID:
            flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
            HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
            break;
    }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                         SAMPLEAPP_PERIODIC_CLUSTERID,
                         1,
                         (uint8*)&SampleAppPeriodicCounter,
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
        // Error occurred in request to send.
    }
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
    uint8 buffer[3];
    buffer[0] = (uint8)(SampleAppFlashCounter++);
    buffer[1] = LO_UINT16( flashTime );
    buffer[2] = HI_UINT16( flashTime );

    if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                         SAMPLEAPP_FLASH_CLUSTERID,
                         3,
                         buffer,
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
        // Error occurred in request to send.
    }
}

/*********************************************************************
 * @fn      SampleApp_Send_P2P_Message
 *
 * @brief   point to point.
 *          协调器或终端设备通过RF传输数据
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_Send_P2P_Message( void )
{
    uint16 length = 0x00;
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
    uint8 data[11]="++++++++++";
    length = 11;
#elif ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
    /*
     * 0~3 id
     * 4~7 command
     * 8   length - n
     * 9~n+9 value
     */
    uint8 i = 0;
    uint8 data[13]= {0};
    uint8 guid_str[4] = {0};
    uint8 short_addr[4] = {0};
    SampleApp_GetShortAddrStr(short_addr);
    SampleApp_GetGUIDStr(guid_str);
    for( i=0; i<4; i++ )
    {
        data[i] = guid_str[i];
    }
    data[4] = 'P';
    data[5] = 'I';
    data[6] = 'N';
    data[7] = 'G';
    data[8] = 0x04;
    for( i=0; i<4; i++ )
    {
        data[i+9] = short_addr[i];
    }
    length = 13;
#endif
  
    if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       length,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
        // Error occurred in request to send.
    }
}

/*********************************************************************
 * @fn      SampleApp_Send_C2E_Message
 *
 * @brief   point to point.
 *          协调器或终端设备通过RF传输数据
 *
 * @param   none
 *
 * @return  none
 */
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
void SampleApp_Send_C2E_Message( uint8* msg )
{
    uint16 length = msg[SRF_POS_LEN] + 9; // n+9
    uint16 _shortAddr = 0x00;
    uint8* data;
    uint8 i = 0x00;
    
    //1 验证ID格式，错误串口回复 "FECI"，并return
    //2 验证CMD命令格式，错误串口回复 "FECC"，并return
    //3 验证ShortAddress格式，错误串口回复 "SADE"，并return
    
    data = (uint8*)osal_msg_allocate(length);
    
    _shortAddr = SampleUtil_GetShortAddrFromU8Array(&msg[SRF_POS_SA0]);
#define C2E_DEBUG 0
#if C2E_DEBUG    
    HalUARTWrite(0x00, &msg[SRF_POS_SA0], 4);
#endif
    SampleApp_P2P_DstAddr.addr.shortAddr = _shortAddr;//0xDE8A;            //发给终端设备
    /*
     * 0~3 guid
     * 4~7 command
     * 8   length
     * 9~n+8 value
     */
    for( i=0;i<4;i++ )
    {
        data[i] = msg[SRF_POS_ID0+i];
    }
    for( i=0;i<4;i++ )
    {
        data[i+4] = msg[SRF_POS_CMD0+i];
    }
    data[8] = msg[SRF_POS_LEN];
    for( i=0;i<data[8];i++ )
    {
        data[9+i] = msg[SRF_POS_DAT0+i];
    }

#if C2E_DEBUG
    HalUARTWrite(0x00, "A\n", 2);
#endif
    if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       length,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
#if C2E_DEBUG
        HalUARTWrite(0x00, "B\n", 2);
#endif
    }
    else
    {
        // Error occurred in request to send.
#if C2E_DEBUG
        HalUARTWrite(0x00, "C\n", 2);
#endif
    }
    // Release the memory
    osal_msg_deallocate( (uint8 *)data );
}
#endif  //协调器

/*********************************************************************
 * @fn      SampleApp_Send_E2C_Ack
 *
 * @brief   终端设备消息回复
 *
 * @param   none
 *
 * @return  none
 */
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
void SampleApp_Send_E2C_Ack( uint8* value, uint8 length )
{
    uint8  af_len = 9 + length;
    uint8* data;
    uint8 i = 0x00;
    // localEndDevMsg
    /*
     * 0~3   guid
     * 4~7   command
     * 8     length - n
     * 9~8+n value
     */
    
    data = (uint8*)osal_msg_allocate(af_len);
    SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器
    // guid
    for( i=0;i<4;i++ )
    {
        data[i] = localEndDevMsg.guid[i];
    }
    // command
    for( i=0;i<4;i++ )
    {
        data[i+4] = localEndDevMsg.command[i];
    }
    data[8] = length;
    for( i=0;i<length;i++ )
    {
        data[9+i] = value[i];
    }
  
    HalUARTWrite(0x00, "E2C_Ack A\n", 10);
    if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       af_len,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
        HalUARTWrite(0x00, "E2C_Ack B\n", 10);
    }
    else
    {
        // Error occurred in request to send.
        HalUARTWrite(0x00, "E2C_Ack C\n", 10);
    }
    // Release the memory
    osal_msg_deallocate( (uint8 *)data );
}
#endif  //终端设备

/*********************************************************************
 * @fn      SampleApp_Send_E2C_Ack
 *
 * @brief   终端设备消息回复
 *
 * @param   none
 *
 * @return  none
 */
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
void SampleApp_Send_E2C_PushBtn_Evt( uint8 evt )
{
    uint8  af_len = 9;
    uint8* data;
    uint8 i = 0x00;
    uint8 guid[4] = {0};
    // localEndDevMsg
    /*
     * 0~3   guid
     * 4~7   command
     * 8     length - n
     * 9~8+n value (目前为空)
     */
    
    //1 申请数据内存空间
    data = (uint8*)osal_msg_allocate(af_len);
    //2 通过宏定义获取Guid
    SampleApp_GetGUIDStr(guid);
    //3 修改发送地址
    SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器
    //4 填充发送数据
    // guid
    for( i=0;i<4;i++ )
    {
        data[i] = guid[i];
    }
    // command
    if( evt == TAKE_MEDIC_EVT )
    {
        data[4] = 'A';
        data[5] = 'K';
        data[6] = 'T';
        data[7] = 'K';
    }
    else if( evt == ADD_MEDIC_EVT )
    {
        data[4] = 'A';
        data[5] = 'K';
        data[6] = 'A';
        data[7] = 'D';
    }
    else if( evt == QUERY_MEDIC_EVT )
    {
        data[4] = 'L';
        data[5] = 'T';
        data[6] = 'M';
        data[7] = 'E';
    }
    else
    {
        osal_msg_deallocate( (uint8 *)data );
        return;
    }
    data[8] = 0x00;
  
    HalUARTWrite(0x00, "E2C_Evt A\n", 10);
    if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       af_len,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
        HalUARTWrite(0x00, "E2C_Evt B\n", 10);
    }
    else
    {
        // Error occurred in request to send.
        HalUARTWrite(0x00, "E2C_Evt C\n", 10);
    }
    // Release the memory
    osal_msg_deallocate( (uint8 *)data );
}
#endif  //终端设备

/***********************************************************
 * @fn      SampleApp_ProcessUartData
 *
 * @brief   将从串口接收到的数据，从串口发出
 *
 * @param   MSGpkt - 数据包内容
 *
 * @return  none
 */
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_COORDINATOR )	//协调器
void SampleApp_ProcessUartData(mtOSALSerialData_t *MSGpkt)
{  
    //为了正确地进行下面工作，用mtOSALSerialData_t类型来指向整个zigbee数据包(不是串口数据包)
    uint8 *pMsg;
    pMsg = MSGpkt->msg;
    //定义一个指针，指向真正的串口接收数据存放位置，MSGptk里面还有一些别的Header噢。
    switch ( MSGpkt->hdr.event )
    {
      case CMD_SERIAL_MSG://如果是串口消息，则进行下面的处理
        //HalLedSet( HAL_LED_RED, HAL_LED_MODE_FLASH );//用LED灯指示一下收到数据啦
        /*
        uint8 *pBuffer;
        uint8 datalength;
        uint8 i;    //定义几个变量，为从接收到的串口包里面提取数据以及写回串口做准备
        datalength = *pMsg++;
        //串口包中第一个字节是数据长度噢
        pBuffer = osal_mem_alloc(datalength);
        //分配一块内存准备把串口消息数据拿出来
        if(pBuffer != NULL)
        {
          for(i = 0;i < datalength; i++)
             *pBuffer++ = *pMsg++;
          //把消息中的串口数据按照datalength数量挨个捞出来放血(放血啥意思？把池子里面那东西捞出来放血，明白不？
          //HalUARTWrite(0,pBuffer,datalength);    //捞出来放血的串口数据再写回串口，也就是送到串口助手显示
          osal_mem_free(pBuffer); //动态申请的内存记得用完了free一下噢
        }
        */
#ifdef ZS_DEBUG_UART
        HalUARTWrite(0x00,"<" ,1);    /* 调试使用 */
        HalUARTWrite(0x00,&pMsg[SRF_POS_ID0] ,pMsg[MT_RPC_POS_LEN]+12);  /* 调试使用 */
        HalUARTWrite(0x00,">\n" ,2);  /* 调试使用 */
#endif
        SampleApp_Send_C2E_Message(&pMsg[MT_RPC_POS_LEN]);
        break;
      default:
        break;
    }
}
#endif //协调器

/*********************************************************************
 * @fn      SampleApp_GetShortAddrStr
 *
 * @brief   point to point.
 *
 * @param   addrCA
 *
 * @return  none
 */
#if ( ZSTACK_DEVICE_BUILD == DEVICE_BUILD_ENDDEVICE )	//终端设备
void SampleApp_GetGUIDStr(uint8* str)
{
    uint8 ch;
    uint16 _guid = GUID;
    ch = _guid / 256 / 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    str[0] = ch;
    ch = _guid / 256 % 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    str[1] = ch;
    ch = _guid % 256 / 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    str[2] = ch;
    ch = _guid % 256 % 16;
    ch = ch > 9 ? ch + 'A' - 10 : ch + '0';
    str[3] = ch;
}
#endif //终端设备
