
#ifndef _SAMPLE_RF_H_
#define _SAMPLE_RF_H_

#define SRF_UART_SOF   '#'

#define SRF_FRAME_HDR_SZ 13

#define SRF_POS_LEN    0
#define SRF_POS_ID0    1
#define SRF_POS_SA0    5
#define SRF_POS_CMD0   9
#define SRF_POS_DAT0   13


#ifdef RFID             /* �ն��豸���ڱ���׶λ��GUID */
#define GUID RFID	/* ͨ���꣬����GUID */
#else                   /* Э������Ĭ�ϵ�GUIDΪ0x00 */
#define GUID 0x0000
#endif

/* ���ӱ�ǩ�Զ��� */
#if (GUID == 0x0001)
#elif (GUID == 0x0002)
#endif

#endif