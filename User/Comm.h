#ifndef __Comm_H_
#define __Comm_H_

//用于防止定义与声明混淆 
#ifdef __COMM_DEFINE_ 
#define COMM_EXTERN    
#else
#define COMM_EXTERN extern
#endif

#include "sc32_conf.h"
#include "includes.h"

//******************  接收发送引脚宏定义   *******************
#define	REC_PIN        GPIO_Pin_1
#define REC_PORT       GPIOC 

#define SEND_PIN       GPIO_Pin_2                    
#define SEND_PORT      GPIOC

#define CLR_COMM_TX     PC_BIT(2) = 0;
#define SET_COMM_TX     PC_BIT(2) = 1;

#define RX_PIN          PC_BIT(1)

#define PROGRAM_CODE_NUME     01
#define PROGRAM_CODE_VERSION  00


#define FaultNoSource        0
#define FaultStall           2048
#define FaultSoftOVCurrent   2  //过流都定义的是0x04
#define FaultHardOVCurrent   1
#define FaultOverVoltageDC   256    
#define FaultUnderVoltageDC  512
#define FaultPhaseLost       4096
#define FaultVTH             16  //过温

//*******************   变量定义  ***********************
COMM_EXTERN uint8_t f_RxRequest;
COMM_EXTERN uint8_t f_TxHead;
COMM_EXTERN uint8_t f_TxTail;
COMM_EXTERN uint8_t f_RxLow;
COMM_EXTERN uint8_t f_RxHead;
COMM_EXTERN uint8_t f_RxFin;
COMM_EXTERN uint8_t f_TxRequest;

COMM_EXTERN uint8_t CommTxDat[5];
COMM_EXTERN uint8_t CommRxDat[5];
COMM_EXTERN uint16_t CommPcs;
COMM_EXTERN uint8_t CommBit;
COMM_EXTERN uint8_t CommByte;
COMM_EXTERN uint8_t CommTime;
COMM_EXTERN uint16_t TxDelay;
COMM_EXTERN uint8_t RcvDataLast;

COMM_EXTERN uint8_t OldCommMode;



COMM_EXTERN unsigned char flagSendID;  
COMM_EXTERN unsigned char flagSendCnt; 
COMM_EXTERN unsigned char WindDuty;
COMM_EXTERN unsigned char mcFaultSourceAlt;




//*****************   函数声明   ************************
extern void Comm(void);
extern void CommIntial(void);
extern void TxDataHandle(void);
extern void RxTxComm(void);
extern void WindDutyToPowerValue(SOC_MOT* Mx);

#endif
