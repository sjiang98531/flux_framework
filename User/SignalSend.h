#ifndef __SIGNALSEND_H
#define __SIGNALSEND_H

#include "sc32_conf.h"
#include "BinToHex.h"

#define  Out_PIN_SignalH       {PC_BIT(2) = 1;}
#define  Out_PIN_SignalL       {PC_BIT(2) = 0;}

#define  Read_PIN_Signal        GPIO_ReadDataBit(GPIOC,GPIO_Pin_2)

#define  CLOSE_Signal()        {Out_PIN_SignalH;} //1
#define  OPEN_Signal()         {Out_PIN_SignalL;} //0

extern   Flag_Bits              Signal[3];
//========================================
#define  B_SignalgRec_Flag      Signal[0].Byte    //标志寄存器
#define  B_SignalRec_Ok         Signal[0].Bits.B0 //接收完成
#define  B_SignalRec_Head       Signal[0].Bits.B1 //接收头码正确
#define  B_SignalRec_New        Signal[0].Bits.B2 //接收进行中
#define  B_SignalRec_Old        Signal[0].Bits.B3 //接收使能
#define  B_SignalRec_Temp       Signal[0].Bits.B4 //接收
#define  B_SignalRec_Error      Signal[0].Bits.B5 //错误标志位
//========================================
#define  B_SignalSend_Flag      Signal[1].Byte    //标志寄存器
#define  B_SignalSend_Head      Signal[1].Bits.B0 //发送数据头
#define  B_SignalSend_Ing       Signal[1].Bits.B1 //发送数据
#define  B_SignalSend_Send      Signal[1].Bits.B2 //发送标志

//========================================
#define  B_Motor_Flag           Signal[2].Byte    //标志寄存器
#define  B_OnF                  Signal[2].Bits.B0 //开关机标志位
#define  B_Out_Osc              Signal[2].Bits.B1 //上下摇头标志位
#define  B_Out_OscUD            Signal[2].Bits.B2 //上下摇头标志位
//========================================

extern unsigned char R_SignalRec_Code[4];  //红外码区

void Signal_Config(void);
void Decode_Signal(void);
void IR_Send_Signal(void);
void SendData_Signal(unsigned char *tdata);

#endif
