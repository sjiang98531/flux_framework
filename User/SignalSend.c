/*====================================================================
			       珀菲电子技术部
		       版权所有(A)2005--2026
======================================================================
项    目:通用程序调试
产品型号:
文件  名:
原理  图:
芯    片:
校验  码:	
原始版本:V0.1
创建  人:mingtao_lu
创建日期:2024.04.15
描　  述:
======================================================================
修改时间:
版    本:V0.2
修改  人:mingtao_lu
描　  述:
====================================================================*/
#include "SignalSend.h"
//======================================//
Flag_Bits     Signal[3];              //数据结构体定义
//======================================//
unsigned char R_SignalSend_Data[10];  //发码数据区
unsigned char R_SignalSend_Bit;       //发码位码数
unsigned char R_SignalSend_Byte;      //发码字节数
unsigned char R_SignalSend_Step;      //发码步骤
unsigned char R_SignalSend_Dly;       //发码延时
unsigned char R_SignalSend_Num;       //发码周期计时
//======================================//
unsigned char R_SignalRec_Code[4];   //通信数据区
unsigned char R_SignalRec_Temp;      //通信缓存值
unsigned char R_SignalRec_Cnt;       //通信电平计时
unsigned char R_SignalRec_Bit;       //通信位数
unsigned char R_SignalRec_Num;       //通信计数
unsigned char R_SignalRec_Step;      //通信步骤

//====================================================================
//===        通信初始化函数          ===//
void Signal_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;//GPIO结构体,
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;   //通信口
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_PU;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}
//====================================================================
//===        遥控器数据装载函数      ===//
void SendData_Signal(unsigned char *tdata)
{
    unsigned char i;
    
    if((++ R_SignalSend_Num) > 24) //2.5ms*n=100ms发一次
    { 
        R_SignalSend_Num = 0;
        
        //if(!B_SignalSend_Send)
        {
            for(i = 0;i < 10;i ++,tdata++)
            {
                R_SignalSend_Data[i] = *tdata; 
            }
        
            R_SignalSend_Dly = 0;
            B_SignalSend_Send = 1;  //发送调试数据
        }
    }
}
//====================================================================
//===        遥控器发送处理函数      ===//
void IR_Send_Signal(void)
{	
    if(B_SignalSend_Send)  //发送状态
    {
        if(R_SignalSend_Dly != 0) // 等待一个电平周期发送完成 
        {
            R_SignalSend_Dly --;
            return;
        }
//========================================
        if(R_SignalSend_Step == 0) // 发送引导码 
        {
            if(B_SignalSend_Head == 0) // 引导码第一个电平 
            {
                B_SignalSend_Head = 1;
                OPEN_Signal();
                R_SignalSend_Dly = 15;//4.5ms
            }
            else if(B_SignalSend_Head == 1) // 引导码第二个电平 
            {
                B_SignalSend_Head = 0;
                CLOSE_Signal();
                R_SignalSend_Dly = 3;//4.5ms
                R_SignalSend_Step ++;
            }
        }
        else if(R_SignalSend_Step == 1) // 发送字节数据 
        {
            //if(((R_SignalSend_Data[R_SignalSend_Byte] << R_SignalSend_Bit) & 0x80) == 0)
            if(((R_SignalSend_Data[R_SignalSend_Byte] >> R_SignalSend_Bit) & 0x01) == 0)
            {
                if(B_SignalSend_Ing == 0) // 数据0的第一个电平 
                {
                    B_SignalSend_Ing = 1;
                    OPEN_Signal();
                    R_SignalSend_Dly = 3;
                }
                else if(B_SignalSend_Ing == 1) // 数据0的第二个电平 
                {
                    B_SignalSend_Ing = 0;
                    CLOSE_Signal();
                    R_SignalSend_Dly = 3;
                    R_SignalSend_Bit ++;
                }
            }
            else
            {
                if(B_SignalSend_Ing == 0) // 数据1的第一个电平 
                {
                    B_SignalSend_Ing = 1;
                    OPEN_Signal();
                    R_SignalSend_Dly = 3;
                }
                else if(B_SignalSend_Ing == 1) // 数据1的第二个电平 
                {
                    B_SignalSend_Ing = 0;
                    CLOSE_Signal();
                    R_SignalSend_Dly = 7;
                    R_SignalSend_Bit ++;
                }
            }
    
            if(R_SignalSend_Bit >= 8) // 发完一个字节 
            {
                R_SignalSend_Bit = 0;
                if((++ R_SignalSend_Byte) >= sizeof(R_SignalSend_Data)) // 发完buf 
                {
                    R_SignalSend_Byte = 0;
                    R_SignalSend_Step ++;
                }
            }
        }
        else if(R_SignalSend_Step == 2) // 发送字节数据 
        {
            OPEN_Signal();
            R_SignalSend_Dly = 3;
            R_SignalSend_Step ++;
        }
        else // 发送完成 
        {
            R_SignalSend_Step = 0; //清发送步骤
            B_SignalSend_Send = 0; //置发送状态置为空闲 
            B_SignalSend_Ing  = 0; //
            B_SignalSend_Head = 0; //
            R_SignalSend_Bit  = 0; //
            R_SignalSend_Byte = 0; //
            
            CLOSE_Signal();  //清数据线
            
            //asm("NOP"); 
    		//asm("NOP"); 
    		//asm("NOP"); 
    		//asm("NOP"); 
    		//asm("NOP"); 
    		//GCC_DELAY(2);//延时
    		
            //P01 = 1;
            //GPIO_ENABLE_INPUT(P0TRIS,GPIO_PIN_1);
            //GPIO_ENABLE_UP(P0UP,GPIO_PIN_1);
        }
    }
}
//====================================================================
//===         遥控接收函数           ===//
void Decode_Signal(void)
{
    //if(B_Send_Send) return;//发送时不解码
//========================================
    if(Read_PIN_Signal == Bit_RESET)// 获取IO引脚状态 
    {
        B_SignalRec_New = 0;
    }
    else
    {
        B_SignalRec_New = 1;
    }

    if(B_SignalRec_New == B_SignalRec_Old) // 同一电平计数 
    {
        if(R_SignalRec_Cnt < 160)  //20ms
        {
            R_SignalRec_Cnt ++;
        }
        else
        {
            B_SignalRec_Error = 1;
        }
    }
    else // 电平翻转 
    {
        if(B_SignalRec_Head == 0) // 接收头码 
        {
            if(B_SignalRec_Temp == 0)
            {   // 低电平6.0ms    ->5 + 48 - 5
                if((B_SignalRec_Old == 0) && (R_SignalRec_Cnt >= 43) && (R_SignalRec_Cnt <= 53))
                {
                    B_SignalRec_Temp = 1;
                }
            }
            else
            {   // 高电平(<=3.0ms) 因为电平翻转才往下执行，所以无需再判 old 
                if(R_SignalRec_Cnt <= 29) //24+5
                {
                    B_SignalRec_Temp = 0;
                    B_SignalRec_Head = 1;
                }
                else
                {
                    B_SignalRec_Error = 1;
                }
            }
        }
        else               // 接收字节数据 
        {
            if(B_SignalRec_Temp == 0) 
            {
                B_SignalRec_Temp = 1;
                if(R_SignalRec_Cnt >= 15) // 低电平1250us(<1.25ms) 
                {
                    B_SignalRec_Error = 1;
                }
            }
            else // 高电平 
            {
                B_SignalRec_Temp = 0;
                R_SignalRec_Temp >>= 1;
                if(R_SignalRec_Cnt < 5) // "0" -> 500us
                {
					R_SignalRec_Temp &= 0x7F;
                }
                else if(R_SignalRec_Cnt < 10) // "1" -> 1000us
                {
                    R_SignalRec_Temp |= 0x80;
                }
                else // error 
                {
                    B_SignalRec_Error = 1;
                }

                if((++ R_SignalRec_Bit) >= 8) // 收完一个字节 
                {
                    R_SignalRec_Bit = 0;
                    R_SignalRec_Code[R_SignalRec_Num] = R_SignalRec_Temp;
                    R_SignalRec_Temp = 0;
                    
                    if((++ R_SignalRec_Num) >= sizeof(R_SignalRec_Code)) // 收完buf 
                    {
                        R_SignalRec_Num = 0;
                        B_SignalRec_Head = 0;
                        B_SignalRec_Ok = 1;
                    }
                }
            }
        }

        // 备份与清零 
        B_SignalRec_Old = B_SignalRec_New;
        R_SignalRec_Cnt = 0;
    }

    // 故障处理 
    if(B_SignalRec_Error == 1)
    {
        B_SignalRec_Error = 0;
        B_SignalRec_Head = 0;
        B_SignalRec_Temp = 0;
        R_SignalRec_Temp = 0;
        R_SignalRec_Cnt  = 0;
        R_SignalRec_Bit  = 0;
        R_SignalRec_Num  = 0;
    }
}
//====================================================================
//===         遥控接收函数           ===//
/*void Decode_Signal(void)
{
    //if(B_Send_Send) return;//发送时不解码
//========================================
    if(Read_PIN_Signal == 0)// 获取IO引脚状态 
    {
        B_SignalRec_New = 0;
    }
    else
    {
        B_SignalRec_New = 1;
    }

    if(B_SignalRec_New == B_SignalRec_Old) // 同一电平计数 
    {
        if(R_SignalRec_Cnt < 80)
        {
            R_SignalRec_Cnt ++;
        }
        else
        {
            B_SignalRec_Error = 1;
        }
    }
    else // 电平翻转 
    {
        if(B_SignalRec_Head == 0) // 接收头码 
        {
            if(B_SignalRec_Temp == 0)
            {   // 低电平2ms    ->5 + 16 - 5
                if((B_SignalRec_Old == 0) && (R_SignalRec_Cnt >= 11) && (R_SignalRec_Cnt <= 21))
                {
                    B_SignalRec_Temp = 1;
                }
            }
            else
            {   // 高电平(<=0.5ms) 因为电平翻转才往下执行，所以无需再判 old 
                if(R_SignalRec_Cnt <= 9) //4
                {
                    B_SignalRec_Temp = 0;
                    B_SignalRec_Head = 1;
                }
                else
                {
                    B_SignalRec_Error = 1;
                }
            }
        }
        else               // 接收字节数据 
        {
            if(B_SignalRec_Temp == 0) 
            {
                B_SignalRec_Temp = 1;
                if(R_SignalRec_Cnt >= 21) // 低电平2000us(<2ms) 
                {
                    B_SignalRec_Error = 1;
                }
            }
            else // 高电平 
            {
                B_SignalRec_Temp = 0;
                R_SignalRec_Temp >>= 1;
                if(R_SignalRec_Cnt < 6) // "0" 
                {
					R_SignalRec_Temp &= 0x7F;
                }
                else if(R_SignalRec_Cnt < 12) // "1" 
                {
                    R_SignalRec_Temp |= 0x80;
                }
                else // error 
                {
                    B_SignalRec_Error = 1;
                }

                if((++ R_SignalRec_Bit) >= 8) // 收完一个字节 
                {
                    R_SignalRec_Bit = 0;
                    R_SignalRec_Code[R_SignalRec_Num] = R_SignalRec_Temp;
                    R_SignalRec_Temp = 0;
                    
                    if((++ R_SignalRec_Num) >= sizeof(R_SignalRec_Code)) // 收完buf 
                    {
                        R_SignalRec_Num = 0;
                        B_SignalRec_Head = 0;
                        B_SignalRec_Ok = 1;
                    }
                }
            }
        }

        // 备份与清零 
        B_SignalRec_Old = B_SignalRec_New;
        R_SignalRec_Cnt = 0;
    }

    // 故障处理 
    if(B_SignalRec_Error == 1)
    {
        B_SignalRec_Error = 0;
        B_SignalRec_Head = 0;
        B_SignalRec_Temp = 0;
        R_SignalRec_Temp = 0;
        R_SignalRec_Cnt  = 0;
        R_SignalRec_Bit  = 0;
        R_SignalRec_Num  = 0;
    }
}*/

//====================================================================
//===           程序结束             ===//
//====================================================================
