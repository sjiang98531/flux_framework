//#define  __COMM_DEFINE_      //用于区别定义与声明

//#include "sc32_conf.h"
////#include "stdint_h"
//#include "Comm.h"
//#include "includes.h"



////20-380w  30-120 1->4W     30--->20w+n*4w
//const uint16_t SpeedTable[] = {0, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
////                    //功率  

/////**************************************************************
////*函数名称：void CommIntial(void)
////*函数功能：通讯初始化函数
////***************************************************************/
//void CommIntial(void)
//{
//    f_RxRequest = 0;
//    f_RxLow = 0;
//    f_TxHead = 0;
//    f_TxTail = 0;
//    CommPcs = 0;
//    CommByte = 0;
//    CommBit = 0;
//}


//  uint16_t  mcActualSpeedFlt = 0;
/////**************************************************************
////*函数名称：void Comm(void)
////*函数功能：通讯部分处理
////***************************************************************/
//void Comm(void)
//{
////    static uint8_t   RcvDataLast = 0;
//	 static uint8_t   Flag10msCnt = 0;
//   static uint16_t  mcActualSpeedCnt = 0;
//   static uint32_t mcActualSpeedSum = 0;
//   uint8_t TempCommRxDat = 0;

//    if (f_RxFin == 1)     //已接受完成
//    {
//      f_RxFin = 0;
//      TxDelay = 0;
////      SleepSet.SleepDelayCout = 0;
//        TempCommRxDat = ~CommRxDat[1];
//      if (CommRxDat[0] == TempCommRxDat)
//      {
////          #if  (!TEST_DEBUG)
////              #if (!DUTY_TEST)
//                  if (CommRxDat[0] <= 20)
//                  {
//                      WindDuty = SpeedTable[CommRxDat[0]];
//                      OldCommMode = 1; 
//                  }
//                  else if (CommRxDat[0] <= 120)
//                  {
//                      WindDuty = CommRxDat[0];
//                      OldCommMode = 0;
//                  }
//                  else
//                  {
//                      WindDuty = 0;
//                  }
////              #else
////                  WindDuty = CommRxDat[0];
////              #endif
//          
//          if (WindDuty >= 120)  WindDuty = 120;
////          #endif
//      }
//      RcvDataLast = CommRxDat[0];
//    }
//    
//    if (!f_RxRequest)
//    {
//        if (TxDelay < 0xfff0) TxDelay++;
//        if (TxDelay == 20)   
//        {
//            f_RxRequest = 1;
//            f_TxHead = 0;
//            f_TxTail = 0;
//            CommPcs = 0;
//            CommByte = 0;
//            CommBit = 0;
//        }
//        else if (TxDelay > 10000)
//        {
//            TxDelay  = 0;
//            f_RxRequest = 0;
//            CommPcs = 0;
//            CommByte = 0;
//            CommBit = 0;
//            f_RxLow = 0;
//        } 
//    }

//    if (++Flag10msCnt >= 5)
//    {
//        Flag10msCnt = 0;
//        mcActualSpeedCnt++;
//        mcActualSpeedSum += M1.sUserLoop.iMachineSpeed; //机械转速
//        if (mcActualSpeedCnt >= 64)
//        {
//            mcActualSpeedFlt = mcActualSpeedSum >> 6;
//            mcActualSpeedSum = 0;
//            mcActualSpeedCnt = 0;
//        }
//        
//    }
// }
///****************************************************************
//	函数名：WindDutyToPowerValue
//	描述：  风量转化为功率给定
//	输入：  无
//	输出：  无
//****************************************************************/
//void WindDutyToPowerValue(SOC_MOT* Mx)
//{
//    if (WindDuty < 30)
//    {
//        M1.sMC.bRunFlag = 0;   /* 电机运行标志 */
//    }
//    else if (WindDuty <= 120)
//    {
//        if (M1.sFaultDet.SysFaultFlag.iByte == 0x00)
//        {
//            M1.sMC.bRunFlag = 1;   /* 电机运行标志 */
//            Mx->sUserLoop.sTargetRamp.iDest = (20 + (WindDuty-30)*4)<<4; //Mx->sUserLoop.InputPower  M1.sUserLoop.sTargetRamp.iDest  
//            
//            if (Mx->sUserLoop.sTargetRamp.iDest > MaxInputPower_Limit)
//            {
//                Mx->sUserLoop.sTargetRamp.iDest = MaxInputPower_Limit;
//            }
//            else if (Mx->sUserLoop.sTargetRamp.iDest < (20<<4))
//            {
//                Mx->sUserLoop.sTargetRamp.iDest = (20<<4);
//            }
//            
//        }
//        else
//        {
//            M1.sMC.bRunFlag = 0;   /* 电机运行标志 */
//            Mx->sUserLoop.sTargetRamp.iDest = 0;
//        }
//    }
//    else
//    {
//        M1.sMC.bRunFlag = 0;   /* 电机运行标志 */
//    }
//}    
// 

//void TxDataHandle(void)
//{
//    static uint16_t DataChangeTx = 0;

//    uint16_t temp;

//    if(flagSendID == 0xff)
//    {
//      DataChangeTx++;

//      if(M1.sMC.eStatus == RUN)
//      {
//         if (DataChangeTx & 0x01)
//         {
//             CommTxDat[0] 	= 0x03;
//             temp = ((uint16_t)mcActualSpeedFlt)>>3;
//             if (temp > 254) temp = 254;
//             CommTxDat[1] 	= (uint8_t)temp;
//         }
//         else
//         {
//             CommTxDat[0] 	= 0x04;
////             temp = mcFocCtrl.NTCTempFlt>>7;
//             temp = 0;
//             if (temp > 254) temp = 254;
//             CommTxDat[1] 	= (uint8_t)temp;
//         }
//         
//      }
//  		else
//  		{			 
//  	      if (DataChangeTx & 0x01)
//          {
//            CommTxDat[0] 	= 0x06;
//            switch(M1.sFaultDet.SysFaultFlag.iByte)
//            {
//                    case FaultNoSource:					
//                            CommTxDat[1] 	= 0x00;			
//                    break;
//                    case FaultStall:
//                            CommTxDat[1] 	= (1<<5);				
//                        break;	
//                    case FaultSoftOVCurrent:
//                            CommTxDat[1] 	= (1<<2);
//                        break;
//                    case FaultHardOVCurrent:
//                            CommTxDat[1] 	= (1<<2);				
//                        break;
//                    case FaultOverVoltageDC:
//                            CommTxDat[1] 	= (1<<4);	
//                        break;
//                    case FaultUnderVoltageDC:
//                            CommTxDat[1] 	= (1<<6);	
//                        break;
//                    case  FaultPhaseLost:
//                            CommTxDat[1] 	= (1<<1);
//                        break;
//                    case FaultVTH:
//                            CommTxDat[1] 	= (1<<3);
//                        break;
//                    default:
//                        break;
//            } 
//          }
//          else
//          {
//              CommTxDat[0] 	= 0x04;
////             temp = mcFocCtrl.NTCTempFlt>>7;
//              temp = 0;
//             if (temp > 254) temp = 254;
//             CommTxDat[1] 	= (uint8_t)temp;
//          }  
//      }
//      CommTxDat[2] = CommTxDat[0]+CommTxDat[1];
//   }
//   else
//   {
//       switch (flagSendID)
//       {
//          case 0:
//              CommTxDat[0] = 0x09;
//              CommTxDat[1] = (unsigned char)(PROGRAM_CODE_NUME/256);
//              CommTxDat[2] = CommTxDat[0]+CommTxDat[1];
//              flagSendID++;
//            break;
//          case 1:
//              CommTxDat[0] = 0x0A;
//              CommTxDat[1] = (unsigned char)(PROGRAM_CODE_NUME%256);
//              CommTxDat[2] = CommTxDat[0]+CommTxDat[1];
//              flagSendID++; 
//            break;
//          case 2:
//              CommTxDat[0] = 0x0B;
//              CommTxDat[1] = (unsigned char)(PROGRAM_CODE_VERSION);
//              CommTxDat[2] = CommTxDat[0]+CommTxDat[1];
//              flagSendID = 0x00;
//              flagSendCnt++;
//              if (flagSendCnt >= 3)
//              {
//                  flagSendCnt = 0;
//                  flagSendID = 0xff;
//              }
//            break;
//          default:
//              flagSendID = 0xff;
//            break;
//       }      
//    }
//}

////16K - 125us
//void RxTxComm(void)
//{
//		uint8_t j;

//		if (f_RxRequest)  //send
//    {
//        if (!f_TxHead)
//        {
//						if(CommPcs < 0xfff0) { CommPcs++;}
//						if (CommPcs <= 36*2){  CLR_COMM_TX;}
//            else if (CommPcs < 54*2) {SET_COMM_TX;}
//            else
//            {
//                f_TxHead = 1;
//                f_TxTail = 0;
//                CommPcs = 0;
//                CommByte = 0;
//                CommBit = 0;
//                TxDataHandle();
//            } 
//        }
//        else if (!f_TxTail)
//        {
//            if(CommTxDat[CommByte] & 0x01)  j=12*2;
//            else j=4*2;
//            if(CommPcs < 0xfff0)  CommPcs++;
//            if (CommPcs <= j) {CLR_COMM_TX;}
//            else if (CommPcs < j+6) { SET_COMM_TX;}
//            else
//            {
//                CommPcs = 0;
//                if (++CommBit >= 24)
//                {
//                    f_TxTail = 1;  
//                }
//                else
//                {
//                    if (!(CommBit&0x07))  CommByte++;
//                    else CommTxDat[CommByte]>>=1;
//                }
//            }
//        }
//        else if (f_TxTail)
//        {
//            if(CommPcs < 0xfff0)  CommPcs++;
//            if (CommPcs < 16) 
//            {
//                CLR_COMM_TX;
//            }
//            else
//            {
//                f_RxRequest = 0;
//                f_TxTail = 0;
//                f_TxHead = 0;
//                CommPcs = 0;
//                CommByte = 0;
//                CommBit = 0;
//                SET_COMM_TX;
//            }
//        } 
//    }
//    else //recieve
//		{
//        if (CommPcs < 0xfff0) CommPcs++;
//				if (!RX_PIN)
//				{
//						if (f_RxLow)
//						{
//								if (!f_RxHead)
//                {
//										if ((CommPcs > 48*2)&&(CommPcs < 60*2))
//										{
//												f_RxHead = 1;
//										}
//										else
//										{
//												f_RxHead = 0;
//										}
//                    CommByte = 0;
//                    CommBit = 0;
//								}
//                else
//                {
//                    CommRxDat[CommByte] >>= 1;
//										if (CommPcs < 10*2)   CommRxDat[CommByte] |= 0x00;
//                    else if (CommPcs < 22*2) CommRxDat[CommByte] |= 0x80;
//                    else
//                    {
//                        f_RxHead = 0;
//												CommByte = 0;
//												CommBit = 0;
//                    }
//                    if (++CommBit >= 16)
//                    {
//                        f_RxFin = 1;
//                        f_RxHead = 0;
//												CommByte = 0;
//												CommBit = 0;
//                    }
//                    else
//                    {
//                        if (!(CommBit&0x07))  CommByte++;
//                    }
//                }
//                CommPcs = 0;
//                f_RxLow = 0;
//						}  
//				}
//				else
//				{
//						f_RxLow = 1;
//						
//				}
//    }
//}



