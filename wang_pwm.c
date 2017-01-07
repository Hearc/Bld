
/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_pwm.c
* Compiler :     
* Revision :     Revision
* Date :         2017-01-07
* Updated by :   
* Description :  
*                
* 
* LPC11C14 sytem clock: 48Mhz
* system clock: 48MHz
********************************************************************************
* This edition can only be used in DCBoard V6.0 using MornSun DC/DC Module
*******************************************************************************/

#include "wang_pwm.h"
uint8_t data[3];

/****************************************************************************
**	序号     | BYTE1 | BYTE2 | BYTE3 |BYTE4	| BYTE5 | BYTE6 | BYTE7 | BYTE8 |
**	1        | 0xA0  | 状态  | 自检结果     |      错误标识 BLD_ERR1        |
**	2        | 0xB0  | 状态  | 硬件  | 软件 |      上电后时间               |
**	3        | 0xB1  | 状态  | 检测器灵敏度 |               |               |
**
****************************************************************************/
uint8_t slave_data[3][8] = {{0xA0,0,0,0,0,0,0,0},
                            {0xB0,0,0,0,0,0,0,0},
                            {0xB1,0,0,0,0,0,0,0}};

uint16_t CmpCounter;

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        16位定时器0初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer16_0_Init (void)
{
    //配置PWM引脚
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_8, (IOCON_FUNC2 | IOCON_MODE_INACT)); /* CT16B0_MAT0 */
//	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_9, (IOCON_FUNC2 | IOCON_MODE_INACT)); /* CT16B0_MAT1 */

    Chip_TIMER_Init(LPC_TIMER16_0);                                     /* 打开定时器模块               */

    LPC_TIMER16_0->TCR     = 0x02;                                      /* 定时器复位                   */
    LPC_TIMER16_0->PR      = 0;                                         /* 设置分频系数                 */
    LPC_TIMER16_0->PWMC    = 0x03;                                      /* 设置MAT0，1 PWM输出          */
    LPC_TIMER16_0->MCR     = 0x02<<9;                                   /* 设置MR3匹配后复位TC,PWM周期  */
    LPC_TIMER16_0->MR[3]     = SystemCoreClock / 10000;                 /* 周期控制，1秒                */
    LPC_TIMER16_0->MR[0]     = LPC_TIMER16_0->MR[3]/2;                  /* MAT0输出50%方波              */
//    LPC_TIMER16_0->MR[1]     = LPC_TIMER16_0->MR[3]/4;                /* MAT1输出75%方波              */
    LPC_TIMER16_0->TCR     = 0x01;                                      /* 启动定时器                   */
}

/*********************************************************************************************************
** Function name:       timer1Init
** Descriptions:        32位定时器1初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
** P1_1输出2MHz方波
*********************************************************************************************************/
void Timer32_1_Init (void)
{
    //配置PWM引脚
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_1, (IOCON_FUNC3 | IOCON_MODE_INACT)); /* CT32B1_MAT0 */

    Chip_TIMER_Init(LPC_TIMER32_1);                                     /* 打开定时器模块               */
//		Chip_TIMER_Reset(LPC_TIMER32_1);

    LPC_TIMER32_1->TCR     = 0x02;                                      /* 定时器复位                   */
    LPC_TIMER32_1->PR      = 0;                                         /* 设置分频系数                 */
    LPC_TIMER32_1->PWMC    = 0x01;                                      /* 设置MAT0，1PWM输出           */
    LPC_TIMER32_1->MCR     = 0x02<<9;                                   /* 设置MR3匹配后复位TC,PWM周期  */
    LPC_TIMER32_1->MR[3]     = Chip_Clock_GetSystemClockRate() /2143000;    /* 周期控制，1秒                */
    LPC_TIMER32_1->MR[0]     = LPC_TIMER32_1->MR[3]/2;                      /* MAT0输出50%方波              */
//    Chip_TIMER_SetMatch(LPC_TIMER32_1, 1, (Chip_Clock_GetSystemClockRate() / 1000000));
//    LPC_TIMER32_1->MR[1]     = LPC_TIMER32_1->MR[3]/4;                    /* MAT1输出75%方波              */
    LPC_TIMER32_1->TCR     = 0x01;                                          /* 启动定时器                   */
}

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        32位定时器1初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer32_0_Init(void)
{
//    uint32_t timerFreq;
    //配置PIO1_5 捕获引脚
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_5, (IOCON_FUNC2 | IOCON_MODE_INACT));
    Chip_TIMER_Init(LPC_TIMER32_0);                             // Enable timer 1 clock 

//  Chip_TIMER_CaptureRisingEdgeEnable(LPC_TIMER32_0,5);
    LPC_TIMER32_0->CCR     = 0x05;                              /* 设置CAP0.0下降沿捕获中断     */
  
    NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);                      // Enable timer interrupt
    NVIC_EnableIRQ(TIMER_32_0_IRQn);
//	NVIC_SetPriority(TIMER_32_0_IRQn, 2);                       //设置中断优先级
}

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        16位定时器1初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer16_1_Init(void)
{
    uint32_t timerFreq;

    Chip_TIMER_Init(LPC_TIMER16_1);                             // Enable timer 1 clock 
    timerFreq = Chip_Clock_GetSystemClockRate();                // Timer rate is system clock rate 
    Chip_TIMER_Reset(LPC_TIMER16_1);                            // Timer setup for match and interrupt at TICKRATE_HZ
    Chip_TIMER_MatchEnableInt(LPC_TIMER16_1, 1U);

    Chip_TIMER_SetMatch(LPC_TIMER16_1, 1U, (timerFreq / 10000));        //1ms中断 10KHz

    Chip_TIMER_ResetOnMatchEnable(LPC_TIMER16_1, 1U);
    __NOP(); __NOP(); __NOP();

    Chip_TIMER_Enable(LPC_TIMER16_1);
    NVIC_ClearPendingIRQ(TIMER_16_1_IRQn);                              // Enable timer interrupt
    NVIC_EnableIRQ(TIMER_16_1_IRQn);
}
///**********************************************************************
// * @brief	  Handle interrupt from 32-bit timer
// * @return	Nothing
// * @note	  used for 
// * @author  FENG XQ
// * @date    2016-08-13
// *********************************************************************/
/*
void TIMER32_0_IRQHandler(void)
{
	LPC_TIMER32_0->IR  = 1 << 4;		//清除CAP0中断标志
	ICPcnt++;
}
*/

void TIMER16_1_IRQHandler(void)
{
    if (Chip_TIMER_MatchPending(LPC_TIMER16_1, 1U))
    {
        Chip_GPIO_SetPinToggle(LPC_GPIO, 2, 1);
    }
}

void delay(uint32_t time)
{
    uint32_t i;
    while(time--)
    {
        for(i = 0;i < 11 ;i++);
    }
}
/*********************************************************************************************************
** Function name:       ADC_Read
** Descriptions:        AD采值
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void ADC_Read (uint32_t ulTime)
{
    uint16_t dataADC;

    /* Start A/D conversion 开启ADC转换*/
    Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
    
    /* Waiting for A/D conversion complete等待转换完成 */
    while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH0, ADC_DR_DONE_STAT) != SET) {}

    /* Read ADC value */
    Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &dataADC);
    
    if(dataADC > 100)
    {
        CmpCounter++;
    }
}

//  END OF THE FILE
