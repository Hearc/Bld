
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
**	���     | BYTE1 | BYTE2 | BYTE3 |BYTE4	| BYTE5 | BYTE6 | BYTE7 | BYTE8 |
**	1        | 0xA0  | ״̬  | �Լ���     |      �����ʶ BLD_ERR1        |
**	2        | 0xB0  | ״̬  | Ӳ��  | ��� |      �ϵ��ʱ��               |
**	3        | 0xB1  | ״̬  | ����������� |               |               |
**
****************************************************************************/
uint8_t slave_data[3][8] = {{0xA0,0,0,0,0,0,0,0},
                            {0xB0,0,0,0,0,0,0,0},
                            {0xB1,0,0,0,0,0,0,0}};

uint16_t CmpCounter;

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        16λ��ʱ��0��ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer16_0_Init (void)
{
    //����PWM����
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_8, (IOCON_FUNC2 | IOCON_MODE_INACT)); /* CT16B0_MAT0 */
//	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_9, (IOCON_FUNC2 | IOCON_MODE_INACT)); /* CT16B0_MAT1 */

    Chip_TIMER_Init(LPC_TIMER16_0);                                     /* �򿪶�ʱ��ģ��               */

    LPC_TIMER16_0->TCR     = 0x02;                                      /* ��ʱ����λ                   */
    LPC_TIMER16_0->PR      = 0;                                         /* ���÷�Ƶϵ��                 */
    LPC_TIMER16_0->PWMC    = 0x03;                                      /* ����MAT0��1 PWM���          */
    LPC_TIMER16_0->MCR     = 0x02<<9;                                   /* ����MR3ƥ���λTC,PWM����  */
    LPC_TIMER16_0->MR[3]     = SystemCoreClock / 10000;                 /* ���ڿ��ƣ�1��                */
    LPC_TIMER16_0->MR[0]     = LPC_TIMER16_0->MR[3]/2;                  /* MAT0���50%����              */
//    LPC_TIMER16_0->MR[1]     = LPC_TIMER16_0->MR[3]/4;                /* MAT1���75%����              */
    LPC_TIMER16_0->TCR     = 0x01;                                      /* ������ʱ��                   */
}

/*********************************************************************************************************
** Function name:       timer1Init
** Descriptions:        32λ��ʱ��1��ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
** P1_1���2MHz����
*********************************************************************************************************/
void Timer32_1_Init (void)
{
    //����PWM����
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_1, (IOCON_FUNC3 | IOCON_MODE_INACT)); /* CT32B1_MAT0 */

    Chip_TIMER_Init(LPC_TIMER32_1);                                     /* �򿪶�ʱ��ģ��               */
//		Chip_TIMER_Reset(LPC_TIMER32_1);

    LPC_TIMER32_1->TCR     = 0x02;                                      /* ��ʱ����λ                   */
    LPC_TIMER32_1->PR      = 0;                                         /* ���÷�Ƶϵ��                 */
    LPC_TIMER32_1->PWMC    = 0x01;                                      /* ����MAT0��1PWM���           */
    LPC_TIMER32_1->MCR     = 0x02<<9;                                   /* ����MR3ƥ���λTC,PWM����  */
    LPC_TIMER32_1->MR[3]     = Chip_Clock_GetSystemClockRate() /2143000;    /* ���ڿ��ƣ�1��                */
    LPC_TIMER32_1->MR[0]     = LPC_TIMER32_1->MR[3]/2;                      /* MAT0���50%����              */
//    Chip_TIMER_SetMatch(LPC_TIMER32_1, 1, (Chip_Clock_GetSystemClockRate() / 1000000));
//    LPC_TIMER32_1->MR[1]     = LPC_TIMER32_1->MR[3]/4;                    /* MAT1���75%����              */
    LPC_TIMER32_1->TCR     = 0x01;                                          /* ������ʱ��                   */
}

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        32λ��ʱ��1��ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer32_0_Init(void)
{
//    uint32_t timerFreq;
    //����PIO1_5 ��������
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_5, (IOCON_FUNC2 | IOCON_MODE_INACT));
    Chip_TIMER_Init(LPC_TIMER32_0);                             // Enable timer 1 clock 

//  Chip_TIMER_CaptureRisingEdgeEnable(LPC_TIMER32_0,5);
    LPC_TIMER32_0->CCR     = 0x05;                              /* ����CAP0.0�½��ز����ж�     */
  
    NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);                      // Enable timer interrupt
    NVIC_EnableIRQ(TIMER_32_0_IRQn);
//	NVIC_SetPriority(TIMER_32_0_IRQn, 2);                       //�����ж����ȼ�
}

/*********************************************************************************************************
** Function name:       timer0Init
** Descriptions:        16λ��ʱ��1��ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer16_1_Init(void)
{
    uint32_t timerFreq;

    Chip_TIMER_Init(LPC_TIMER16_1);                             // Enable timer 1 clock 
    timerFreq = Chip_Clock_GetSystemClockRate();                // Timer rate is system clock rate 
    Chip_TIMER_Reset(LPC_TIMER16_1);                            // Timer setup for match and interrupt at TICKRATE_HZ
    Chip_TIMER_MatchEnableInt(LPC_TIMER16_1, 1U);

    Chip_TIMER_SetMatch(LPC_TIMER16_1, 1U, (timerFreq / 10000));        //1ms�ж� 10KHz

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
	LPC_TIMER32_0->IR  = 1 << 4;		//���CAP0�жϱ�־
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
** Descriptions:        AD��ֵ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void ADC_Read (uint32_t ulTime)
{
    uint16_t dataADC;

    /* Start A/D conversion ����ADCת��*/
    Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
    
    /* Waiting for A/D conversion complete�ȴ�ת����� */
    while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH0, ADC_DR_DONE_STAT) != SET) {}

    /* Read ADC value */
    Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &dataADC);
    
    if(dataADC > 100)
    {
        CmpCounter++;
    }
}

//  END OF THE FILE
