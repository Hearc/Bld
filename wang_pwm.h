
/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_pwm.h
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

#ifndef __WANG_PWM_H_
#define __WANG_PWM_H_


#include "board.h"

#define	BloodLeak_YES       Chip_GPIO_WritePortBit(LPC_GPIO,2,10,1);
#define	BloodLeak_NO        Chip_GPIO_WritePortBit(LPC_GPIO,2,10,0);

#define LGT_LED_ON          Chip_GPIO_WritePortBit(LPC_GPIO,2,7,1);
#define LGT_LED_OFF         Chip_GPIO_WritePortBit(LPC_GPIO,2,7,0);

#define RED_LED_ON          Chip_GPIO_WritePortBit(LPC_GPIO,0,7,1);
#define RED_LED_OFF         Chip_GPIO_WritePortBit(LPC_GPIO,0,7,0);
#define RED_LED_Toggle      Chip_GPIO_SetPinToggle(LPC_GPIO, 0, 7);

#define GRN_LED_ON          Chip_GPIO_WritePortBit(LPC_GPIO,1,9,1);
#define GRN_LED_OFF         Chip_GPIO_WritePortBit(LPC_GPIO,1,9,0);
#define GRN_LED_Toggle      Chip_GPIO_SetPinToggle(LPC_GPIO, 1, 9);

void Timer16_0_Init (void);
void Timer32_1_Init (void);
void Timer32_0_Init (void);
void Timer16_1_Init(void);
void delay(uint32_t time);
void ADC_Read (uint32_t ulTime);


#endif

//END OF THE FILE

