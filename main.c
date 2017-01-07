/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          main.c
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

// LPC11C24: Pin17=VDD_CAN  Pin18=CANL   Pin19=CANH   Pin20=VCC    Pin21=GND    Pin22=STB    Pin23=PIO0-6
// LPC11C14: Pin17=PIO1-9   Pin18=PIO2-4 Pin19=CAN-RX Pin20=CAN-TX Pin21=PIO2-5 Pin22=PIO0-6 Pin203=PIO0-7
// LED = PIO2-7
// TinyM0-CAN: UART JP4  PIO1_6 = RXD   PIO1_7 = TXD

/**********************************************************************
 * @brief	main.c for lpc11c14 CAN communication
 * @return	Nothing
 * @note	Th.
 * @author  FENG XQ
 * @date    2016-08-10
 * 
 * update: 2016-08-18
 * 修改发送报文对象编号为 1, 增加发送成功判断标志
 *
 *msg rx counter wrong:solution?
 *********************************************************************/
 
#include "wang_pwm.h"
#include "feng_can.h"
//#include "wang_iap.h"

#define TIMER_TICKRATE_IN_HZ    10UL                    //
#define SYSTICK_RATE_IN_HZ      3000UL                  //
#define LeakCoefficient         (12)                    //12=1.2

#define IAP_ENTER_ADR           0x1FFF1FF1              /*  IAP入口地址定义             */

/* 
 *  定义IAP命令字符
 */                                     
#define IAP_Prepare             50      //准备编程扇区              
#define IAP_RAMTOFLASH          51      //将RAM内容赋值到Flash      
#define IAP_ERASESECTOR         52      //擦除扇区                  
#define IAP_BLANKCHK            53      //扇区查空                  
#define IAP_READPARTID          54      //读取器件ID         
#define IAP_BOOTCODEID          55      //读取Boot代码版本   
#define IAP_COMPARE             56      //比较       

/*
 *  定义IAP返回状态字
 */
#define CMD_SUCCESS                                0
#define INVALID_COMMAND                            1
#define SRC_ADDR_ERROR                             2
#define DST_ADDR_ERROR                             3
#define SRC_ADDR_NOT_MAPPED                        4
#define DST_ADDR_NOT_MAPPED                        5
#define COUNT_ERROR                                6
#define INVALID_SECTOR                             7
#define SECTOR_NOT_BLANK                           8
#define SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION    9
#define COMPARE_ERROR                              10
#define BUSY                                       11

/* 
 *  定义CCLK值大小，单位为KHz 
 */
#define IAP_FCCLK            (24000)

/*
 *  定义函数指针  
 */
void (*IAP_Entry) (uint32_t ulParam_tab[], uint32_t ulPesult_tab[]);
void Leak_Calibration(void);
uint32_t  GulParamin[5];     /*  IAP入口参数缓冲区          */        
uint32_t  GulParamout[4];    /*  IAP出口参数缓冲区          */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

volatile unsigned int	ICPcnt = 0;
volatile unsigned int	LightICP = 0;
volatile unsigned int	DarkICP = 0;

volatile unsigned int ICPLimitMax = 15000;  
volatile unsigned int ICPLimitMin = 500;    
volatile unsigned int CalibrationValue = 0, CalibrationBuf[5] = {0}, CaliTemp = 0;

//BloodLeakFlage = 1  ??
//BloodLeakFlage = 0  ??
unsigned char BloodLeakFlage = 0;
unsigned char ICPUpdateReady = 0;

//LeakThreshold = (CalibrationValue * LeakCoefficient)/10;
//(LightICP - DarkICP) < this value means Leak present
volatile unsigned int LeakThreshold = 0;
//CalibrationFlag 1: 先定标
//CalibrationFlag 0: 正常程序
volatile unsigned char CalibrationFlag = 0, CalibrationCnt = 0;

volatile uint8_t initialized;
uint32_t USecPerOverflow = 65536 * 2;
volatile uint16_t count_time;

//static ADC_CLOCK_SETUP_T ADCSetup;

volatile uint8_t USI_TWI_buffer[12];
volatile uint8_t RxData[8];
unsigned char ModeTest = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
void delay_ms(uint16_t time);
void uartread(void);
/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
#define    UART_BPS         115200                                        /* 串口通信波特率               */
/*********************************************************************************************************
  全局变量
*********************************************************************************************************/
volatile    uint8_t          GucRcvNew;                                  /* 串口接收新数据的标志         */
uint8_t     GucRcvBuf[10] ;                                              /* 串口接收数据缓冲区           */
uint32_t    GulNum;                                                      /* 串口接收数据的个数           */

//主机运行状态	
uint8_t masterstate;
uint32_t movetime;          //主机运行时间
uint8_t check;              //00 切换到自检状态 01 下一步 02 系统维护	
uint8_t checkstart = 0;
uint8_t TxBuff[8];
uint8_t RxBuff[8];

uint8_t	hardwareversion = 0x05,softwareversion = 0x05;
/*********************************************************************************************************
** Function name:       uartSendByte
** Descriptions:        向串口发送子节数据，并等待数据发送完成，使用查询方式
** input parameters:    ucDat:   要发送的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartSendByte (uint8_t ucDat)
{
    LPC_USART->THR = ucDat;                                              /*  写入数据                    */
    while ((LPC_USART->LSR & 0x40) == 0);                                /*  等待数据发送完毕            */
}

/*********************************************************************************************************
** Function name:       uartSendStr
** Descriptions:        向串口发送字符串
** input parameters:    pucStr:  要发送的字符串指针
**                      ulNum:   要发送的数据个数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartSendStr (uint8_t const *pucStr, uint32_t ulNum)
{
    uint32_t i;

    for (i = 0; i < ulNum; i++)
    {                                                                   /* 发送指定个字节数据           */
        uartSendByte (*pucStr++);
    }
}
/*********************************************************************************************************
* Function Name:        UART_IRQHandler
* Description:          UART中断服务函数
* Input:                无
* Output:               无
* Return:               无
*********************************************************************************************************/
void UART_IRQHandler (void)
{
    GulNum = 0;

    while ((LPC_USART->IIR & 0x01) == 0)
    {                                                                   /*  判断是否有中断挂起          */
        switch (LPC_USART->IIR & 0x0E)
        {                                                               /*  判断中断标志                */
        
            case 0x04:                                                  /*  接收数据中断                */
                GucRcvNew = 1;                                          /*  置接收新数据标志            */
                for (GulNum = 0; GulNum < 8; GulNum++)
                {                                                       /*  连续接收8个字节             */
                    GucRcvBuf[GulNum] = LPC_USART->RBR;
                }
                break;
            
            case 0x0C:                                                  /*  字符超时中断                */
                GucRcvNew = 1;
                while ((LPC_USART->LSR & 0x01) == 0x01)
                {                                                       /*  判断数据是否接收完毕        */
                    GucRcvBuf[GulNum] = LPC_USART->RBR;
                    GulNum++;
                }
                break;
                
            default:
                break;
        }
    }
	
}

 /*********************************************************************************************************
** Function name:       sectorPrepare
** Descriptions:        IA操作扇区选择，命令代码50
** input parameters:    ucSec1:           起始扇区
**                      ucSec2:           终止扇区
** output parameters:   GulParamout[0]:    IAP操作码状态,IAP返回值
** Returned value:      GulParamout[0]:    IAP操作码状态,IAP返回值
*********************************************************************************************************/
uint32_t sectorPrepare (uint8_t ucSec1, uint8_t ucSec2)
{  
    GulParamin[0] = IAP_Prepare;                                        /*  设置命令字                  */
    GulParamin[1] = ucSec1;                                             /*  设置参数                    */
    GulParamin[2] = ucSec2;                            
    (*IAP_Entry)(GulParamin, GulParamout);                              /*  调用IAP服务程序             */
   
    return (GulParamout[0]);                                            /*  返回状态码                  */
}

/*********************************************************************************************************
** Function name:       ramCopy
** Descriptions:        复制RAM的数据到FLASH,命令代码51
** input parameters:    ulDst:             目标地址,即FLASH起始地址。以512字节为分界
**                      ulSrc:             源地址,即RAM地址。地址必须字对齐
**                      ulNo:              复制字节个数,?512/1024/4096/8192
** output parameters:   GulParamout[0]:    IAP操作码状态,IAP返回值
** Returned value:      GulParamout[0]:    IAP操作码状态,IAP返回值
*********************************************************************************************************/
uint32_t ramCopy (uint32_t ulDst, uint32_t ulSrc, uint32_t ulNo)
{  
    GulParamin[0] = IAP_RAMTOFLASH;                                     /*  设置命令字                   */
    GulParamin[1] = ulDst;                                              /*  设置参数                     */
    GulParamin[2] = ulSrc;
    GulParamin[3] = ulNo;
    GulParamin[4] = IAP_FCCLK;
    (*IAP_Entry)(GulParamin, GulParamout);                              /*  调用IAP服务程序              */
    
    return (GulParamout[0]);                                            /*  返回状态码                   */
}

/*********************************************************************************************************
** Function name:       sectorErase
** Descriptions:        扇区擦除,命令代码52
** input parameters:    ucSec1             起始扇区
**                      ucSec2             终止扇区92
** output parameters:   GulParamout[0]:    IAP操作码状态,IAP返回值
** Returned value:      GulParamout[0]:    IAP操作码状态,IAP返回值
*********************************************************************************************************/
uint32_t sectorErase (uint8_t ucSec1, uint8_t ucSec2)
{  
    GulParamin[0] = IAP_ERASESECTOR;                                    /*  设置命令字                   */
    GulParamin[1] = ucSec1;                                             /*  设置参数                     */
    GulParamin[2] = ucSec2;
    GulParamin[3] = IAP_FCCLK;
    (*IAP_Entry)(GulParamin, GulParamout);                              /*  调用IAP服务程序              */

    return (GulParamout[0]);                                            /*  返回状态码                   */
}

/*********************************************************************************************************
** Function name:       blankChk
** Descriptions:        扇区查空,命令代码53
** input parameters:    ucSec1:              起始扇区
**                      ucSec2:              终止扇区92
** output parameters:   GulParamout[0]:    IAP操作码状态,IAP返回值
** Returned value:      GulParamout[0]:    IAP操作码状态,IAP返回值
*********************************************************************************************************/
uint32_t blankChk (uint8_t ucSec1, uint8_t ucSec2)
{  
    GulParamin[0] = IAP_BLANKCHK;                                       /*  设置命令字                   */
    GulParamin[1] = ucSec1;                                             /*  设置参数                     */
    GulParamin[2] = ucSec2;
    (*IAP_Entry)(GulParamin, GulParamout);                              /*  调用IAP服务程序              */

    return (GulParamout[0]);                                            /*  返回状态码                   */
}

/*********************************************************************************************************
** Function name:       parIdRead
** Descriptions:        读取器件ID,命令代码54
** input parameters:    无
** output parameters:   GulParamout[0]:    IAP操作码状态,IAP返回值
** Returned value:      GulParamout[0]:    IAP操作码状态,IAP返回值
*********************************************************************************************************/
uint32_t parIdRead (void)
{  
    GulParamin[0] = IAP_READPARTID;                                     /*  设置命令字                   */
    (*IAP_Entry)(GulParamin, GulParamout);                              /*  调用IAP服务程序              */

    return (GulParamout[0]);                                            /*  返回状态码                   */
}

/*********************************************************************************************************
** Function name:       codeIdBoot
** Descriptions:        读取器件Boot代码版本,命令代码55
** input parameters:    无
** output parameters:   GulParamout[0]:    IAP操作码状态,IAP返回值
** Returned value:      GulParamout[0]:    IAP操作码状态,IAP返回值
*********************************************************************************************************/
uint32_t codeIdBoot (void)
{  
    GulParamin[0] = IAP_BOOTCODEID;                                     /*  设置命令字                   */
    (*IAP_Entry)(GulParamin, GulParamout);                              /*  调用IAP服务程序              */

    return (GulParamout[0]);                                            /*  返回状态码                   */
}

/*********************************************************************************************************
** Function name:       dataCompare
** Descriptions:        数据校验,命令代码56
** input parameters:    ulDst:             目标地址,即RAM/FLASH起始地址。地址必须字对齐
**                      ulSrc:             源地址,即FLASH/RAM地址。地址必须字对齐
**                      ulNo:              复制字节个数，必须能被4整除
** output parameters:   GulParamout[0]:    IAP操作码状态,IAP返回值
** Returned value:      GulParamout[0]:    IAP操作码状态,IAP返回值
*********************************************************************************************************/
uint32_t dataCompare (uint32_t ulDst, uint32_t ulSrc, uint32_t ulNo)
{  
    GulParamin[0] = IAP_COMPARE;                                        /*  设置命令字                   */
    GulParamin[1] = ulDst;                                              /*  设置参数                     */
    GulParamin[2] = ulSrc;
    GulParamin[3] = ulNo;
    (*IAP_Entry)(GulParamin, GulParamout);                              /*  调用IAP服务程序              */

    return (GulParamout[0]);                                            /*  返回状态码                   */
}
 
 
//检查数据是否在正常范围内
//正常返回0；超限返回1
unsigned char ICPErrCheck(void)
{
    unsigned char i, cnt;
//    printf("LightICP = %d",LightICP);
    if ((LightICP > ICPLimitMax) || (LightICP < ICPLimitMin) 
        ||(LightICP < DarkICP))
    {
        if((LightICP > ICPLimitMax))
        {
            cnt = 10;
        }
        else if(LightICP < ICPLimitMin)
        {
            cnt = 8;
        }
        else if(LightICP < DarkICP)
        {
            cnt = 4;
        } 
        else 
        {
            cnt = 2;
        }
        GRN_LED_OFF;
        NVIC_DisableIRQ(TIMER_32_0_IRQn);
        //printf("cnt = %d",cnt);
        for (i=0; i<cnt; i++)
        {
            RED_LED_Toggle;
            delay_ms(200);
        }

        ICPcnt = 0;
        DarkICP = 0;
        LightICP = 0; 
        ICPUpdateReady = 0;
        NVIC_EnableIRQ(TIMER_32_0_IRQn);
        GRN_LED_ON;	
        return 1;
    }
    return 0; 
}
/**********************************************************************
 * @brief	  Handle interrupt from 32-bit timer
 * @return	Nothing
 * @note	  used for 
 * @author  FENG XQ
 * @date    2016-08-13
 *********************************************************************/
void TIMER32_0_IRQHandler(void)
{
    LPC_TIMER32_0->IR  = 1 << 4;            //清除CAP0中断标志
    ICPcnt++;
}

/**********************************************************************
 * @brief	  SysTick_Handler Interrupt Handler
 * @return	Nothing
 * @note	  Th.
 * @author  FENG XQ
 * @date    2016-08-16
 *********************************************************************/
//-------------------------------------------------------------------------------------------
//                    Light ICP                                          Dark ICP
//     ^  500ms ^      1000ms     ^          2000ms                ^      1000ms      ^
//     --------------------------------------------------------------------------------
//     +        *                 *                                *                  *
//Light On  ICP begin        ICP End/Light Off                   ICP begin         ICP End 
//-------------------------------------------------------------------------------------------  
//Light on 1500ms  Light Off 3000ms
//ICP sampling 1000ms both in light or dark periods
//ICP sampling begin befor 1000ms LED light on/off (stable light)
void SysTick_Handler(void)
{
    static unsigned int timer0_cnt = 0;
    timer0_cnt++;
    count_time++;
    Chip_GPIO_SetPinToggle(LPC_GPIO, 2, 1);
    
    switch (timer0_cnt)
    {
        case    1:                              //light sourcce on
            LGT_LED_ON;
            break;
        case    250:                            //ICP begin, Light ICP measurement
            ICPcnt = 0;
            NVIC_EnableIRQ(TIMER_32_0_IRQn);
            break;
        case    750:                            //ICP end and Light source off
            LightICP = ICPcnt;
            NVIC_DisableIRQ(TIMER_32_0_IRQn);
            LGT_LED_OFF;
            break;
        case    1751:                           //ICP begin, Dark ICP measurement
            ICPcnt = 0;
            NVIC_EnableIRQ(TIMER_32_0_IRQn);
            break;
        case    2250:                           //End Dack ICP measurement
            DarkICP = ICPcnt;
            NVIC_DisableIRQ(TIMER_32_0_IRQn);
            ICPUpdateReady = 1;
            //定标时 显示测得值
            if((CalibrationFlag)&&(CalibrationCnt<5))
            {
                TxBuff[0] = 0xB1;
                TxBuff[1] = masterstate;
                TxBuff[2] = LightICP >> 8;
                TxBuff[3] = LightICP &0xFF;
                TxBuff[4] = DarkICP >> 8;
                TxBuff[5] = DarkICP & 0xFF;
                TxBuff[6] = CalibrationCnt >> 8;
                TxBuff[7] = CalibrationCnt & 0xFF;
                Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);

                CalibrationBuf[CalibrationCnt] = (LightICP - DarkICP);
                CalibrationCnt ++;
            }
            else
            {
                //异常时不改变错误状态
                if ((LightICP < ICPLimitMax) && (LightICP > ICPLimitMin) 
                    && (LightICP > DarkICP))
                {
                    if((LightICP - DarkICP) < LeakThreshold)           //blood leak
                    { 
                        BloodLeakFlage = 1; 
                    }
                    else
                    { 
                        BloodLeakFlage = 0;
                    }                    
                }
            }
            timer0_cnt = 0;
            break;
        default:
            break;
    }
}

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/*
static void Init_ADC_PinMux(void)
{
#if (defined(BOARD_NXP_XPRESSO_11U14) || defined(BOARD_NGX_BLUEBOARD_11U24) || defined(BOARD_NXP_XPRESSO_11U37H))
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, FUNC2);
#elif defined(BOARD_NXP_XPRESSO_11C24)
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_11, FUNC2);
#else
    #error "Pin muxing for ADC not configured"
#endif
}
*/
/*****************************************************************************
 * 函数原型声明
 ****************************************************************************/

static void fail(void)
{
    while (1) { }
}

void init(void)
{
    //CriticalSectionLocker lock;
    if (!initialized)
    {
        initialized = true;

        if ((SystemCoreClock % 1000000) != 0)  // Core clock frequency validation
        {
            fail();
        }

        if (SysTick_Config((SystemCoreClock / 1000000) * USecPerOverflow) != 0)
        {
            fail();
        }
    }
}

//定标
void Leak_Calibration(void)
{
    uint8_t ucErr;
    uint32_t p;

    CalibrationFlag = 1;

    RED_LED_OFF;
    GRN_LED_ON;
    CalibrationCnt = 0;
    //等待定标结束
    //测试5组数， 取后4组数求平均
    do 
    {
        __WFI();
    }
    while (CalibrationCnt<5);

    GRN_LED_OFF;
    RED_LED_ON;
    CalibrationFlag = 0;
    CalibrationValue = (CalibrationBuf[1]+CalibrationBuf[2]+CalibrationBuf[3]+CalibrationBuf[4])/4;
    
    if(ICPErrCheck())               //判断定标值是否正常  否 红灯闪烁
    {
        LGT_LED_OFF;
        RED_LED_OFF;
        GRN_LED_OFF;
        while (ICPErrCheck())       //ICP values exceed limits
        {
            if(GucRcvNew)
            {
                uartread();
            }
            RED_LED_Toggle;
            delay_ms(200);
        }
    }
    else 
    {
        /* 定标正常 将定标值存入Flash 中 */
        IAP_Entry = (void(*)())IAP_ENTER_ADR;
        NVIC_DisableIRQ(TIMER_32_0_IRQn);
        NVIC_DisableIRQ(SysTick_IRQn);
        ucErr = parIdRead();                                                /*  读器件ID                    */
        ucErr = codeIdBoot();                                               /*  读Boot版本号                */

        ucErr = sectorPrepare(7, 7);                                        /*  准备扇区1                   */
        ucErr = sectorErase(7, 7);                                          /*  擦除扇区1                   */
        ucErr = blankChk(7, 7) ;                                            /*  查空扇区1                   */

        ucErr = sectorPrepare(7, 7);                                        /*  选择扇区1                   */
        ucErr = ramCopy(0x00007000, 0x10000000, 512);                       /*  写数据到扇区1               */
        ucErr = dataCompare(0x00007000, 0x10000000, 4);                     /*  比较扇区                    */
        NVIC_EnableIRQ(TIMER_32_0_IRQn);
        NVIC_EnableIRQ(UART0_IRQn);

        p = ((*((unsigned char*)0x00007025))<<8) + (*((unsigned char*)0x00007024));
        
        /* 如果存储出现错误 红灯闪烁 */
        if(ucErr != 0)
        {
            LGT_LED_OFF;
            RED_LED_OFF;
            GRN_LED_OFF;
            while (1)           //ICP values exceed limits
            {
                RED_LED_Toggle;
                delay_ms(500);
            } 
        }
    }
    
    if ((CalibrationValue>ICPLimitMax)||(CalibrationValue<ICPLimitMin))
    {
        LGT_LED_OFF;
        RED_LED_OFF;
        GRN_LED_OFF;
        while ((CalibrationValue>ICPLimitMax)||(CalibrationValue<ICPLimitMin))           //ICP values exceed limits
        {
            if(GucRcvNew) uartread();
            RED_LED_Toggle;
            delay_ms(200);
        }
   } 
   else
   {
        LeakThreshold = (CalibrationValue * LeakCoefficient)/10;
   }
    GRN_LED_ON;
    RED_LED_OFF;
    BloodLeak_NO;
}

/**********************************************************************
 * @brief	  
 * @return	Nothing
 * @note	  Function  
 * @author  WANG HS
 * @date    2016-12-27
 *********************************************************************/
int main(void)
{
//    unsigned char i = 0;

//    uint32_t n;
//    uint8_t ucErr;
//    uint32_t p;
    SystemCoreClockUpdate();

    Board_Init();
    //开启串口中断
    NVIC_EnableIRQ(UART0_IRQn);                                          /* 使能UART中断，并配置优先级   */
    NVIC_SetPriority(UART0_IRQn, 1);
    Chip_UART_IntEnable(LPC_USART,UART_IER_RBRINT);

    /*每 100us 产生中断 向压电陶瓷发送方波 并读取反馈*/
    Timer32_0_Init();

    SysTick_Config(SystemCoreClock /500);           //2ms
    
    //Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_IOCON);        //关闭引脚配置时钟
    /* 读取定标值 */
    CalibrationValue =((*((unsigned char*)0x00007025))<<8) + (*((unsigned char*)0x00007024));
    //定标超限报错
    if ((CalibrationValue>ICPLimitMax)||(CalibrationValue<ICPLimitMin))
    {
        LGT_LED_OFF;
        RED_LED_OFF;
        GRN_LED_OFF;
        while ((CalibrationValue>ICPLimitMax)||(CalibrationValue<ICPLimitMin))           //ICP values exceed limits
        {
            if(GucRcvNew)
            {
                uartread();
            }
            RED_LED_Toggle;
            delay_ms(200);
        }
    } 
    else
    {
        LeakThreshold = (CalibrationValue * LeakCoefficient)/10;
    }
    
    GRN_LED_ON;
    RED_LED_OFF;
    BloodLeak_NO;

    while (1) 
    {
        if(GucRcvNew)
        {
            uartread();
        } 
        if ((!checkstart)&&(ICPUpdateReady))
        {
            ICPErrCheck();
  
            if (BloodLeakFlage)
            {
                BloodLeak_YES;
                RED_LED_ON;
                GRN_LED_OFF;
            }
            else
            {
                BloodLeak_NO;
                RED_LED_OFF;
                GRN_LED_ON;
            }
            TxBuff[0] = 0xA0;
            TxBuff[1] = masterstate;
            TxBuff[2] = 0;
            TxBuff[3] = 0x03;
            TxBuff[4] = 0;
            TxBuff[5] = 0;
            TxBuff[6] = 0;
            TxBuff[7] = BloodLeakFlage;
            Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
            ICPUpdateReady = 0;
        }
        //发送测试数据
        if (checkstart && ICPUpdateReady)
        {
            ICPErrCheck();
            if (BloodLeakFlage)
            {
                BloodLeak_YES;
                RED_LED_ON;
                GRN_LED_OFF;
            }
            else
            {   
                BloodLeak_NO;
                RED_LED_OFF;
                GRN_LED_ON;
            }
            TxBuff[0] = 0xA0;
            TxBuff[1] = masterstate;
            TxBuff[2] = 0;
            TxBuff[3] = (!BloodLeakFlage) << 1;
            TxBuff[4] = 0;
            TxBuff[5] = 0;
            TxBuff[6] = 0;
            TxBuff[7] = BloodLeakFlage;
            Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
            checkstart = 0;
            ICPUpdateReady = 0;
        }
        delay_ms(10);
        
        __WFI();      // Go to Sleep
    }
}
void uartread(void)
{
    if(GucRcvNew == 1)                                                          /* 判断是否有新数据             */
    {
        GucRcvNew = 0;                                                      /* 清除标志                     */
        masterstate = GucRcvBuf[1];
        switch (GucRcvBuf[0])
        {
            case 0x50:
            {
                TxBuff[0] = 0xB0;
                TxBuff[1] = masterstate;

                TxBuff[2] = hardwareversion;
                TxBuff[3] = softwareversion ;
                //上电运行时间
                TxBuff[4] = movetime >> 24;
                TxBuff[5] = (movetime & 0xFFFFFF) >> 16 ;
                TxBuff[6] = (movetime & 0xFFFF) >> 8;
                TxBuff[7] = movetime & 0xFF;
                Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
                break;
            }

            case 0x51:
            { 
                TxBuff[0] = 0xB1;
                TxBuff[1] = masterstate;
                TxBuff[2] = LightICP >> 8;
                TxBuff[3] = LightICP &0xFF;
                TxBuff[4] = DarkICP >> 8;
                TxBuff[5] = DarkICP & 0xFF;
                TxBuff[6] = CalibrationCnt >> 8;
                TxBuff[7] = CalibrationCnt & 0xFF;
                Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
                break;
            }

            case 0x52:
            {
                TxBuff[0] = 0xB2;
                TxBuff[1] = masterstate;
                TxBuff[2] = CalibrationValue >> 8;
                TxBuff[3] = CalibrationValue &0xFF;
                TxBuff[4] = (LightICP - DarkICP) >> 8;
                TxBuff[5] = (LightICP - DarkICP) & 0xFF;
                TxBuff[6] = 0;
                TxBuff[7] = BloodLeakFlage;
                Chip_UART_SendBlocking(LPC_USART,&TxBuff, 8);
                break;
           }
           
           case 0x61:
           {
                //进行定标
                Leak_Calibration();
                break;
           }
           
           case 0x60:
           {
                check = GucRcvBuf[8];
                if((masterstate == 0x02) && (check == 0x01 || check == 0x02))
                {
                  checkstart = 1;
                }
                break;
            }
            default:
                break;
        }
    }
}

void delay_ms(uint16_t time)
{
    count_time = 0;
    while((time / 2) >= count_time);
}

//end of the file FENGXQ
