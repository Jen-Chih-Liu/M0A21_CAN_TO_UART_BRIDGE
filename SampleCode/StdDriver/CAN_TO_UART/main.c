/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/06/19 3:12p $
 * @brief    Demonstrate CAN bus to UART sample code
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "lwrb.h"

#define can_speed 500000
#define uart_baudrate 115200
//#define system_lxt_trim_hirc //select lxt trim hric or hxt 12mhz for system clock
#define CAN_STD_FRAME  //by CAN TX FRAME
#define CAN_TX_RETRY_DISABLE
lwrb_t lw_uart_tx_buff;
__attribute__((aligned(4))) uint8_t uart_tx_buff_data[256];
#define TX_FIFO_SIZE 16  /* TX Hardware FIFO size */
#define UART_TX_SIZE 64
__attribute__((aligned(4))) uint8_t gUartTransmitGroup[UART_TX_SIZE];


lwrb_t lw_uart_rx_buff;
__attribute__((aligned(4))) uint8_t uart_rx_buff_data[256];
#define UART_RX_SIZE   64
__attribute__((aligned(4))) uint8_t gUartReciveGroup[UART_RX_SIZE]  = {0};


volatile uint8_t USART0_RX_FLAG = 0;
volatile  uint8_t USART0_RX_COUNT = 0;
void CAN_Msg_tx_fifo(STR_CANMSG_T *Msg);
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
STR_CANMSG_T rrMsg;

//#define dbg_printf printf
#define dbg_printf(...)
/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if (u32IIDR == 1)
    {
        CAN_Receive(tCAN, 0, &rrMsg);
        CAN_Msg_tx_fifo(&rrMsg);
    }

    if (u32IIDR == 5 + 1)
    {
        CAN_Receive(tCAN, 5, &rrMsg);
        CAN_Msg_tx_fifo(&rrMsg);
    }
}



/**
  * @brief  CAN0_IRQ Handler.
  * @param  None.
  * @return None.
  */
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN->IIDR;

    if (u8IIDRstatus == 0x00008000)       /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if (CAN->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear Rx Ok status*/

            dbg_printf("RX OK INT\n") ;
        }

        if (CAN->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear Tx Ok status*/

            dbg_printf("TX OK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if (CAN->STATUS & CAN_STATUS_EWARN_Msk)
        {
            dbg_printf("EWARN INT\n") ;

            /* Do Init to release busoff pin */
            CAN->CON = (CAN_CON_INIT_Msk | CAN_CON_CCE_Msk);
            CAN->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));

            while (CAN->CON & CAN_CON_INIT_Msk);
        }

        if (CAN->STATUS & CAN_STATUS_BOFF_Msk)
        {
            dbg_printf("BOFF INT\n") ;
        }
    }
    else if (u8IIDRstatus != 0)
    {
        dbg_printf("=> Interrupt Pointer = %d\n", CAN->IIDR - 1);

        CAN_MsgInterrupt(CAN, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN, ((CAN->IIDR) - 1));     /* Clear Interrupt Pending */

    }
    else if (CAN->WU_STATUS == 1)
    {
        dbg_printf("Wake up\n");

        CAN->WU_STATUS = 0;                       /* Write '0' to clear */
    }

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();


#ifdef system_lxt_trim_hirc
    /* Set X32_OUT(PC.4) and X32_IN(PC.5) to input mode */
    GPIO_SetMode(PC, BIT4 | BIT5, GPIO_MODE_INPUT);

    /* Set PC multi-function pins for X32_OUT(PC.4) and X32_IN(PC.5) */
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(SYS_GPC_MFP1_PC4MFP_Msk | SYS_GPC_MFP1_PC5MFP_Msk)) |
                    (SYS_GPC_MFP1_PC4MFP_X32_OUT | SYS_GPC_MFP1_PC5MFP_X32_IN);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable External Low speed crystal (LXT) */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for External Low speed clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);
    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

#else


    /* Set XT1_OUT(PA.4) and XT1_IN(PA.5) to input mode */
    GPIO_SetMode(PA, BIT4 | BIT5, GPIO_MODE_INPUT);
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA5MFP_Msk | SYS_GPA_MFP1_PA4MFP_Msk)) |
                    (SYS_GPA_MFP1_PA5MFP_XT1_IN | SYS_GPA_MFP1_PA4MFP_XT1_OUT);
    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

#endif

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.6 and TXD=PB.4 */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB4MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk)) |
                    (SYS_GPB_MFP1_PB4MFP_UART0_TXD | SYS_GPB_MFP1_PB6MFP_UART0_RXD);

    /* Set PB multi-function pins for CAN0 TXD(PB.7) and RXD(PB.5) */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB5MFP_Msk | SYS_GPB_MFP1_PB7MFP_Msk)) |
                    (SYS_GPB_MFP1_PB5MFP_CAN0_RXD | SYS_GPB_MFP1_PB7MFP_CAN0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/**
  * @brief      Init CAN driver
  */

void CAN_Init(CAN_T  *tCAN)
{
    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_CAN0CKEN_Msk;
}

/**
  * @brief      Disable CAN
  * @details    Reset and clear all CAN control and disable CAN IP
  */

void CAN_STOP(CAN_T  *tCAN)
{
    /* Disable CAN0 Clock and Reset it */
    SYS->IPRST1 |= SYS_IPRST1_CAN0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_CAN0RST_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_CAN0CKEN_Msk;
}



void SelectCANSpeed(CAN_T  *tCAN)
{
    uint32_t unItem;
    int32_t i32Err = 0;
    //set can bus speed
    i32Err = CAN_Open(tCAN,  can_speed, CAN_NORMAL_MODE);

    if (i32Err < 0)
    {
        dbg_printf("Set CAN bit rate is fail, please check clock seeting\n");

        while (1);
    }
}
void write_tx_sw_fifo(uint8_t *ch, uint8_t size)
{
    __disable_irq();

    lwrb_write(&lw_uart_tx_buff, ch, size);

    /* Check if Tx is working */
    if ((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
    {
        /* Enable Tx Empty Interrupt. (Trigger first one) */
        UART_ENABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
    }

    __enable_irq();

}

void CAN_Msg_tx_fifo(STR_CANMSG_T *Msg)
{
    uint8_t i;
    gUartTransmitGroup[0] = 0x55;
    gUartTransmitGroup[1] = 0xAA;
    /* Fill in ID area*/
    gUartTransmitGroup[2] = (uint8_t)((Msg->Id >> 24) & 0x000000ff);
    gUartTransmitGroup[3] = (uint8_t)((Msg->Id >> 16) & 0x000000ff);
    gUartTransmitGroup[4] = (uint8_t)((Msg->Id >> 8) & 0x000000ff);
    gUartTransmitGroup[5] = (uint8_t)(Msg->Id & 0x000000ff);
    /* Fill in length area */
    gUartTransmitGroup[6] = (uint8_t)(Msg->DLC);

    /* Fill in data area */
    for (i = 0; i < Msg->DLC; i++)
    {
        gUartTransmitGroup[7 + i] =  Msg->Data[i];
    }

    write_tx_sw_fifo(gUartTransmitGroup, 7 + (Msg->DLC));
}

/*----------------------------------------------------------------------------*/
/*  Send Rx Msg by Normal Mode Function (With Message RAM)                    */
/*----------------------------------------------------------------------------*/

void NormalMode_Rx_init(CAN_T *tCAN)
{
    //reciver std message buffer
    if (CAN_SetRxMsgAndMsk(tCAN, MSG(0), CAN_STD_ID, 0x7FF, 0x0) == FALSE)
    {
        dbg_printf("Set Rx Msg Object failed\n");
        return;
    }

    //reciver ext message buffer
    if (CAN_SetRxMsgAndMsk(tCAN, MSG(5), CAN_EXT_ID, 0x1FFFFFFF, 0x0) == FALSE)
    {
        dbg_printf("Set Rx Msg Object failed\n");
        return;
    }

    /* INT Mode */
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk);
    NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN0_IRQn);
}

void UART0_IRQHandler(void)
{
    uint32_t u32IntStatus;
    int32_t size;
    uint8_t loc_data[1];
    u32IntStatus = UART0->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAINT_Msk))
    {
        /* Get all the input characters */
        while (UART_GET_RX_EMPTY(UART0) == 0)
        {
            loc_data[0] = UART_READ(UART0);
            lwrb_write(&lw_uart_rx_buff, loc_data, 1);
        }

    }

    /* Check if Tx is working */
    if ((UART0->INTEN & UART_INTEN_THREIEN_Msk) != 0)
    {
        if (u32IntStatus & UART_INTSTS_THREIF_Msk)
        {
            if ((lw_uart_tx_buff.w - lw_uart_tx_buff.r) != 0)
            {
                /* Fill the Tx FIFO */
                size = (lw_uart_tx_buff.w - lw_uart_tx_buff.r);

                if (size >= TX_FIFO_SIZE)
                {
                    size = TX_FIFO_SIZE;
                }

                while (size)
                {
                    lwrb_read(&lw_uart_tx_buff, loc_data, 1);

                    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)
                    {
                    };

                    UART0->DAT = loc_data[0];

                    size--;
                }
            }
            else
            {
                if ((lw_uart_tx_buff.w - lw_uart_tx_buff.r) == 0)
                {
                    /* No more data, just stop Tx (Stop work) */
                    UART_DISABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
                    lwrb_reset(&lw_uart_tx_buff);
                }
            }
        }
    }

}


/**
 * @brief       HIRC Trim IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The CKFAIL_IRQHandler default IRQ
 */
void CKFAIL_IRQHandler()
{
    if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_TFAILIF_Msk)
    {
        /* Get Trim Failure Interrupt */
        /* Display HIRC trim status */
        dbg_printf("HIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_TFAILIF_Msk;
    }

    if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_CLKERIF_Msk)
    {
        /* Get Clock Error Interrupt */
        /* Display HIRC trim status */
        dbg_printf("Clock Error Interrupt\n");
        /* Clear Clock Error Interrupt */
        SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk;
    }
}


void TrimHIRC()
{
    /*  Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->HIRCTRIMIEN |= (SYS_HIRCTRIMIEN_CLKEIEN_Msk | SYS_HIRCTRIMIEN_TFALIEN_Msk);
    SYS->HIRCTRIMCTL = (SYS->HIRCTRIMCTL & ~SYS_HIRCTRIMCTL_FREQSEL_Msk) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    while (1)
    {
        if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_FREQLOCK_Msk)
        {
            dbg_printf("HIRC Frequency Lock\n");
            SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_FREQLOCK_Msk;     /* Clear Trim Lock */
            break;
        }
    }
}

void CAN_Disable_Auto_TXRetry(CAN_T *tCAN)
{
    tCAN->CON = tCAN->CON | (CAN_CON_DAR_Msk);
}

//PLASE USE LXT OR external crystal 12MHZ, the sample code is use lxt trim
int main()
{
    uint8_t rx_size = 0;
    uint8_t loc_data_m[1];
    CAN_T *tCAN;

    tCAN = (CAN_T *) CAN;

    SYS_Init();
    /* Enable Interrupt */
    NVIC_EnableIRQ(CKFAIL_IRQn);
#ifdef system_lxt_trim_hirc
    /* Trim HIRC to 48MHz */
    TrimHIRC();
#endif
    lwrb_init(&lw_uart_tx_buff, uart_tx_buff_data, sizeof(uart_tx_buff_data)); /* Initialize buffer */
    lwrb_init(&lw_uart_rx_buff, uart_rx_buff_data, sizeof(uart_rx_buff_data)); /* Initialize buffer */
    /* Init UART0*/
    UART_Open(UART0, uart_baudrate);
    UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(UART0_IRQn);

    /* Select CAN Multi-Function */
    CAN_Init(tCAN);

    SelectCANSpeed(tCAN);

    NormalMode_Rx_init(tCAN);
#ifdef CAN_TX_RETRY_DISABLE
    CAN_Disable_Auto_TXRetry(tCAN);
#endif

    while (1)
    {


        if ((lw_uart_rx_buff.w - lw_uart_rx_buff.r) > 0)
        {
            __disable_irq();
            lwrb_read(&lw_uart_rx_buff, loc_data_m, 1);
            gUartReciveGroup[USART0_RX_COUNT] = loc_data_m[0];

            //compare start byte
            if (USART0_RX_COUNT == 0 && gUartReciveGroup[USART0_RX_COUNT] != 0x55)
                goto Exit_UART_RX_initial_check;

            USART0_RX_COUNT++;

            //check the format
            if (gUartReciveGroup[0] == 0x55 && gUartReciveGroup[1] == 0xaa && gUartReciveGroup[6] != 0 &&
                    (USART0_RX_COUNT >= (gUartReciveGroup[6] + 7))
               )
            {
							  
                USART0_RX_FLAG = 1;
            }

Exit_UART_RX_initial_check:

            if ((lw_uart_rx_buff.w - lw_uart_rx_buff.r) == 0)
            {
                lwrb_reset(&lw_uart_rx_buff);
            }

            __enable_irq();
        }


        if (USART0_RX_FLAG == 1)
        {
            /* The format of UartReceiveGroup is
            * 55 AA ID1 ID2 ID3 ID4 Length Data1 Data2 ...
            * */

            uint32_t id = 0;
            uint32_t count = 0;
            /* Get device address */
            id = gUartReciveGroup[2] << 24;
            id += gUartReciveGroup[3] << 16;
            id += gUartReciveGroup[4] << 8;
            id += gUartReciveGroup[5];

            STR_CANMSG_T tMsg;
            /* Send a 11-bits message */
            tMsg.FrameType = CAN_DATA_FRAME;

#ifdef CAN_STD_FRAME
            tMsg.IdType   = CAN_STD_ID;
#else
            tMsg.IdType   = CAN_EXT_ID;
#endif
            tMsg.Id       = id;
            tMsg.DLC      = gUartReciveGroup[6];

            for (count = 0; count < gUartReciveGroup[6]; count++)
                tMsg.Data[count]  = gUartReciveGroup[7 + count];

            if (CAN_Transmit(tCAN, MSG(1), &tMsg) == FALSE) // Configure Msg RAM and send the Msg in the RAM
            {
                dbg_printf("Set Tx Msg Object failed\n");
            }

            //clear buffer
            for (count = 0; count < 7; count++)
                gUartReciveGroup[count] = 0;
            USART0_RX_COUNT=0;						
            USART0_RX_FLAG = 0;   //clear flag
        }
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
