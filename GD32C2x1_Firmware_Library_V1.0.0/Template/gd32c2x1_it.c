/*!
    \file    gd32c2x1_it.c
    \brief   interrupt service routines

    \version 2025-05-30, V1.0.0, firmware for gd32c2x1
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32c2x1_it.h"
#include "main.h"
#include "systick.h"
#include "cap_touch.h"

#define SRAM_ECC_ERROR_HANDLE(s)    do{}while(1)

void NMI_Handler(void)
{
    if(SET == syscfg_interrupt_flag_get(SYSCFG_FLAG_ECCME)) {
        SRAM_ECC_ERROR_HANDLE("SRAM two bits non-correction check error\r\n"); 
    } else if(SET == syscfg_interrupt_flag_get(SYSCFG_FLAG_ECCSE)) {
        SRAM_ECC_ERROR_HANDLE("RAM single bit correction check error\r\n"); 
    } else { 
        /* if NMI exception occurs, go to infinite loop */
        /* HXTAL clock monitor NMI error or NMI pin error */
        while(1) {
        }
    }
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles USART0 wakeup interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART0_WKUP_IRQHandler(void)
{
    /* 完全禁用USART以停止中断 */
    usart_disable(USART0);
    
    /* 禁用所有中断 */
    USART_CTL0(USART0) &= ~(USART_CTL0_IDLEIE | USART_CTL0_RBNEIE | USART_CTL0_TCIE | USART_CTL0_TBEIE | USART_CTL0_PERRIE);
    USART_CTL1(USART0) &= ~(USART_CTL1_LBDIE);
    USART_CTL2(USART0) &= ~(USART_CTL2_WUIE | USART_CTL2_ERRIE | USART_CTL2_CTSIE);
    
    /* 清除所有中断标志 */
    USART_INTC(USART0) = 0xFFFFFFFF;
    
    /* 禁用NVIC中断 */
    nvic_irq_disable(USART0_WKUP_IRQn);
    
    /* 重新使能USART（仅用于发送） */
    usart_enable(USART0);
}

/* Note: SysTick_Handler is defined in systick.c */

/*!
    \brief      this function handles TIMER0 channel interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER0_Channel_IRQHandler(void)
{
    if (SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH0)) {
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH0);
        cap_touch_timer_capture_callback(TIMER0, TIMER_CH_0);
    }
    
    if (SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH1)) {
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH1);
        cap_touch_timer_capture_callback(TIMER0, TIMER_CH_1);
    }
    
    if (SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH2)) {
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH2);
        cap_touch_timer_capture_callback(TIMER0, TIMER_CH_2);
    }
    
    if (SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH3)) {
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH3);
        cap_touch_timer_capture_callback(TIMER0, TIMER_CH_3);
    }
}

/*!
    \brief      this function handles TIMER2 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    if (SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH0)) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH0);
        cap_touch_timer_capture_callback(TIMER2, TIMER_CH_0);
    }
    
    if (SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH1)) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH1);
        cap_touch_timer_capture_callback(TIMER2, TIMER_CH_1);
    }
    
    if (SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH2)) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH2);
        cap_touch_timer_capture_callback(TIMER2, TIMER_CH_2);
    }
    
    if (SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH3)) {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH3);
        cap_touch_timer_capture_callback(TIMER2, TIMER_CH_3);
    }
}

/*!
    \brief      this function handles TIMER15 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER15_IRQHandler(void)
{
    if (SET == timer_interrupt_flag_get(TIMER15, TIMER_INT_FLAG_CH0)) {
        timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_CH0);
        cap_touch_timer_capture_callback(TIMER15, TIMER_CH_0);
    }
}

/*!
    \brief      this function handles TIMER16 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER16_IRQHandler(void)
{
    if (SET == timer_interrupt_flag_get(TIMER16, TIMER_INT_FLAG_CH0)) {
        timer_interrupt_flag_clear(TIMER16, TIMER_INT_FLAG_CH0);
        cap_touch_timer_capture_callback(TIMER16, TIMER_CH_0);
    }
}

