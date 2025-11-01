/**
 * @file systick.c
 * @brief 系统滴答定时器实现
 */

#include "systick.h"
#include "cap_touch.h"

static volatile uint32_t delay_time = 0;

/**
 * @brief 配置systick定时器
 */
void systick_config(void)
{
    /* setup systick timer for 1ms interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/**
 * @brief 延迟毫秒
 */
void delay_1ms(uint32_t count)
{
    delay_time = count;
    while(0U != delay_time){
    }
}

/**
 * @brief 延迟递减
 */
void delay_decrement(void)
{
    if (0U != delay_time){
        delay_time--;
    }
}

/**
 * @brief SysTick中断处理函数
 */
void SysTick_Handler(void)
{
    delay_decrement();
    cap_touch_systick_handler();  /* 更新触摸模块时间戳 */
}
