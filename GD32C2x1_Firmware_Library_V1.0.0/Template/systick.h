/**
 * @file systick.h
 * @brief 系统滴答定时器头文件
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include "gd32c2x1.h"

/* 配置systick */
void systick_config(void);

/* 延迟函数 */
void delay_1ms(uint32_t count);
void delay_decrement(void);

#endif /* SYSTICK_H */
