/**
 * @file main.c
 * @brief 电容触摸传感器示例程序
 * @version 1.0
 * @date 2025-11-01
 */

#include "gd32c2x1.h"
#include "systick.h"
#include "cap_touch.h"
#include <stdio.h>

/* 触摸数据就绪回调函数 */
void on_touch_data_ready(capture_data_t *data);

/* USART配置 */
void usart_config(void);

/**
 * @brief 主函数
 */
int main(void)
{
    capture_data_t touch_data;
    
    /* 配置系统滴答定时器 */
    systick_config();
    
    /* 配置USART用于数据输出 */
    usart_config();
    
    /* 初始化电容触摸模块 */
    cap_touch_init();
    
    /* 注册数据就绪回调函数 */
    cap_touch_register_data_ready_callback(on_touch_data_ready);
    
    printf("\r\n/**** GD32 Capacitive Touch Sensor Demo ****/\r\n");
    printf("Channels: %d\r\n", CAP_TOUCH_CHANNEL_COUNT);
    printf("Touch sensing started...\r\n\r\n");
    
    while (1) {
        /* 循环调用触摸检测处理函数 */
        cap_touch_process();
        
        /* 延时一小段时间，控制扫描速度 */
        /* 也可以不延时，让扫描速度达到最快 */
        // delay_1ms(1);
        
        /* 尝试从FIFO读取数据 */
        if (cap_touch_fifo_read(&touch_data) == CAP_OK) {
            /* 处理读取到的触摸数据 */
            printf("FIFO Data - CH0:%lu CH1:%lu CH2:%lu CH3:%lu CH4:%lu CH5:%lu TS:%llu\r\n",
                   touch_data.values[0], touch_data.values[1], touch_data.values[2],
                   touch_data.values[3], touch_data.values[4], touch_data.values[5],
                   touch_data.timestamp);
        }
    }
}

/**
 * @brief 触摸数据就绪回调函数
 * 
 * 当6个通道数据采集完成时会自动调用此函数
 */
void on_touch_data_ready(capture_data_t *data)
{
    /* 实时输出触摸数据 */
    printf("Touch - CH0:%lu CH1:%lu CH2:%lu CH3:%lu CH4:%lu CH5:%lu\r\n",
           data->values[0], data->values[1], data->values[2],
           data->values[3], data->values[4], data->values[5]);
    
    /* 可以在这里添加触摸检测逻辑 */
    /* 例如: 如果值小于某个阈值，认为有触摸发生 */
    for (int i = 0; i < CAP_TOUCH_CHANNEL_COUNT; i++) {
        if (data->values[i] < 10000) {  /* 阈值需要根据实际情况调整 */
            printf("Channel %d touched!\r\n", i);
        }
    }
}

/**
 * @brief 配置USART
 */
void usart_config(void)
{
    /* 使能USART时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);
    
    /* 连接引脚到USART */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);  /* USART0_TX */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10); /* USART0_RX */
    
    /* 配置USART GPIO */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_3, GPIO_PIN_9);
    
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_3, GPIO_PIN_10);
    
    /* 配置USART参数 */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

/**
 * @brief 重定向printf到USART
 */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
