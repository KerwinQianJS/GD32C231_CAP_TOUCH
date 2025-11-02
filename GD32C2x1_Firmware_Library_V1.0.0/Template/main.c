/**
 * @file main.c
 * @brief 电容触摸传感器示例程序
 * @version 1.0
 * @date 2025-11-01
 */

#include "gd32c2x1.h"
#include "systick.h"
#include "cap_touch.h"

/* 触摸检测阈值 - 根据实际情况调整 */
#define TOUCH_THRESHOLD 150  /* 触摸阈值，超过此值认为被触摸 */

/* 触摸数据就绪回调函数 */
void on_touch_data_ready(capture_data_t *data);

/* USART配置 */
void usart_config(void);

/* USART发送函数 */
void usart_send_byte(uint8_t data);
void usart_send_buffer(uint8_t *buffer, uint16_t length);

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
    
    /* 初始化触摸指示GPIO (PB0-PB5) */
    cap_touch_gpio_indicator_init();
    
    /* 注册数据就绪回调函数 */
    cap_touch_register_data_ready_callback(on_touch_data_ready);
    
    while (1) {
        /* 每秒通过PB6(USART0_TX)发送"1234"字符串 */
        // usart_send_byte('1');
        // usart_send_byte('2');
        // usart_send_byte('3');
        // usart_send_byte('4');
        // usart_send_byte('\r');  /* 回车 */
        // usart_send_byte('\n');  /* 换行 */
        
        /* 简单延时 (大约1秒，具体时间取决于系统时钟) */
        // for(volatile uint32_t i = 0; i < 3000000; i++) {
        //     __NOP();
        // }
        
        /* 循环调用触摸检测处理函数 */
        cap_touch_process();
    }
}

uint8_t sum_check(uint8_t *data, uint16_t len)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF; // 0x01 & oxff   0000 0001
}

/**
 * @brief 触摸数据就绪回调函数
 * 
 * 当所有通道数据采集完成时会自动调用此函数
 */
void on_touch_data_ready(capture_data_t *data)
{
    /* 更新所有通道的GPIO指示状态 */
    for(uint8_t i = 0; i < CAP_TOUCH_CHANNEL_COUNT; i++) {
        cap_touch_update_gpio_indicator(i, TOUCH_THRESHOLD);
    }
    
            // cap_test_gpio_toggle();
            uint8_t send_buf[15] = {0};
    /* 发送数据包头 */
    // usart_send_byte(0xAA);
    // usart_send_byte(0x55);
    send_buf[0] = 0xAA;
    send_buf[1] = 0x55;

    send_buf[2] = (data->values[0] >> 0) & 0xFF;   /* 低字节 */
    send_buf[3] = (data->values[0] >> 8) & 0xFF;   /* 高字节 */
    send_buf[4] = (data->values[1] >> 0) & 0xFF;   /* 低字节 */
    send_buf[5] = (data->values[1] >> 8) & 0xFF;   /* 高字节 */
    send_buf[6] = (data->values[2] >> 0) & 0xFF;   /* 低字节 */
    send_buf[7] = (data->values[2] >> 8) & 0xFF;   /* 高字节 */
    send_buf[8] = (data->values[3] >> 0) & 0xFF;   /* 低字节 */
    send_buf[9] = (data->values[3] >> 8) & 0xFF;   /* 高字节 */
    send_buf[10] = (data->values[4] >> 0) & 0xFF;   /* 低字节 */
    send_buf[11] = (data->values[4] >> 8) & 0xFF;   /* 高字节 */
    send_buf[12] = (data->values[5] >> 0) & 0xFF;   /* 低字节 */
    send_buf[13] = (data->values[5] >> 8) & 0xFF;   /* 高字节 */

    send_buf[14] = sum_check(send_buf, 14);

    usart_send_buffer(send_buf, 15);
}

/**
 * @brief 配置USART
 */
void usart_config(void)
{
    /* 先禁用NVIC中的USART0唤醒中断 */
    nvic_irq_disable(USART0_WKUP_IRQn);
    
    /* 使能USART时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART0);
    
    /* 配置PB6为USART0_TX */
    gpio_af_set(GPIOB, GPIO_AF_0, GPIO_PIN_6);  /* USART0_TX */
    
    /* 配置USART TX GPIO */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_1, GPIO_PIN_6);
    
    /* 配置USART参数 */
    usart_deinit(USART0);
    
    /* 禁用所有USART中断 */
    usart_interrupt_disable(USART0, USART_INT_WU);
    usart_interrupt_disable(USART0, USART_INT_IDLE);
    usart_interrupt_disable(USART0, USART_INT_RBNE);
    usart_interrupt_disable(USART0, USART_INT_TC);
    usart_interrupt_disable(USART0, USART_INT_TBE);
    
    /* 清除所有中断标志 */
    USART_INTC(USART0) = 0xFFFFFFFF;
    
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_DISABLE);  /* 不需要接收 */
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    
    /* 禁用唤醒功能 */
    usart_wakeup_disable(USART0);
    
    /* 启用USART */
    usart_enable(USART0);
    
    /* 再次清除所有中断标志 */
    USART_INTC(USART0) = 0xFFFFFFFF;
}

/**
 * @brief 通过USART0发送一个字节
 */
void usart_send_byte(uint8_t data)
{
    usart_data_transmit(USART0, data);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
}

/**
 * @brief 通过USART0发送数据缓冲区
 */
void usart_send_buffer(uint8_t *buffer, uint16_t length)
{
    for(uint16_t i = 0; i < length; i++) {
        usart_send_byte(buffer[i]);
    }
}

/* Note: fputc is defined in gd32c231c_eval.c */

