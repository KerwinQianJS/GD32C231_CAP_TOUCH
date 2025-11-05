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
#define TOUCH_THRESHOLD 150 /* 触摸阈值，超过此值认为被触摸 */

/* DMA发送缓冲区大小 */
#define DMA_SEND_BUFFER_SIZE 32

typedef struct {
    uint16_t header;   /* 包头: 0xA5A5 */
    uint16_t data[6];  /* 6个数据 */
    uint16_t checksum; /* 累加校验值 */
} __attribute__((packed)) cap_frame_t;
/* 32字节对齐的全局DMA发送缓冲区 */
__attribute__((aligned(32))) static cap_frame_t g_dma_send_buffer;

/* 触摸数据就绪回调函数 */
void on_touch_data_ready(capture_data_t *data);

/* USART配置 */
void usart_config(void);

/* DMA配置 */
void dma_config(void);

/* USART发送函数 */
void usart_send_byte(uint8_t data);
void usart_send_buffer(uint8_t *buffer, uint16_t length);

/* DMA发送函数 */
void usart_send_buffer_dma(uint8_t *buffer, uint16_t length);
uint8_t usart_dma_is_busy(void);

/**
 * @brief 主函数
 */
int main(void)
{
    capture_data_t touch_data;

    /* 配置系统滴答定时器 */
    systick_config();

    /* 配置DMA */
    dma_config();

    /* 配置USART用于数据输出 */
    usart_config();

    /* 初始化电容触摸模块 */
    cap_touch_init();

    /* 初始化触摸指示GPIO (PB0-PB5) */
    cap_touch_gpio_indicator_init();

    /* 注册数据就绪回调函数 */
    cap_touch_register_data_ready_callback(on_touch_data_ready);

    while (1)
    {
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

uint16_t sum_check(uint8_t *data, uint16_t len)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum & 0xFFFF; // 0x01 & oxff   0000 0001
}

/**
 * @brief 触摸数据就绪回调函数
 *
 * 当所有通道数据采集完成时会自动调用此函数
 */
void on_touch_data_ready(capture_data_t *data)
{
    // /* 更新所有通道的GPIO指示状态 */
    // for(uint8_t i = 0; i < CAP_TOUCH_CHANNEL_COUNT; i++) {
    //     cap_touch_update_gpio_indicator(i, TOUCH_THRESHOLD);
    // }

    cap_test_gpio_toggle();

    /* 准备数据到全局DMA缓冲区 */
    g_dma_send_buffer.header = 0xA5A5;

    g_dma_send_buffer.data[0] = data->values[0];  /* 低字节 */
    g_dma_send_buffer.data[1] = data->values[1];  /* 低字节 */
    g_dma_send_buffer.data[2] = data->values[2];  /* 低字节 */
    g_dma_send_buffer.data[3] = data->values[3];  /* 低字节 */
    g_dma_send_buffer.data[4] = data->values[4];  /* 低字节 */
    g_dma_send_buffer.data[5] = data->values[5];  /* 低字节 */

    g_dma_send_buffer.checksum = sum_check((uint8_t *)&g_dma_send_buffer.data[0], 12);


    /* 使用DMA发送 */
    usart_send_buffer_dma((uint8_t *)&g_dma_send_buffer, 16);
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

    /* 配置PB6为USART0_TX, PB7为USART0_RX */
    gpio_af_set(GPIOB, GPIO_AF_0, GPIO_PIN_6); /* USART0_TX */
    gpio_af_set(GPIOB, GPIO_AF_0, GPIO_PIN_7); /* USART0_RX */

    /* 配置USART TX GPIO */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_1, GPIO_PIN_6);
    
    /* 配置USART RX GPIO */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_1, GPIO_PIN_7);

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

    usart_baudrate_set(USART0, 921600U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);   /* 使能接收 */
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    /* 禁用唤醒功能 */
    usart_wakeup_disable(USART0);

    /* 启用USART */
    usart_enable(USART0);

    /* 使能USART的DMA发送 */
    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);

    /* 再次清除所有中断标志 */
    USART_INTC(USART0) = 0xFFFFFFFF;
}

/**
 * @brief 配置DMA用于USART发送
 */
void dma_config(void)
{
    dma_parameter_struct dma_init_struct;

    /* 使能DMA和DMAMUX时钟 */
    rcu_periph_clock_enable(RCU_DMA);
    rcu_periph_clock_enable(RCU_DMAMUX);

    /* 复位DMA通道0 (用于USART0_TX) */
    dma_deinit(DMA_CH0);
    dma_struct_para_init(&dma_init_struct);

    /* 配置DMA参数 */
    dma_init_struct.request      = DMA_REQUEST_USART0_TX;          /* USART0_TX请求 */
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;       /* 内存到外设 */
    dma_init_struct.memory_addr  = (uint32_t)&g_dma_send_buffer;    /* 内存地址 */
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;     /* 内存地址自增 */
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;          /* 内存数据宽度8位 */
    dma_init_struct.number       = 0;                              /* 传输数量(稍后设置) */
    dma_init_struct.periph_addr  = (uint32_t)&USART_TDATA(USART0); /* 外设地址 */
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;    /* 外设地址不变 */
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;      /* 外设数据宽度8位 */
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;        /* 超高优先级 */

    dma_init(DMA_CH0, &dma_init_struct);

    /* 配置DMA模式 */
    dma_circulation_disable(DMA_CH0);           /* 非循环模式 */
    dma_memory_to_memory_disable(DMA_CH0);      /* 非内存到内存模式 */
    
    /* 禁用DMAMUX同步模式 */
    dmamux_synchronization_disable(DMAMUX_MUXCH0);
}

/**
 * @brief 通过DMA发送数据缓冲区(非阻塞)
 * @param buffer 要发送的数据缓冲区
 * @param length 要发送的数据长度
 * @note 调用前确保上次传输已完成
 */
void usart_send_buffer_dma(uint8_t *buffer, uint16_t length)
{
    /* 等待上次传输完成 */
    // while (dma_flag_get(DMA_CH0, DMA_FLAG_FTF) == RESET)
    //     ;

    /* 清除传输完成标志 */
    dma_flag_clear(DMA_CH0, DMA_FLAG_FTF);

    /* 禁用DMA通道 */
    dma_channel_disable(DMA_CH0);

    /* 配置传输数量和内存地址 */
    dma_transfer_number_config(DMA_CH0, length);
    dma_memory_address_config(DMA_CH0, (uint32_t)buffer);

    /* 启动DMA传输 */
    dma_channel_enable(DMA_CH0);
}

/**
 * @brief 检查DMA是否正在发送
 * @return 1:忙碌 0:空闲
 */
uint8_t usart_dma_is_busy(void)
{
    return (dma_flag_get(DMA_CH0, DMA_FLAG_FTF) == RESET) ? 1 : 0;
}

/* Note: fputc is defined in gd32c231c_eval.c */
