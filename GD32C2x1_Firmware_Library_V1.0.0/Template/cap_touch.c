/**
 * @file cap_touch.c
 * @brief 电容式触摸传感器模块实现 - GD32C2x1版本
 * @version 1.0
 * @date 2025-11-01
 *
 * 本模块使用定时器输入捕获功能来测量触摸板电容充电时间，
 * 通过状态机实现非阻塞式的触摸检测。
 *
 * 工作原理:
 * 1. 放电阶段: GPIO输出低电平，使触摸电容放电
 * 2. 充电阶段: GPIO切换为定时器输入捕获模式，通过上拉电阻充电
 * 3. 捕获阶段: 定时器测量电容充电到高电平的时间
 * 4. 数据处理: 充电时间与电容值成正比，手指触摸会增大电容
 */

#include "cap_touch.h"
#include <stddef.h>
#include <stdint.h>

/* 放电延时(单位:一个NOP) 500个NOP约10微秒 (48MHz系统时钟) */

/** 捕获超时时间(定时器计数值)
 * 8MHz定时器，每计数0.125us
 * 0xFFFF(65535) = 8.2ms - 太长！
 * 0x3FFF(16383) = 2.0ms - 推荐（更稳定）
 * 0x1FFF(8191)  = 1.0ms - 较快
 * 0x0FFF(4095)  = 0.5ms - 最快但可能不稳定
 */
#define CAPTURE_TIMEOUT 0x1FFF /* 2ms超时，平衡速度和稳定性 */

/**
 * @brief 触摸传感器状态枚举
 */
typedef enum {
    CAP_STATE_INIT,         /*!< 初始化状态 */
    CAP_STATE_DISCHARGE,    /*!< 电容放电状态 */
    CAP_STATE_WAIT_CAPTURE, /*!< 等待定时器捕获状态 */
    CAP_STATE_DONE          /*!< 完成状态 */
} cap_touch_state_t;

/**
 * @brief 触摸传感器参数结构体
 */
typedef struct {
    uint32_t          gpio_port;      /*!< GPIO端口(GPIOA/GPIOB/GPIOC等) */
    uint32_t          timer;          /*!< 定时器外设(TIMER0/TIMER2等) */
    uint32_t          rcu_gpio;       /*!< GPIO时钟 */
    uint32_t          rcu_timer;      /*!< 定时器时钟 */
    uint32_t          timer_channel;  /*!< 定时器通道 */
    uint32_t          timer_int_flag; /*!< 定时器中断标志 */
    uint32_t          gpio_pin;       /*!< GPIO引脚号 */
    uint32_t          gpio_af;        /*!< GPIO复用功能 */
    IRQn_Type         timer_irq;      /*!< 定时器IRQ编号 */
    cap_touch_state_t state;          /*!< 当前状态 */
    uint8_t           discharge_cnt;  /*!< 放电计数器 */
} cap_touch_pad_t;

/**
 * @brief GPIO指示引脚映射表
 *
 * 6个通道对应PB0-PB5
 */
static const uint32_t g_indicator_pins[6] = {
    GPIO_PIN_0, /* Channel 0 -> PB0 */
    GPIO_PIN_1, /* Channel 1 -> PB1 */
    GPIO_PIN_2, /* Channel 2 -> PB2 */
    GPIO_PIN_3, /* Channel 3 -> PB3 */
    GPIO_PIN_4, /* Channel 4 -> PB4 */
    GPIO_PIN_5  /* Channel 5 -> PB5 */
};

/**
 * @brief 触摸板数组配置
 *
 * 配置说明:
 * - 使用TIMER0的4个通道: CH0(PA0), CH1(PA1), CH2(PA2), CH3(PA3)
 * - 使用TIMER15单通道: CH0(PA6)
 * - 使用TIMER16单通道: CH0(PA7)
 *
 * 引脚分配:
 * Channel 0: PA0  - TIMER0_CH0  (AF2) - TOUCH_IN1
 * Channel 1: PA1  - TIMER0_CH1  (AF2) - TOUCH_IN2
 * Channel 2: PA2  - TIMER0_CH2  (AF2) - TOUCH_IN3
 * Channel 3: PA3  - TIMER0_CH3  (AF2) - TOUCH_IN4
 * Channel 4: PA6  - TIMER15_CH0 (AF5) - TOUCH_IN5
 * Channel 5: PA7  - TIMER16_CH0 (AF5) - TOUCH_IN6
 */
cap_touch_pad_t g_touch_pads[CAP_TOUCH_CHANNEL_COUNT] = {
    [0] = {.gpio_pin       = GPIO_PIN_0,
           .gpio_port      = GPIOA,
           .rcu_gpio       = RCU_GPIOA,
           .timer          = TIMER0,
           .rcu_timer      = RCU_TIMER0,
           .timer_channel  = TIMER_CH_0,
           .timer_int_flag = TIMER_INT_FLAG_CH0,
           .timer_irq      = TIMER0_Channel_IRQn,
           .gpio_af        = GPIO_AF_5,
           .state          = CAP_STATE_INIT},
    [1] = {.gpio_pin       = GPIO_PIN_1,
           .gpio_port      = GPIOA,
           .rcu_gpio       = RCU_GPIOA,
           .timer          = TIMER0,
           .rcu_timer      = RCU_TIMER0,
           .timer_channel  = TIMER_CH_1,
           .timer_int_flag = TIMER_INT_FLAG_CH1,
           .timer_irq      = TIMER0_Channel_IRQn,
           .gpio_af        = GPIO_AF_5,
           .state          = CAP_STATE_INIT},
    [2] = {.gpio_pin       = GPIO_PIN_2,
           .gpio_port      = GPIOA,
           .rcu_gpio       = RCU_GPIOA,
           .timer          = TIMER0,
           .rcu_timer      = RCU_TIMER0,
           .timer_channel  = TIMER_CH_2,
           .timer_int_flag = TIMER_INT_FLAG_CH2,
           .timer_irq      = TIMER0_Channel_IRQn,
           .gpio_af        = GPIO_AF_5,
           .state          = CAP_STATE_INIT},
    [3] = {.gpio_pin       = GPIO_PIN_3,
           .gpio_port      = GPIOA,
           .rcu_gpio       = RCU_GPIOA,
           .timer          = TIMER0,
           .rcu_timer      = RCU_TIMER0,
           .timer_channel  = TIMER_CH_3,
           .timer_int_flag = TIMER_INT_FLAG_CH3,
           .timer_irq      = TIMER0_Channel_IRQn,
           .gpio_af        = GPIO_AF_5,
           .state          = CAP_STATE_INIT},
    [4] = {.gpio_pin       = GPIO_PIN_6,
           .gpio_port      = GPIOA,
           .rcu_gpio       = RCU_GPIOA,
           .timer          = TIMER15,
           .rcu_timer      = RCU_TIMER15,
           .timer_channel  = TIMER_CH_0,
           .timer_int_flag = TIMER_INT_FLAG_CH0,
           .timer_irq      = TIMER15_IRQn,
           .gpio_af        = GPIO_AF_5,
           .state          = CAP_STATE_INIT},
    [5] = {.gpio_pin       = GPIO_PIN_7,
           .gpio_port      = GPIOA,
           .rcu_gpio       = RCU_GPIOA,
           .timer          = TIMER16,
           .rcu_timer      = RCU_TIMER16,
           .timer_channel  = TIMER_CH_0,
           .timer_int_flag = TIMER_INT_FLAG_CH0,
           .timer_irq      = TIMER16_IRQn,
           .gpio_af        = GPIO_AF_5,
           .state          = CAP_STATE_INIT}
};

/** 当前处理的触摸通道索引 */
static uint8_t g_current_channel = 0;

/** 触摸数据实例 */
capture_data_t g_touch_data = {.values = {0}, .timestamp = 0};

/** 数据采集完成回调函数指针 */
static cap_touch_data_ready_callback_t g_data_ready_callback = NULL;

/** 系统时间戳(微秒) */
static volatile uint64_t g_system_us = 0;

/* 私有函数声明 */
static void cap_touch_pad_init(cap_touch_pad_t *touch_pad);
static void cap_touch_pad_discharge(cap_touch_pad_t *touch_pad);
static void cap_touch_pad_start_capture(cap_touch_pad_t *touch_pad);
static void cap_touch_scan_next(void);

/**
 * @brief 初始化触摸板为GPIO输出模式(放电准备)
 */
static void cap_touch_pad_init(cap_touch_pad_t *touch_pad)
{
    /* 配置GPIO为输出模式，输出低电平进行放电 */
    gpio_mode_set(touch_pad->gpio_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, touch_pad->gpio_pin);
    /* 使用最低速度，降低噪声 */
    gpio_output_options_set(touch_pad->gpio_port, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_1, touch_pad->gpio_pin);
    gpio_bit_write(touch_pad->gpio_port, touch_pad->gpio_pin, RESET);

    /* 进入放电状态 */
    touch_pad->state         = CAP_STATE_DISCHARGE;
    touch_pad->discharge_cnt = 0;
}

/**
 * @brief 启动定时器输入捕获
 */
static void cap_touch_pad_start_capture(cap_touch_pad_t *touch_pad)
{
    /* 进入等待捕获状态 */
    touch_pad->state = CAP_STATE_WAIT_CAPTURE;
    // 先设置状态，再配置定时器，因为可能中断马上就来

    timer_ic_parameter_struct timer_icinitpara;

    /* 1. 先配置定时器输入捕获参数 */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    /* 硬件数字滤波：0x05表示需要5个连续采样点一致才认为信号有效
     * 采样频率 = 定时器时钟频率 / (prescaler * filter)
     * 0x00 = 无滤波
     * 0x01-0x03 = 轻度滤波（推荐用于低噪声环境）
     * 0x04-0x07 = 中度滤波（推荐用于中等噪声环境）
     * 0x08-0x0F = 强滤波（用于高噪声环境，会略微降低响应速度）
     */
    timer_icinitpara.icfilter = 0x03; /* 硬件滤波，过滤掉毛刺 */
    timer_input_capture_config(touch_pad->timer, touch_pad->timer_channel, &timer_icinitpara);

    /* 2. 清除定时器计数器 */
    timer_counter_value_config(touch_pad->timer, 0);

    /* 3. 清除中断标志 */
    timer_interrupt_flag_clear(touch_pad->timer, touch_pad->timer_int_flag);

    timer_interrupt_enable(touch_pad->timer, touch_pad->timer_int_flag);

    /* 5. 使能定时器输入捕获通道 */
    timer_channel_output_state_config(touch_pad->timer, touch_pad->timer_channel, TIMER_CCX_ENABLE);

    /* 6. 最后配置GPIO（此时定时器已准备好捕获上升沿）*/
    gpio_mode_set(touch_pad->gpio_port, GPIO_MODE_AF, GPIO_PUPD_NONE, touch_pad->gpio_pin);
    /* 降低GPIO速度，减少高频噪声耦合 */
    gpio_output_options_set(touch_pad->gpio_port, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_1, touch_pad->gpio_pin);
    gpio_af_set(touch_pad->gpio_port, touch_pad->gpio_af, touch_pad->gpio_pin);
}

/**
 * @brief 扫描下一个通道
 */
static void cap_touch_scan_next(void)
{
#if 0
    g_current_channel = (g_current_channel + 1) % CAP_TOUCH_CHANNEL_COUNT;

    /* 当完成一轮6个通道的采集后 */
    if (g_current_channel == 0) {
        /* 更新时间戳 */
        g_touch_data.timestamp = g_system_us;

        /* 调用回调函数通知数据采集完成 */
        if (g_data_ready_callback != NULL) { g_data_ready_callback(&g_touch_data); }
    }
#endif
}

/**
 * @brief 定时器输入捕获中断回调函数
 *
 * 当定时器捕获到触摸板充电完成时调用此函数
 * 需要在对应的定时器中断处理函数中调用
 */
void cap_touch_timer_capture_callback(uint32_t timer_periph, uint16_t channel)
{
    uint8_t i = g_current_channel;

    /* 标记当前通道回到初始状态 */
    g_touch_pads[i].state = CAP_STATE_DISCHARGE;

    /* 检查是否是当前通道的中断 */
    //   if (g_touch_pads[i].timer == timer_periph &&
    //       g_touch_pads[i].timer_channel == channel) {

    /* 读取捕获值 */
    g_touch_data.values[i] = timer_channel_capture_value_register_read(timer_periph, channel);

    /* 禁用定时器输入捕获通道 */
    timer_channel_output_state_config(timer_periph, channel, TIMER_CCX_DISABLE);

    // /* 禁用中断 */
    timer_interrupt_disable(timer_periph, g_touch_pads[i].timer_int_flag);

    /* ⚠️ 关键修复：先将当前通道GPIO切换为输出低电平进行放电 */
    gpio_mode_set(g_touch_pads[i].gpio_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, g_touch_pads[i].gpio_pin);
    gpio_output_options_set(g_touch_pads[i].gpio_port, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_1, g_touch_pads[i].gpio_pin);
    gpio_bit_write(g_touch_pads[i].gpio_port, g_touch_pads[i].gpio_pin, RESET);

    /* 扫描下一个通道 */
    cap_touch_scan_next();
    //   }
}

/**
 * @brief 处理单个触摸传感器的状态机（优化版）
 */
static cap_bool_t cap_touch_process_pad(cap_touch_pad_t *touch_pad)
{
    switch (touch_pad->state) {
    case CAP_STATE_INIT: cap_touch_pad_init(touch_pad); return CAP_TRUE;

    case CAP_STATE_DISCHARGE: cap_touch_pad_start_capture(touch_pad); return CAP_FALSE;

    case CAP_STATE_WAIT_CAPTURE:

        /* 检查是否超时（降低检查频率以减少CPU占用） */
        if (timer_counter_read(touch_pad->timer) >= CAPTURE_TIMEOUT) {
            //  否则保持上次的值不变
            timer_interrupt_disable(touch_pad->timer, touch_pad->timer_int_flag);
            timer_counter_value_config(touch_pad->timer, 0);
            timer_channel_output_state_config(touch_pad->timer, touch_pad->timer_channel, TIMER_CCX_DISABLE);

            cap_touch_pad_init(touch_pad);
            cap_touch_scan_next();
        }
        return CAP_FALSE;

    default: touch_pad->state = CAP_STATE_INIT; return CAP_FALSE;
    }
}

/**
 * @brief 运行触摸检测过程
 *
 * 该函数应在主循环中定期调用
 */
void cap_touch_process(void)
{
    cap_touch_process_pad(&g_touch_pads[g_current_channel]);
}

/**
 * @brief 定时器配置
 */
static void timer_config(uint32_t timer_periph)
{
    timer_parameter_struct timer_initpara;

    /* 初始化定时器 */
    timer_deinit(timer_periph);

    /* 配置定时器基本参数 */
    timer_initpara.prescaler         = 1; /* 48MHz / 6 = 8MHz，每计数0.125us */
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(timer_periph, &timer_initpara);

    /* 使能定时器 */
    timer_enable(timer_periph);
}

/**
 * @brief 初始化电容触摸模块
 */
void cap_touch_init(void)
{
    /* 使能GPIO时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    /* 使能定时器时钟 */
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER15);
    rcu_periph_clock_enable(RCU_TIMER16);

    /* 配置定时器 */
    timer_config(TIMER0);
    timer_config(TIMER15);
    timer_config(TIMER16);

    /* 配置NVIC */
    nvic_irq_enable(TIMER0_Channel_IRQn, 3);
    nvic_irq_enable(TIMER15_IRQn, 3);
    nvic_irq_enable(TIMER16_IRQn, 3);

    /* 初始化第一个通道 */
    cap_touch_pad_init(&g_touch_pads[0]);
}

/**
 * @brief 注册数据采集完成回调函数
 */
void cap_touch_register_data_ready_callback(cap_touch_data_ready_callback_t callback)
{
    g_data_ready_callback = callback;
}

/**
 * @brief SysTick中断处理函数 - 用于时间戳
 * 应在systick中断中每1ms调用一次
 */
void cap_touch_systick_handler(void)
{
    g_system_us += 1000;
}

/**
 * @brief 初始化触摸指示GPIO
 *
 * 配置PB0-PB5为推挽输出，初始为低电平
 */
void cap_touch_gpio_indicator_init(void)
{
    /* 使能GPIOB时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);

    /* 配置PB0-PB5为推挽输出模式 */
    for (uint8_t i = 0; i < 6; i++) {
        gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, g_indicator_pins[i]);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_1, g_indicator_pins[i]);
        gpio_bit_write(GPIOB, g_indicator_pins[i], RESET); /* 初始为低电平 */
    }
}

void cap_test_gpio_toggle(void)
{
    gpio_bit_toggle(GPIOB, g_indicator_pins[1]);
}
