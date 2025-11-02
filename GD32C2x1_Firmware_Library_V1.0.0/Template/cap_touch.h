/**
 * @file cap_touch.h
 * @brief 电容式触摸传感器模块头文件 - GD32C2x1版本
 * @version 1.0
 * @date 2025-11-01
 */

#ifndef CAP_TOUCH_H_
#define CAP_TOUCH_H_

#include "gd32c2x1.h"
#include <stdio.h>
#include <stdint.h>

/** 触摸通道数量定义 */
#define CAP_TOUCH_CHANNEL_COUNT 6  /* 启用6个通道 */

/** 返回值定义 */
typedef enum {
    CAP_OK = 0,
    CAP_ERROR = 1
} cap_err_t;

/** 布尔类型定义 */
typedef enum {
    CAP_FALSE = 0,
    CAP_TRUE = 1
} cap_bool_t;

#pragma pack(1)
/**
 * @brief 触摸数据结构体，包含所有通道值和时间戳
 */
typedef struct {
    uint32_t values[CAP_TOUCH_CHANNEL_COUNT]; /*!< 触摸值数组，索引对应通道号 */
    uint64_t timestamp;                       /*!< 数据采集时间戳(微秒) */
} capture_data_t;
#pragma pack()

/**
 * @brief 数据采集完成回调函数类型
 * @param data_packet 指向完整数据包的指针
 */
typedef void (*cap_touch_data_ready_callback_t)(capture_data_t *data_packet);

/**
 * @brief 初始化电容触摸模块
 */
void cap_touch_init(void);

/**
 * @brief 运行触摸检测过程
 *
 * 该函数应在主循环中定期调用，采用非阻塞方式
 */
void cap_touch_process(void);

/**
 * @brief 获取指定通道的触摸值
 *
 * @param channel 触摸通道索引(0-5)
 * @return uint32_t 触摸值，值越大表示电容越大
 */
uint32_t cap_touch_get_value(uint8_t channel);

/**
 * @brief 获取所有通道的触摸值数组指针
 *
 * @return uint32_t* 指向触摸值数组的指针，便于数据传输
 */
uint32_t *cap_touch_get_values_array(void);

/**
 * @brief 获取完整的触摸数据包指针
 *
 * @return capture_data_t* 指向完整数据包的指针，包含values和timestamp
 */
capture_data_t *cap_touch_get_data_packet(void);

/**
 * @brief 注册数据采集完成回调函数
 *
 * @param callback 回调函数指针，当6个通道数据采集完成时会调用此函数
 */
void cap_touch_register_data_ready_callback(cap_touch_data_ready_callback_t callback);

/**
 * @brief 从FIFO中读取触摸数据
 *
 * @param data_buffer 存储读取数据的缓冲区指针
 * @return cap_err_t CAP_OK:成功读取一个数据包, CAP_ERROR:FIFO为空或读取失败
 */
cap_err_t cap_touch_fifo_read(capture_data_t *data_buffer);

/**
 * @brief 获取FIFO中可读取的数据包数量
 *
 * @return uint16_t 可读取的完整数据包数量
 */
uint16_t cap_touch_fifo_get_count(void);

/**
 * @brief 清空触摸数据FIFO
 */
void cap_touch_fifo_clear(void);

/**
 * @brief SysTick中断处理函数 - 用于时间戳
 * 应在systick中断中每1ms调用一次
 */
void cap_touch_systick_handler(void);

/**
 * @brief 初始化触摸指示GPIO (PB0-PB5)
 * 
 * 当检测到触摸时，对应的GPIO会输出高电平
 */
void cap_touch_gpio_indicator_init(void);

/**
 * @brief 更新触摸指示GPIO状态
 * 
 * @param channel 通道号(0-5)
 * @param threshold 触摸检测阈值
 * 
 * 当通道值超过阈值时，对应GPIO输出高电平，否则输出低电平
 */
void cap_touch_update_gpio_indicator(uint8_t channel, uint32_t threshold);

/**
 * @brief 定时器输入捕获中断回调函数
 * 
 * @param timer_periph 定时器外设
 * @param channel 定时器通道
 * 
 * 当定时器捕获到触摸板充电完成时调用此函数
 * 需要在对应的定时器中断处理函数中调用
 */
void cap_touch_timer_capture_callback(uint32_t timer_periph, uint16_t channel);

void cap_test_gpio_toggle(void);
#endif /* CAP_TOUCH_H_ */
