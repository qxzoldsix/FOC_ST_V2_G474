#ifndef __KEY_DRIVER_H__
#define __KEY_DRIVER_H__

#include "main.h"

/* 按键永久屏蔽屏蔽值 */
#define key_alway_shield 0xFFFFFFFF

/* 按键时间定义 */
#define click_one_wait_for_double_time 500       // 双击两次之间最大间隔时间 单位ms
#define long_press_time 1600                     // 识别为长按的时间 单位ms
#define long_press_effective_interval_time 10000 // 长按时每次触发间隔 单位ms

/* 按键数量与编号 */
typedef enum {
	key_sw1,
	key_sw2,
	key_sw3,
	key_sw4,
	key_sw5,
	key_sw6,
	key_num,                  // 总按键数量
} key_t;

/* 按键状态机定义 */
typedef enum 
{
	key_release,                       // 按键释放状态
	key_short_pressing,                // 按键短按状态
	key_double_pressing,               // 按键双击状态
	key_click_one_wait_for_double,     // 按键单击等待双击状态
	key_long_pressing,                 // 按键长按状态
	key_state_machine_num,             // 状态机数量 辅助计数
} key_state_machine_t;

/* 按键键值定义 */
typedef enum {
	key_none = key_state_machine_num,  // 无按键
	key_click_one,                     // 单击
	key_click_double,                  // 双击
	key_long_press,                    // 长按
} key_value_t;

/* 按键防抖滤波结构体 */
typedef struct {
	uint32_t hold_time;             // 按键保持时间
	uint64_t time_point;            // 时间点
	uint32_t trigger_variable;      // 触发变量
	uint8_t last_variable_state;    // 上一次变量状态
	uint8_t result;                 // 滤波结果
} hold_filter_t;

/* 按键驱动层结构体 */
typedef struct {
	hold_filter_t bsp_hold_filter;    	// 硬件层按键滤波
	uint32_t shield;                    // 屏蔽时间 0=完全屏蔽 1=使能
	uint32_t press_time;                // 按下开始时间
	uint32_t release_time;              // 释放开始时间
	key_state_machine_t state_machine;  // 按键当前状态机
	key_value_t key_value;              // 最终识别键值
} key_driver_t;

// 按键驱动初始化
void key_driver_init(void);
// 按键状态检测
void key_check(void);
// 按键数组初始化
void key_array_init(void);
// 按键处理主函数（单击/双击/长按）
void key_process(void);
// 按键保持滤波
uint8_t hold_filter(hold_filter_t *filter_p, uint32_t variable);

extern key_driver_t key_array[];   // 按键实例数组 (4 keys)

#endif /* __KEY_DRIVER_H__ */


