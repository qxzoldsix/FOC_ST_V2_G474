#include "key_driver.h"
#include "key_bsp.h"

key_driver_t key_array[key_num];

/**
***********************************************************************
* @brief:      key_driver_init(void)
* @param:		   void
* @retval:     void
* @details:    按键驱动初始化
***********************************************************************
**/
void key_driver_init(void)
{
	key_bsp_init();
	key_array_init();
}
/**
***********************************************************************
* @brief:      key_check(void)
* @param:		   void
* @retval:     void
* @details:    检测按键是否被按下
***********************************************************************
**/
void key_check(void)
{
	key_driver_t *key_p;
	key_p = &key_array[key_sw1];
	hold_filter(&key_p->bsp_hold_filter, key_1); // 获取滤波后的按键状态
	key_p = &key_array[key_sw2];
	hold_filter(&key_p->bsp_hold_filter, key_2); // 获取滤波后的按键状态
	key_p = &key_array[key_sw3];
	hold_filter(&key_p->bsp_hold_filter, key_3); // 获取滤波后的按键状态
	key_p = &key_array[key_sw4];
	hold_filter(&key_p->bsp_hold_filter, key_4); // 获取滤波后的按键状态
//	key_p = &key_array[key_sw5];
//	hold_filter(&key_p->bsp_hold_filter, key_5); // 获取滤波后的按键状态
//	key_p = &key_array[key_sw6];
//	hold_filter(&key_p->bsp_hold_filter, key_6); // 获取滤波后的按键状态
}
/**
***********************************************************************
* @brief:      key_array_init(void)
* @param:		   void
* @retval:     void
* @details:    按键数组初始化
***********************************************************************
**/
void key_array_init(void)
{
	key_driver_t *key_p;
	key_p = &key_array[key_sw1];
	key_p->bsp_hold_filter.hold_time = 10;
	key_p->bsp_hold_filter.time_point = 0;
	key_p->bsp_hold_filter.trigger_variable = 0;	// 0电平触发
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;

	key_p = &key_array[key_sw2];
	key_p->bsp_hold_filter.hold_time = 10;
	key_p->bsp_hold_filter.time_point = 0;
	key_p->bsp_hold_filter.trigger_variable = 0;
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;

	key_p = &key_array[key_sw3];
	key_p->bsp_hold_filter.hold_time = 10;
	key_p->bsp_hold_filter.time_point = 0;
	key_p->bsp_hold_filter.trigger_variable = 0;
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;
	
	key_p = &key_array[key_sw4];
	key_p->bsp_hold_filter.hold_time = 10;
	key_p->bsp_hold_filter.time_point = 0;
	key_p->bsp_hold_filter.trigger_variable = 0;
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;
	
}
/**
***********************************************************************
* @brief:      hold_filter(hold_filter_t *filter_p, uint32_t variable)
* @param:		   filter_p: 按键触发滤波结构体指针
* @param:		   variable: 按键状态标志
* @retval:     滤波后按键状态
* @details:    按键稳态滤波处理
***********************************************************************
**/
uint8_t hold_filter(hold_filter_t *filter_p, uint32_t variable)
{
	uint8_t variable_state = 0;
	uint32_t pass_time, system_time_temp;
	
	system_time_temp = get_key_bsptick();
	pass_time = system_time_temp - filter_p->time_point;

	if (filter_p->trigger_variable == variable)
	{
		variable_state = 1;
	}

	if (variable_state == filter_p->last_variable_state) 
	{
		if (filter_p->last_variable_state) 
		{
			if (pass_time > filter_p->hold_time && filter_p->result == 0) 
			{
				filter_p->result = 1;
			}
		} 
		else 
		{
			if (pass_time > filter_p->hold_time && filter_p->result == 1) 
			{
				filter_p->result = 0;
			}
		}
	} 
	else 
	{
		if (pass_time >= filter_p->hold_time) 
		{
			filter_p->time_point = system_time_temp;
		} 
		else 
		{
			filter_p->time_point = system_time_temp - (filter_p->hold_time - pass_time);
		}
	}
	filter_p->last_variable_state = variable_state;
	
	return filter_p->result;
}
/**
***********************************************************************
* @brief:      key_process(void)
* @param:		   void
* @retval:     void
* @details:    按键处理函数(长按、短按、单击、双击)
***********************************************************************
**/
void key_process(void)
{
	uint8_t i;
	key_driver_t *key_p;

	key_check();
	
	for (i = 0; i < key_num; i++) 
	{
		key_p = &key_array[i];
		key_p->key_value = key_none;
		if (get_key_bsptick() > key_p->shield && key_p->shield != key_alway_shield) 
		{
			switch (key_p->state_machine) 
			{
				case key_release:
				if (key_p->bsp_hold_filter.result)  // 按键按下
				{
					key_p->press_time = get_key_bsptick();
					key_p->state_machine = key_short_pressing;
				}
					break;
				
				case key_short_pressing:
					if (key_p->bsp_hold_filter.result)  // 持续按住
					{
						if (get_key_bsptick() - key_p->press_time > long_press_time)  // 按住时间超过长按判定时间
						{
							key_p->press_time = get_key_bsptick();
							key_p->key_value = key_long_press; // 长按触发
							key_p->state_machine = key_long_pressing;
						}
					} 
					else // 松开
					{ 
						key_p->release_time = get_key_bsptick();
						key_p->state_machine = key_click_one_wait_for_double;
					}
					break;
					
				case key_click_one_wait_for_double:
					if (get_key_bsptick() - key_p->press_time < click_one_wait_for_double_time)  // 双击等待时间内
					{
						if (key_p->bsp_hold_filter.result)  // 再次按下
						{
							key_p->key_value = key_click_double; // 双击触发
							key_p->state_machine = key_double_pressing;
						}
					} 
					else  // 超时
					{
						key_p->key_value = key_click_one; // 单击触发
						key_p->state_machine = key_release;
					}
					break;

				case key_double_pressing: // 双击后等待松开
					if (!key_p->bsp_hold_filter.result) 
					{
						key_p->state_machine = key_release;
					}
					break;

				case key_long_pressing:
					if (key_p->bsp_hold_filter.result)  // 持续按住
					{	
						if ((get_key_bsptick() - key_p->press_time) % long_press_effective_interval_time == 0) 
						{
							key_p->key_value = key_long_press; // 长按触发
						}
					} 
					else  // 松开后复位
					{	
						key_p->state_machine = key_release;
					}
					break;

				default:
					break;
			}
		} 
		else 
		{
			if (!key_p->bsp_hold_filter.result) 
			{
				key_p->state_machine = key_release;
			}
		}
	}
}
