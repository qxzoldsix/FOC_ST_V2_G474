#include "key_bsp.h"
#include "gpio.h"

static key_bsp_gpio_t* key_ports[] = { 
  GPIOC,
	GPIOC,
	GPIOC,
	GPIOC
};

static uint16_t key_pins[] = {
	GPIO_PIN_9,
	GPIO_PIN_8,
	GPIO_PIN_7,
	GPIO_PIN_6,
 
};

key_bsp_t key_bsp;

/**
***********************************************************************
* @brief:      key_bsp_init(void)
* @param:		   void
* @retval:     void
* @details:    ¡ã¡ä?¨¹¨®2?t3?¨º??¡¥
***********************************************************************
**/
void key_bsp_init(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	/* KEY GPIO Periph clock enable */
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//	__HAL_RCC_GPIOC_CLK_ENABLE();

//	for (int i=0; i<sizeof(key_ports)/sizeof(key_ports[0]); i++) 
//	{
//		GPIO_InitStruct.Pin = key_pins[i];
//		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//		GPIO_InitStruct.Pull = GPIO_PULLUP;
//		HAL_GPIO_Init(key_ports[i], &GPIO_InitStruct);
//	}
}
/**
***********************************************************************
* @brief:      key_bsp_read_pin(key_bsp_gpio_t *port, uint16_t pin)
* @param:		   port: GPIO   pin: ¨°y??o?
* @retval:     ¡Á¡ä¨¬?¡À¨º??
* @details:    ?¨¢¨¨?¡ã¡ä?¨¹IO¡Á¡ä¨¬?
***********************************************************************
**/
key_bsp_state key_bsp_read_pin(key_bsp_gpio_t *port, uint16_t pin)
{
	key_bsp_state bit_status;
	
	bit_status = HAL_GPIO_ReadPin(port, pin);
	
	return bit_status;
}
/**
***********************************************************************
* @brief:      get_key_bsptick(void)
* @param:		   void
* @retval:     ¦Ì?¡äe¨º¡À??
* @details:    ??¨¨?¡ã¡ä?¨¹¦Ì?¦Ì?¡äe¨º¡À??
***********************************************************************
**/
uint32_t get_key_bsptick(void)
{
	uint32_t key_tick;
	
	key_tick = HAL_GetTick();
	
  return key_tick;
}
