#ifndef __LED_OPS_H__
#define __LED_OPS_H__

#include <stdint.h>
#include "stm32f4xx_hal.h" 

typedef enum
{
   LED_OK =0,
   LED_ERROR = -1,
} LED_StatusTypeDef;

typedef enum
{
  LED_STATE_OFF = 0,
  LED_STATE_ON,
}LED_StateTypeDef;

typedef struct
{
  GPIO_TypeDef *port;  
  uint16_t pin;
  uint8_t active_level; // 1 for active high, 0 for active low
  LED_StateTypeDef state; // Current state of the LED
} LED_HandleTypedef;

LED_StatusTypeDef LED_Init(LED_HandleTypedef *hled);
LED_StatusTypeDef LED_SetState(LED_HandleTypedef *hled, LED_StateTypeDef state);
LED_StatusTypeDef LED_Toggle(LED_HandleTypedef *hled);
LED_StateTypeDef LED_GetState(LED_HandleTypedef *hled);

#endif