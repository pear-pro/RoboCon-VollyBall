#include "led_ops.h"

LED_StatusTypeDef LED_Init(LED_HandleTypedef *hled)
{
    if (hled == NULL || hled->port == NULL || hled->pin == 0) {
        return LED_ERROR;
    }

    return LED_SetState(hled, LED_STATE_OFF);
}

LED_StatusTypeDef LED_SetState(LED_HandleTypedef *hled, LED_StateTypeDef state)
{
    if(hled == NULL || hled->port ==NULL){
        return LED_ERROR;
    }
    hled->state = state;
    HAL_GPIO_WritePin(hled->port, hled->pin, hled->active_level 
                                                             ? (state == LED_STATE_ON ? GPIO_PIN_SET : GPIO_PIN_RESET) 
                                                             : (state == LED_STATE_ON ? GPIO_PIN_RESET : GPIO_PIN_SET));
    return LED_OK;
}

LED_StatusTypeDef LED_Toggle(LED_HandleTypedef *hled)
{
    if(hled ==NULL || hled->port ==NULL)
    {
        return LED_ERROR;
    }
    HAL_GPIO_TogglePin(hled->port, hled->pin);
    hled->state = hled->state == LED_STATE_ON ? LED_STATE_OFF : LED_STATE_ON;
    return LED_OK;
}

LED_StateTypeDef LED_GetState(LED_HandleTypedef *hled)
{
    if (hled == NULL) {
        return LED_STATE_OFF;
    }

    return hled->state;
}