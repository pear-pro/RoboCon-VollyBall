#include "buzzer.h"
#include "stm32_hal_legacy.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "tim.h"

void buzzer_on(uint16_t psc,uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim12, psc);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, pwm);



}
void buzzer_off()
{
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0);
}
void buzzer_startsing(void)
{
    buzzer_on(4, 1000);
    HAL_Delay(500);
    buzzer_off();
    HAL_Delay(500);
    buzzer_on(2, 1000);
    HAL_Delay(500);
    buzzer_off();
    HAL_Delay(1000);
    buzzer_on(2, 1000);
    HAL_Delay(500);
    buzzer_off();
    HAL_Delay(500);
    buzzer_on(2, 1000);
    HAL_Delay(500);
    buzzer_off();
    HAL_Delay(500);
    buzzer_on(2, 1000);
    HAL_Delay(500);
    buzzer_off();

}