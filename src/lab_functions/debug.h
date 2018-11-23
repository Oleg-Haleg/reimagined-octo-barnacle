#pragma once

//#include "mcu_support_package/inc/stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

void debug_init(void);
int16_t mem_speed (int16_t speed, bool memorize);
void USART1_IRQHandler(void);
void SysTick_Handler(void);
void TIM1_IRQHandler(void);
