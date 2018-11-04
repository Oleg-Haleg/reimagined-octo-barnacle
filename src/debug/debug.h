#pragma once

//#include "mcu_support_package/inc/stm32f10x.h"
#include <stdint.h>

void init_USART1(void);
void USART1_IRQHandler(void);
void TIM1_IRQHandler(void);
  