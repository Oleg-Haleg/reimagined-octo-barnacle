#pragma once

//#include "mcu_support_package/inc/stm32f10x.h"
#include <stdint.h>

// Коэффиценты ПИД
typedef struct
{
  // Пропорциональный
  float coefProp;
  // Интегальный
  float coefInt;
  // Дифференциальный
  float coefDif;
} params_PID;

params_PID init_PID (float kP, float kI, float kD);
int16_t control_run(params_PID coefs, int16_t mistake);
