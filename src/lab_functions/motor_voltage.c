/*******************************************************************************************************
Definition: File to set new voltage on motor
Developer: Trishin Vadim
Notes: 
*******************************************************************************************************/
/*******************************************************************************************************
Описание: Файл для установки напряжения на мотор
Разработчик: Тришин Вадим
Заметки: 
*******************************************************************************************************/
#include "motor_voltage.h"
#include <stm32f10x.h>
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

/***************************************************************************************************
Local defines
***************************************************************************************************/
/***************************************************************************************************
Локальные дефайны
***************************************************************************************************/
#define TIMER_PRESCALER		720
#define EXT_TIM_PULSE 	150
#define TIM_PULSE		50
/***************************************************************************************************
Local data types
***************************************************************************************************/
/***************************************************************************************************
Локальные типы данных
***************************************************************************************************/

/***************************************************************************************************
File local variables
***************************************************************************************************/
/***************************************************************************************************
Локальные переменные файла
***************************************************************************************************/
uint16_t previousState;
GPIO_InitTypeDef port;
TIM_TimeBaseInitTypeDef timer;
TIM_OCInitTypeDef timerPWM;
uint16_t buttonPreviousState;
/***************************************************************************************************
Global functions
***************************************************************************************************/
/***************************************************************************************************
Глобальные функции
***************************************************************************************************/
/**************************************************************************************************
Описание: Инициализация пинов под ШИМ
Аргументы: Нет
Возврат:   Нет
Замечания: Нет
**************************************************************************************************/
void PWM_init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 
    GPIO_InitTypeDef port;
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = GPIO_Pin_8;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);
	
	  const uint16_t period = 1000; // Period 1s (period = 1000 ticks of timer i.e. 1 seconds)
		const uint16_t freq = 1000; // Frequency 1ms (1 timer tick per 1 milisecond)
 
		TIM_TimeBaseInitTypeDef structTIM;
		TIM_TimeBaseStructInit(&structTIM);
		structTIM.TIM_Period = period - 1;
		structTIM.TIM_Prescaler = SystemCoreClock / freq - 1;
		TIM_TimeBaseInit(TIM1, &structTIM);	
 
    TIM_OCInitTypeDef timerPWM;
    timerPWM.TIM_Pulse = 50;
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM1, &timerPWM);
	
		// Configure timer on interrupting by update 
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
		//	NVIC_SetPriority(TIM1_UP_IRQn, 2);
		// Enable interrupting from TIM1
		NVIC_EnableIRQ(TIM1_UP_IRQn);
	
		// Turns on timer
		TIM_Cmd(TIM1, ENABLE);
}
/**************************************************************************************************
Описание: Инициализация USART 
Аргументы: Новое значение напряжения на моторе в диапазоне от -1000 до 1000
Возврат:   Нет
Замечания: Измеряется в мВ ?
**************************************************************************************************/
void TIM1_IRQHandler()
{	

} 
void motor_voltage_setVoltage(int16_t voltage)
{
}
<<<<<<< HEAD

// классический ассерт для STM32
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t * file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
     
    (void)file;
    (void)line;

    __disable_irq();
    while(1)
    {
        // это ассемблерная инструкция "отладчик, стой тут"
        // если вы попали сюда, значит вы ошиблись в параметрах. Смотрите в call stack
        __BKPT(0xAB);
    }
}
#endif
=======
>>>>>>> aca289fd494e608515624d4abe1bf87572791c6b
