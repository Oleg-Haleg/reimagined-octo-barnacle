/*******************************************************************************************************
Definition: File to communicate with the terminal
Developer: Trishin Vadim
Notes: 
*******************************************************************************************************/
/*******************************************************************************************************
Описание: Файл для связи с терминалом
Разработчик: Тришин Вадим
Заметки: 
*******************************************************************************************************/
#include "debug.h"
#include "control.h"
#include "motor_speed.h"
#include <stm32f10x.h>
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"

/***************************************************************************************************
Local defines
***************************************************************************************************/
/***************************************************************************************************
Локальные дефайны
***************************************************************************************************/

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

/***************************************************************************************************
Global functions
***************************************************************************************************/
/***************************************************************************************************
Глобальные функции
***************************************************************************************************/

/**************************************************************************************************
Definition: Initializating USART 
Arguments: No
Return:   No
Notes: To communicate with the terminal
**************************************************************************************************/
/**************************************************************************************************
Описание: Инициализация USART 
Аргументы: Нет
Возврат:   Нет
Замечания: Для обмена данными с терминалом
**************************************************************************************************/
void debug_init(void)
{
  //*************** Initialize pins for USART
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
  GPIO_InitTypeDef structGPIO;

  structGPIO.GPIO_Speed = GPIO_Speed_2MHz;
  // USART1_TX
  structGPIO.GPIO_Pin  = GPIO_Pin_9;
  structGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &structGPIO);
  // USART1_RX
  structGPIO.GPIO_Pin  = GPIO_Pin_10; 
  structGPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &structGPIO);
	
  //*************** USART initialization
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);
  USART_InitTypeDef structUSART;
  
  USART_StructInit(&structUSART);
  // Is it correct baud?
  structUSART.USART_BaudRate = 9600;
  USART_Init(USART1, &structUSART);
  
  // Turns on RX interrupt
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
//  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  NVIC_SetPriority(USART1_IRQn, 2);
  NVIC_EnableIRQ(USART1_IRQn);
  
  USART_Cmd(USART1, ENABLE);
  
  //*************** Initializating SysTick
  // One timer tick per second
//  SysTick_Config(SystemCoreClock/1);
//  NVIC_SetPriority(SysTick_IRQn, 4);
//  NVIC_EnableIRQ(SysTick_IRQn);

  //*************** Initializating TIM
  // Frequency 1ms (1 timer tick per 1 milisecond)
	const uint16_t freq = 1000;
  // Period 1s (period = 1000 ticks of timer i.e. 1 seconds)
  const uint16_t period = 1000;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  
	TIM_TimeBaseInitTypeDef structTIM;
  TIM_TimeBaseStructInit(&structTIM);
	structTIM.TIM_Period = period - 1;
	structTIM.TIM_Prescaler = SystemCoreClock / freq - 1;
	TIM_TimeBaseInit(TIM1, &structTIM);
	 
  // Configure timer on interrupting by update 
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
//	NVIC_SetPriority(TIM1_UP_IRQn, 2);
  // Enable interrupting from TIM1
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	
  // Turns on timer
	TIM_Cmd(TIM1, ENABLE);
}

/**************************************************************************************************
Definition: Memorize speed
Arguments: Speed (-2048..2047), memorizing flag (if false just returns value of speed)
Return:   Speed value
Notes:
**************************************************************************************************/
/**************************************************************************************************
Описание: Запоминание скорости
Аргументы: Скорость от (-2048 до 2047)
           Флаг запоминания (если false - просто возвращает значение скорости)
Возврат:   Значение скорости
Замечания:
**************************************************************************************************/
int16_t mem_speed (int16_t speed, bool memorize)
{
  static int16_t memorizedSpeed;
  if (memorize)
  {
    if (speed >= -2048 && speed <= 2047)
    {
      memorizedSpeed = speed;
    }
  }
  return memorizedSpeed;
}
/**************************************************************************************************
Definition: Interrupt from USART 
Arguments: No
Return:   No
Notes: Waiting for new message and meorizing new speed if value in message = -2048..2047
**************************************************************************************************/
/**************************************************************************************************
Описание: Прерывание от USART
Аргументы: Нет
Возврат:   Нет
Замечания: Ожидание нового сообщения и запоминание скорости, если значение в сообщении = -2048..2047
**************************************************************************************************/
void USART1_IRQHandler(void)
{
  // Flag to remember if previous message was 'b'
  static bool waitForVelocity = false;
  
  // Make received data signed (must work, but have little doubths)
  int16_t receivedData = (int16_t)(USART_ReceiveData(USART1));
  if (waitForVelocity)
  {
    if (receivedData >= -2048 && receivedData <= 2047)
    {
      mem_speed(receivedData, true);
    }
    NVIC_EnableIRQ(SysTick_IRQn);
//    TIM_Cmd(TIM1, ENABLE);
    waitForVelocity = false;
  }
  else if (receivedData == 'b')
  {
    NVIC_DisableIRQ(SysTick_IRQn);
//    TIM_Cmd(TIM1, DISABLE);
    waitForVelocity = true;
  }
}

/**************************************************************************************************
Definition: Interrupt from SysTick 
Arguments: No
Return:   No
Notes: Every second sends message by USART
**************************************************************************************************/
/**************************************************************************************************
Описание: Прерывание от SysTick
Аргументы: Нет
Возврат:   Нет
Замечания: Посылает сообщение по USART каждую секунду
**************************************************************************************************/
void SysTick_Handler(void)
{
  int16_t speed = motor_speed_getSpeed();
  USART_SendData(USART1, speed);
  //Clear interrupt bit
//  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

void TIM1_IRQHandler(void)
{
  int16_t speed = motor_speed_getSpeed();
  USART_SendData(USART1, speed);    
}
