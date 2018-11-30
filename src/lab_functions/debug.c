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
Локальные дефайны
***************************************************************************************************/

/***************************************************************************************************
Локальные типы данных
***************************************************************************************************/

/***************************************************************************************************
Локальные переменные файла
***************************************************************************************************/

/***************************************************************************************************
Глобальные функции
***************************************************************************************************/

/**************************************************************************************************
Описание: Инициализация USART 
Аргументы: Нет
Возврат:   Нет
Замечания: Для обмена данными с терминалом
**************************************************************************************************/
void debug_init(void)
{
  //**** Инициализация USART
  //** Шины
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);
  //** Пины
  GPIO_InitTypeDef structGPIO;
  GPIO_StructInit(&structGPIO);
  // USART1_TX
  structGPIO.GPIO_Pin  = GPIO_Pin_9;
  structGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &structGPIO);
  // USART1_RX
  structGPIO.GPIO_Pin  = GPIO_Pin_10; 
  structGPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &structGPIO);
	//** USART
  USART_InitTypeDef structUSART;
  USART_StructInit(&structUSART);
  // Is it correct baud?
  structUSART.USART_BaudRate = 115200;
  USART_Init(USART1, &structUSART);
  // Прерывание на получении
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
  NVIC_EnableIRQ(USART1_IRQn);
  USART_Cmd(USART1, ENABLE);
  //**** Initializating SysTick
  // 1 тик = 1мс
  SysTick_Config(SystemCoreClock/10);
  NVIC_EnableIRQ(SysTick_IRQn);
}

/**************************************************************************************************
Описание: Запоминание скорости
Аргументы: 
* speed - скорость от (-2048 до 2047)
* memorize - флаг запоминания (если false - просто возвращает значение скорости)
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
    // Если предыдущее значение было 'b'
    mem_speed(receivedData, true);
    NVIC_EnableIRQ(SysTick_IRQn);
    waitForVelocity = false;
  }
  else if (receivedData == 'b')
  {
    NVIC_DisableIRQ(SysTick_IRQn);
    waitForVelocity = true;
  }
}

/**************************************************************************************************
Описание: Прерывание от SysTick
Аргументы: Нет
Возврат:   Нет
Замечания: Посылает сообщение по USART каждую секунду
**************************************************************************************************/
void SysTick_Handler(void)
{
  static uint8_t counter = 0;
  // 10 тиков таймера = 1с
  if (++counter == 10)
  {
    int16_t speed = motor_speed_getSpeed();
    USART_SendData(USART1, speed);
    counter = 0;
  }
}
