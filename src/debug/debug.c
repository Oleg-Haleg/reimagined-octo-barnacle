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
#include <stm32f10x.h>
//#include "stm32f10x_usart.h"

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
void init_USART1(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	GPIO_InitTypeDef structGPIO;
  //GPIO_StructInit(&structGPIO);
	
	structGPIO.GPIO_Speed = GPIO_Speed_2MHz;
	
  // USART1_TX, TIM1_CH2
  structGPIO.GPIO_Pin  = GPIO_Pin_9;
  structGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &structGPIO);
  // USART1_RX, TIM1_CH3
  structGPIO.GPIO_Pin  = GPIO_Pin_10; 
  structGPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &structGPIO);
	
  // USART initialization
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);
  
  USART_InitTypeDef structUSART;
  
  USART_StructInit(&structUSART);
  structUSART.USART_BaudRate = 9600; // Is it correct baud?
  USART_Init(USART1, &structUSART);
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // Turns on RX interrupt
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
  
  USART_Cmd(USART1, ENABLE);
}

/**************************************************************************************************
Definition: Initializating TIM 
Arguments: No
Return:   No
Notes:
**************************************************************************************************/
/**************************************************************************************************
Описание: Инициализация TIM 
Аргументы: Нет
Возврат:   Нет
Замечания:
**************************************************************************************************/
void init_TIM1(void)
{
	const uint16_t period = 1000;
	const uint16_t freq = 1000;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  
	TIM_TimeBaseInitTypeDef structTIM;
    
	structTIM.TIM_ClockDivision = TIM_CKD_DIV1;
	structTIM.TIM_CounterMode = TIM_CounterMode_Up;
	structTIM.TIM_Period = period - 1; // period 1ms ?
	structTIM.TIM_Prescaler = SystemCoreClock / freq - 1; // frequency 1ms
	structTIM.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM1, &structTIM);
	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	
	//NVIC_SetPriority(TIM1_UP_IRQn, 2);
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	
	TIM_Cmd(TIM1, ENABLE);
  
  //TIM_OC1Init(TIM1, &);
}

/**************************************************************************************************
Definition: Interrupt from USART 
Arguments: No
Return:   No
Notes: Waiting for new messageand setting new speed if value in message = -2048..2047
**************************************************************************************************/
/**************************************************************************************************
Описание: Прерывание от USART
Аргументы: Нет
Возврат:   Нет
Замечания: Ожидание нового сообщения и установка скорости, если значение в сообщении = -2048..2047
**************************************************************************************************/
void USART1_IRQHandler(void)
{
  // Stops timer
  // Checks if 'b' received (need to remember it, maybe static variable)
  // Resumes timer if not
  
  // If previous message was 'b', then checks if new message = -2048..2047
  // If yes - new speed = message value
  // If no - speed doesn't change
  // Resumes timer
}

/**************************************************************************************************
Definition: Interrupt from TIM 
Arguments: No
Return:   No
Notes: Sends message by USART
**************************************************************************************************/
/**************************************************************************************************
Описание: Прерывание от TIM
Аргументы: Нет
Возврат:   Нет
Замечания: Посылает сообщение по USART
**************************************************************************************************/
void TIM1_IRQHandler(void)
{
  // Each second sends speed value
  // speed = motor_speed_getSpeed(); // Don't use this function
  // USART_SendData(USART1, speed);
}
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
