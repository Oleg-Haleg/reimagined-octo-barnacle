/*******************************************************************************************************
Definition: The main file for h/w on programming microcontrollers
Developer: Trishin Vadim
Notes: Peripherals used: USART1, ADC1, TIM1, GPIO
*******************************************************************************************************/
/*******************************************************************************************************
Описание: Основной файл для домашней работы по программированию микроконтроллеров
Разработчик: Тришин Вадим
Заметки: Используемая перефирия: USART1, ADC1, TIM1, GPIO
*******************************************************************************************************/

#include "mcu_support_package/inc/stm32f10x.h"
//#include <stm32f10x.h>
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_adc.h"
//#include <string.h>

#include "lab_functions/debug.h"
#include "lab_functions/motor_voltage.h"
#include "lab_functions/motor_speed.h"
#include "lab_functions/control.h"

void init_GPIO(void);
void init_ADC1(void);
/***************************************************************************************************
Global functions
***************************************************************************************************/
/***************************************************************************************************
Глобальные функции
***************************************************************************************************/

int main()
{  
  __disable_irq();
  int16_t speed;
  __enable_irq();
  while(1)
  {
    speed = mem_speed(0, false);
    control_run(speed);
  }
  return 0;
}

//
void init_GPIO(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
  
  GPIO_InitTypeDef structGPIO;
  //GPIO_StructInit(&structGPIO);
	
	// Initialize speed once for all pins
	structGPIO.GPIO_Speed = GPIO_Speed_2MHz;
	
	// Add GPIOA 8 and comment it
  // USART1_TX, TIM1_CH2
//  structGPIO.GPIO_Pin  = GPIO_Pin_8;
//  structGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOA, &structGPIO);
	
  // USART1_CTS, TIM1_CH4
  structGPIO.GPIO_Pin  = GPIO_Pin_11; 
  structGPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &structGPIO);	
}

//
void init_ADC1(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
  
  GPIO_InitTypeDef structGPIO;
  //GPIO_StructInit(&structGPIO);
	
	// Initialize speed once for all pins
	structGPIO.GPIO_Speed = GPIO_Speed_2MHz;
	
  // ADC1_IN7, remap(TIM1_CH1N)
  structGPIO.GPIO_Pin  = GPIO_Pin_7;
  structGPIO.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &structGPIO);
	// ADC1_IN8, remap(TIM1_CH2N)
  structGPIO.GPIO_Pin  = GPIO_Pin_0; 
  structGPIO.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &structGPIO);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	//инициализация модуля АЦП
	ADC_InitTypeDef structADC;
	
	structADC.ADC_ContinuousConvMode = ENABLE;
	structADC.ADC_DataAlign = ADC_DataAlign_Right; // Left if used for voltage
	structADC.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	structADC.ADC_Mode = ADC_Mode_Independent;
	structADC.ADC_NbrOfChannel = 1; // one for speed indicator
	structADC.ADC_ScanConvMode = ENABLE;
	
	ADC_Init(ADC1, &structADC);
	
	//настройка канала преобразования АЦП
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
	
	//настройка прерывания от АЦП
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	
	//__disable_irq();
	
	//приоритет прерывания АЦП 
	//NVIC_SetPriority(ADC1_2_IRQn, 2);
	NVIC_EnableIRQ(ADC1_2_IRQn);
	
	//запуск модуля АЦП
	ADC_Cmd(ADC1, ENABLE);

	//__enable_irq();
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
