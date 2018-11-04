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
Definition: Set new voltage on motor
Arguments: The new value of the voltage on the motor in the range from -1000 to 1000
Return:   No
Notes: 
**************************************************************************************************/
/**************************************************************************************************
Описание: Инициализация USART 
Аргументы: Новое значение напряжения на моторе в диапазоне от -1000 до 1000
Возврат:   Нет
Замечания: Измеряется в мВ ?
**************************************************************************************************/
void motor_voltage_setVoltage(int16_t voltage)
{
}
