#include "stm32f1xx_hal.h"
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DHT11_CheckRespone(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DHT11_ReadData(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) ;