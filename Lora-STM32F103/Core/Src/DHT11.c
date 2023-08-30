#include "DHT11.h"
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void DHT11_Init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	Set_Pin_Output(GPIOA, GPIO_PIN_1); //set PA1 ouput to pull down in 18ms
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0); //pull down in 18ms
	delay_us(18000); //18ms
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	delay_us(20); //pull wait
	Set_Pin_Input(GPIOx, GPIO_Pin);
}
uint8_t DHT11_CheckRespone(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	uint8_t Respone = 0;
	delay_us(40); //point middle low
	if (!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
		delay_us(80); //point middle high
		if (!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
			Respone = 1;
		else
			Respone = -1;
	}
		uint32_t timeout = 0;
		while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
			if (timeout++ > 1160 * 4 * 2)
				break; //wait to go low

	return Respone;
}
uint8_t DHT11_ReadData(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {

	uint8_t i, j;
	for (j = 0; j < 8; j++) {
		uint32_t timeout = 0;
		while (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)))
			if (timeout++ > 1160 * 4 * 2)
				break; // wait for the pin to go high
		delay_us(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) // if the pin is low
		{
			i &= ~(1 << (7 - j));   // write 0
		} else
			i |= (1 << (7 - j));  // if the pin is high, write 1
		timeout = 0;
		while ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)))
				if (timeout++ > 1160 * 4 * 2)
				break;  // wait for the pin to go low
	}

	return i;
}