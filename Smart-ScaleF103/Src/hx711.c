#include "hx711.h"

void HX711_Init(HX711 data)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = data.pinSck;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(data.gpioSck, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = data.pinData;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(data.gpioData, &GPIO_InitStruct);

	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);

}

long HX711_Average_Value(HX711 data, uint8_t times)
{
    long sum = 0;
    for (int i = 0; i < times; i++)
    {
        sum += HX711_Value(data);
    }

    return sum / times;
}

long HX711_Value(HX711 data)
{
    uint32_t buffer = 0;
    uint32_t filler = 0;
    unsigned long value = 0;

    while (HAL_GPIO_ReadPin(data.gpioData, data.pinData)==1)
    ;

    for (uint8_t i = 0; i < 24; i++)
    {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    	for(int a = 0; a < 1000; a++);
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    	for(int a = 0; a < 1000; a++);

        buffer = buffer << 1 ;

        if (HAL_GPIO_ReadPin(data.gpioData, data.pinData) == GPIO_PIN_SET)
        {
            buffer++;
        }
    }

    for (int i = 0; i < data.gain; i++)
    {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    	for(int a = 0; a < 1000; a++);
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    	for(int a = 0; a < 1000; a++);
    }

    if ( buffer & 0x800000 ) {
           filler = 0xFF000000;
       } else {
           filler = 0x00000000;
       }

    value = buffer | filler;
    value++;
    // ... and add 1

    return (long)value;
}

HX711 HX711_Tare(HX711 data, uint8_t times)
{
    int sum = HX711_Average_Value(data, times);
    data.offset = sum;
    return data;
}

