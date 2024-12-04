//
// Created by lanzer on 24-12-2.
//

#include "led.h"

LED_TypeDef led_green[8] = {
        {.gpio = GPIOG, .pin = GPIO_PIN_1},
        {.gpio = GPIOG, .pin = GPIO_PIN_2},
        {.gpio = GPIOG, .pin = GPIO_PIN_3},
        {.gpio = GPIOG, .pin = GPIO_PIN_4},
        {.gpio = GPIOG, .pin = GPIO_PIN_5},
        {.gpio = GPIOG, .pin = GPIO_PIN_6},
        {.gpio = GPIOG, .pin = GPIO_PIN_7},
        {.gpio = GPIOG, .pin = GPIO_PIN_8},
};

void LED_Off(LED_TypeDef* led)
{
    HAL_GPIO_WritePin(led->gpio, led->pin, GPIO_PIN_SET);
}

void LED_On(LED_TypeDef* led)
{
    HAL_GPIO_WritePin(led->gpio, led->pin, GPIO_PIN_RESET);
}