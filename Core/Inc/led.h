//
// Created by lanzer on 24-12-2.
//

#ifndef PROJ2_LED_H
#define PROJ2_LED_H

#include "sys.h"

typedef struct {
    GPIO_TypeDef* gpio;
    uint16_t pin;
}LED_TypeDef;

extern LED_TypeDef led_green[];

void LED_Off(LED_TypeDef* led);
void LED_On(LED_TypeDef* LED);

#endif //PROJ2_LED_H
