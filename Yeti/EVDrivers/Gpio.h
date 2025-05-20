/*
 * Gpio.h
 *
 *  Created on: 26.02.2021
 *      Author: cornelius
 *
 * Basic C++ abstraction for GPIOs based on HAL driver.
 * Does not cover init yet, this is done in main.c via auto generated code
 * from .ioc file.
 */

#ifndef SRC_EVDRIVERS_GPIO_H_
#define SRC_EVDRIVERS_GPIO_H_

#include "stm32g4xx_hal.h"
#include "ti_msp_dl_config.h"

class Gpio {
public:
    Gpio(GPIO_TypeDef *_port, uint16_t _pin);
    virtual ~Gpio();
    void set();
    void reset();
    bool read();

    uint16_t getPin();
    GPIO_TypeDef *getPort();

private:
    GPIO_TypeDef *port;
    uint16_t pin;
};

//SAM :

typedef struct {
    GPIO_Regs* port;
    uint32_t pins;
} GPIO;

void set(GPIO gpio);
void reset(GPIO gpio);
bool read(GPIO gpio);

uint16t getPin(GPIO gpio);
GPIO_Regs* getPort(GPIO gpio);

#endif // SRC_EVDRIVERS_GPIO_H_
