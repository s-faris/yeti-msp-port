/*
 * Gpio.cpp
 *
 *  Created on: 26.02.2021
 *      Author: cornelius
 */

#include "Gpio.h"
#include <stdio.h>

Gpio::Gpio(GPIO_TypeDef *_port, uint16_t _pin) {
    port = _port;
    pin = _pin;
}

Gpio::~Gpio() {}

void Gpio::set() { HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); } //DL_GPIO_writePins(this->port, pin)

void Gpio::reset() { HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); }

bool Gpio::read() { return (HAL_GPIO_ReadPin(port, pin) ? true : false); }

uint16_t Gpio::getPin() { return pin; }

GPIO_TypeDef *Gpio::getPort() { return port; }


//SAM:

GPIO::GPIO(GPIO_Regs* gpio, uint32_t pins) {
    this->gpio = gpio;
    this->pins = pins;
}

GPIO::~GPIO() {};

void GPIO::set() { DL_GPIO_setPins(gpioPort, gpioPins); }

void GPIO::reset() { DL_GPIO_clearPins(gpioPort, gpioPins); }

bool GPIO::read() { return (DL_GPIO_readPins(gpioPort, gpioPins) ? true : false); }

uint16_t GPIO::getPin() { return gpioPins; }

GPIO_Regs* GPIO::getPort() { return gpioPort; }
