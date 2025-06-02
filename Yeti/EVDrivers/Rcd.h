/*
 * Rcd.h
 *
 *  Created on: 01.03.2021
 *      Author: cornelius
 *
 * RCD driver
 *
 * If RCD triggers due to too high residual current isrRcdFired is set to true
 * and Relais are switched off directly in interrupt handler.
 *
 * isrRcdEnabled is a flag that can be modified also by high level to enable or
 * disable RCD usage (config option).s
 *
 * isrRcdActive is an yeti-internal flag used to (de-)activate the RCD in sync
 * with relais switching to prevent erroneous firing directly during switch
 * on/off.
 *
 * Requires one timer with input capture pin and 2 CC units for PWM decoding.
 * CC CH1 must trigger on rising edge (direct capture of input pin)
 * and CC CH2 on falling edge (indirect capture of input pin)
 * The timer must be configured before running this driver (incl. enable of
 * interrupt)
 *
 * executeSelfTest() runs an actual HW self test with induced residual current.
 * While running the test emergency switch off is disabled. This test should be
 * run before every charging cycle to ensure the RCD is still functional. It
 * could also be run during charging at regular intervals, but be aware that RCD
 * is inactive for a maximum of 10 seconds.
 *
 * TODO:
 *  - r1: support seperate AC_ERROR input as well. Currently only DC_error
 * (includes AC as well) is supported as in r0
 */

#ifndef SRC_EVDRIVERS_RCD_H_
#define SRC_EVDRIVERS_RCD_H_

#include "cmsis_os.h"

#include "Gpio.h"
#include "InterruptBase.h"
#include "PowerSwitch.h"
#include "EVConfig.h"

// Global variables for error GPIOs (as they are needed in interrupt handler!)s
extern uint16_t rcd_dc_errorin_gpio_pin;
extern GPIO_Regs* rcd_dc_errorin_gpio_port;
extern uint16_t rcd_ac_errorin_gpio_pin;
extern GPIO_Regs* rcd_ac_errorin_gpio_port;


//TODO: ensure corresponding GPIOs and TIM have IRQ enabled and are configured to handle RCD
//Also, determine I2C packet format from TIDA-010237 to determine how to integrate
typedef struct {
    float _residualCurrentPercentPerMa;
    uint16_t _expectedPwmPeriod; //do we need this if we have I2C?

    GPIO* _testout;
    GPIO* _errorin;
    GPIO* _pwmin;
    GPTIMER_Regs* _pwmTimer; //do we need this if we have I2C?
    PowerSwitch* _powerSwitch;

    uint16_t _isrLastRising, _isrRising, _isrFalling = 0;
    uint16_t _isrPwmPeriod;
    float _isrPwmDutyCyclePercent;
    bool _isrPwmUpdated;
    uint16_t _isrPwmUpdateTimeout;
    bool _isrRcdTestModeError;
    bool _isrRcdEnabled;
    bool _isrRcdActive;
    bool _isrRcdFired;
    // bool _isrRcdUnused;
} RCD;

void RCD_executeSelfTest(RCD *rcd);
void RCD_getResidualCurrent(RCD *rcd);
void RCD_enable(RCD *rcd);
void RCD_disable(RCD *rcd);
void RCD_activate(RCD *rcd);
void RCD_deactivate(RCD *rcd);
bool RCD_getEnabled(RCD *rcd);
bool RCD_getRcdFired(RCD *rcd);
void RCD_fire(RCD *rcd);
void RCD_reset(RCD *rcd);




class Rcd : private InterruptBase {
public:
    Rcd(Gpio &_testout, Gpio &_errorin, Gpio &_pwmin,
        TIM_HandleTypeDef *_pwmTimer, PowerSwitch *_powerSwitch);
    ~Rcd();
    bool executeSelfTest();

    float getResidualCurrent();
    void enable();
    void disable();
    void activate();
    void deactivate();
    bool getEnabled();
    // void setUnused(bool u);
    bool getRcdFired();
    void fire();
    void reset();

    virtual void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
    virtual void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
    virtual void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

private:
    float residualCurrentPercentPerMa;
    uint16_t expectedPwmPeriod;

    Gpio &testout;
    Gpio &errorin;
    Gpio &pwmin;
    TIM_HandleTypeDef *pwmTimer;
    PowerSwitch *powerSwitch;

    // Interrupt access only variables. Only access protected by
    // taskENTER_CRITICAL etc.
    uint16_t isrLastRising, isrRising, isrFalling = 0;
    uint16_t isrPwmPeriod;
    float isrPwmDutyCyclePercent;
    bool isrPwmUpdated;
    uint16_t isrPwmUpdateTimeout;
    bool isrRcdTestModeError;
    bool isrRcdEnabled;
    bool isrRcdActive;
    bool isrRcdFired;
    // bool isrRcdUnused;
};

#endif // SRC_EVDRIVERS_RCD_H_
