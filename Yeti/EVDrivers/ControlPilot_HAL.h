/*
 * ControlPilot_HAL.h
 *
 *  Created on: 25.10.2021
 *  Author: cornelius
 *
 * IEC 61851-1 compliant Control Pilot state machine
 *
 * This class provides HAL abstraction for CP and PP signals:
 * 1) timer interrupt triggered ADC readings of PWM signal on CP
 * 2) PWM output on CP
 * 3) CP enable/disable (high impedance)
 * 4) PP reading (not implemented yet)
 * 5) Lock motor control
 * 6) supply voltage reading (uses the same ADC as CP signal sampling)
 *
 * TODO: Lock motor GPIO is currently hard coded
 */

#ifndef SRC_EVDRIVERS_CONTROLPILOT_HAL_H_
#define SRC_EVDRIVERS_CONTROLPILOT_HAL_H_

#include "cmsis_os.h"
#include "main.h"

#include "Adc.h"
#include "Gpio.h"
#include "InterruptBase.h"

typedef struct {
    GPTIMER_Regs* _pwmTimer;
    ADC* _adc;

    float _cpLo, _cpHi;
    GPIO* _cpEnable;
} ControlPilot_HAL;

bool ControlPilot_HAL_readCPSignal(ControlPilot_HAL *cp);
float ControlPilot_HAL_getCPHi(ControlPilot_HAL *cp);
float ControlPilot_HAL_getCPLo(ControlPilot_HAL *cp);
//float ControlPilot_HAL_getSupply12V(ControlPilot_HAL *cp);
//float ControlPilot_HAL_getSupplyN12V(ControlPilot_HAL *cp);

void ControlPilot_HAL_lockMotorLock(ControlPilot_HAL *cp);
void ControlPilot_HAL_lockMotorUnlock(ControlPilot_HAL *cp);
void ControlPilot_HAL_lockMotorOff(ControlPilot_HAL *cp);

void ControlPilot_HAL_setPWM(ControlPilot_HAL *cp, float dc);
void ControlPilot_HAL_enableCP(ControlPilot_HAL *cp);
void ControlPilot_HAL_disableCP(ControlPilot_HAL *cp);





class ControlPilot_HAL : private InterruptBase {
public:
    ControlPilot_HAL(TIM_HandleTypeDef *_pwmTimer, Adc &_adc, Gpio *_cpEnable);
    //ControlPilot_HAL(TIM_HandleTypeDef* _pwmTimer, ADC &_adc, GPIO* _cpEnable);
    virtual ~ControlPilot_HAL();

    bool readCPSignal();
    float getCPHi();
    float getCPLo();
    float getSupply12V();
    float getSupplyN12V();

    void lockMotorLock();
    void lockMotorUnlock();
    void lockMotorOff();

    void setPWM(float dc);
    void disableCP();
    void enableCP();

    // Interrupt callback
    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

private:
    // References to external objects we use here
    TIM_HandleTypeDef *pwmTimer;
    Adc &adc;

    float cpLo, cpHi;
    //IQ Math convert for MSPM0?

    Gpio *cpEnable;
};

#endif // SRC_EVDRIVERS_CONTROLPILOT_HAL_H_
