/*
 * Adc.h
 *
 *  Created on: Mar 11, 2021
 *      Author: cornelius
 *
 * Simple ADC abstraction.
 *
 * ADC is configured to sample all used channels.
 * Results are transferred using DMA to buffer.
 * When DMA finished, triggered data is copied to vars.
 * TODO: recalibration at regular intervals if temperature changes?
 * TODO: internal temperature reading see RM0440 pg 684
 *
 */

#ifndef SRC_EVDRIVERS_ADC_H_
#define SRC_EVDRIVERS_ADC_H_

#include "InterruptBase.h"

// REMINDER: struct holds actual binary values, getters convert to SI

typedef struct {
    static constexpr uint8_t AVG = 50;

//#pragma pack(4) //needed? review memory layout after build
    uint16_t gADCSamples[4];
    uint16_t evseCPLo[AVG];
    uint16_t evseCPHi[AVG];
    volatile uint8_t evseCPLoIdx, evseCPHiIdx;
    volatile uint16_t evsePP, evseDP1, pluckLockFB;
    volatile uint8_t evseCPSampleTarget;
} ADC;

//What were these for?
//float getCarCPHi(ADC* adc);
//float getCarCPLo(ADC* adc);

void getEvseCPLo(ADC* adc, float *out);
void getEvseCPHi(ADC* adc, float *out);

float getEvsePP(ADC* adc);
float getEvseDP1(ADC* adc);
float getPluckLockFB(ADC* adc);

//Allows control pilot to signal whether it is sending high or low current
void setTriggerEvseCPLo(ADC* adc);
void setTriggerEvseCPHi(ADC* adc);

void ADC_ISR(ADC* adc);

    



class Adc : private InterruptBase {
public:
    Adc(ADC_HandleTypeDef *_adc);
    virtual ~Adc();

    float getCarCPHi();
    float getCarCPLo();

    void getEvseCPHi(float *out);
    void getEvseCPLo(float *out);
    float getEvsePP();

    float getSupply12V();
    float getSupplyN12V();
    float getTemperature();

    // Call trigger from ISR only
    // calling ISR prio must be <= ADC DMA IRQ prio
    /*  void triggerCarCPHiISR();
      void triggerCarCPLoISR();
      */
    void triggerEvseCPHiISR();
    void triggerEvseCPLoISR();

    virtual void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

    static constexpr uint8_t AVG = 50;

private:
    ADC_HandleTypeDef *adc;
    // DMA buffer for results
#pragma pack(4)
    uint16_t adc_result_dma[7];
    // buffer for triggered values
    // uint16_t carCPHi, carCPLo;
    uint16_t evseCPHi[AVG], evseCPLo[AVG];
    volatile uint8_t evseCPHiIdx, evseCPLoIdx; 
    volatile uint16_t supply12V, supplyN12V, temperature, evsePP, vrefint;
    volatile uint8_t evseCPSampleTarget;
    volatile uint16_t evseCPuntriggered;
};

#endif // SRC_EVDRIVERS_ADC_H_
