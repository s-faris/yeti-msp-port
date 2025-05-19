/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gRelay_ENBackup;
DL_TimerG_backupConfig gPluckLockBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations */
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_CCS_CP_init();
    SYSCFG_DL_Relay_EN_init();
    SYSCFG_DL_PluckLock_init();
    SYSCFG_DL_RCD_init();
    SYSCFG_DL_AM62L_init();
    SYSCFG_DL_Debug_init();
    SYSCFG_DL_ADC0_init();
    SYSCFG_DL_ADC1_Temp_init();
    SYSCFG_DL_VREF_init();
    SYSCFG_DL_DMA_init();
    SYSCFG_DL_SYSTICK_init();
    SYSCFG_DL_SYSCTL_CLK_init();
    /* Ensure backup structures have no valid state */
	gRelay_ENBackup.backupRdy 	= false;
	gPluckLockBackup.backupRdy 	= false;


}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(Relay_EN_INST, &gRelay_ENBackup);
	retStatus &= DL_TimerG_saveConfiguration(PluckLock_INST, &gPluckLockBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(Relay_EN_INST, &gRelay_ENBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(PluckLock_INST, &gPluckLockBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(CCS_CP_INST);
    DL_TimerA_reset(Relay_EN_INST);
    DL_TimerG_reset(PluckLock_INST);
    DL_I2C_reset(RCD_INST);
    DL_UART_Main_reset(AM62L_INST);
    DL_UART_Main_reset(Debug_INST);
    DL_ADC12_reset(ADC0_INST);
    DL_ADC12_reset(ADC1_Temp_INST);
    DL_VREF_reset(VREF);



    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(CCS_CP_INST);
    DL_TimerA_enablePower(Relay_EN_INST);
    DL_TimerG_enablePower(PluckLock_INST);
    DL_I2C_enablePower(RCD_INST);
    DL_UART_Main_enablePower(AM62L_INST);
    DL_UART_Main_enablePower(Debug_INST);
    DL_ADC12_enablePower(ADC0_INST);
    DL_ADC12_enablePower(ADC1_Temp_INST);
    DL_VREF_enablePower(VREF);


    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralAnalogFunction(GPIO_ROSC_IOMUX);

    DL_GPIO_initPeripheralOutputFunction(GPIO_CCS_CP_C0_IOMUX,GPIO_CCS_CP_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_CCS_CP_C0_PORT, GPIO_CCS_CP_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Relay_EN_C0_IOMUX,GPIO_Relay_EN_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Relay_EN_C0_PORT, GPIO_Relay_EN_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PluckLock_C0_IOMUX,GPIO_PluckLock_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PluckLock_C0_PORT, GPIO_PluckLock_C0_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_RCD_IOMUX_SDA,
        GPIO_RCD_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_RCD_IOMUX_SCL,
        GPIO_RCD_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_RCD_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_RCD_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_AM62L_IOMUX_TX, GPIO_AM62L_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_AM62L_IOMUX_RX, GPIO_AM62L_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_Debug_IOMUX_TX, GPIO_Debug_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_Debug_IOMUX_RX, GPIO_Debug_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(BSL_PIN_0_IOMUX);

    DL_GPIO_initDigitalOutput(LED_PIN_1_IOMUX);

    DL_GPIO_initDigitalOutput(EN_Charging_PIN_2_IOMUX);

    DL_GPIO_initDigitalOutput(REQ_Charge_PIN_3_IOMUX);

    DL_GPIO_initDigitalOutput(IO_Out_2_PIN_4_IOMUX);

    DL_GPIO_initDigitalInputFeatures(IO_IN_1_PIN_5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(IO_Out_1_PIN_6_IOMUX);

    DL_GPIO_initDigitalInputFeatures(IO_IN_2_PIN_7_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(CdM_j_PIN_9_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(CdM_PD_PIN_10_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(CdM_d2_PIN_11_IOMUX);

    DL_GPIO_initDigitalOutput(CdM_d1_PIN_12_IOMUX);

    DL_GPIO_initDigitalOutput(GBT_EV_S2_PIN_13_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Pluck_Lock_CdM_FB_PIN_14_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GBT_EV_S2apo_PIN_15_IOMUX);

    DL_GPIO_initDigitalOutput(GBT_S1_PIN_16_IOMUX);

    DL_GPIO_initDigitalOutput(GBT_S0_PIN_17_IOMUX);

    DL_GPIO_initDigitalOutput(Pluck_Lock_nSleep_PIN_18_IOMUX);

    DL_GPIO_initDigitalOutput(Pluck_Lock_Direction_PIN_19_IOMUX);

    DL_GPIO_initDigitalInputFeatures(R_Mirror_Out_PIN_20_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Relay_Check_PIN_21_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(RCD_Reset_PIN_24_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Super_Cap_Char_PIN_25_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(IO_Out_3_PIN_26_IOMUX);

    DL_GPIO_initDigitalInputFeatures(RCD_nFault_PIN_27_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, BSL_PIN_0_PIN |
		LED_PIN_1_PIN |
		IO_Out_1_PIN_6_PIN |
		CdM_d2_PIN_11_PIN);
    DL_GPIO_enableOutput(GPIOA, BSL_PIN_0_PIN |
		LED_PIN_1_PIN |
		IO_Out_1_PIN_6_PIN |
		CdM_d2_PIN_11_PIN);
    DL_GPIO_clearPins(GPIOB, EN_Charging_PIN_2_PIN |
		REQ_Charge_PIN_3_PIN |
		IO_Out_2_PIN_4_PIN |
		CdM_d1_PIN_12_PIN |
		GBT_EV_S2_PIN_13_PIN |
		GBT_EV_S2apo_PIN_15_PIN |
		GBT_S1_PIN_16_PIN |
		GBT_S0_PIN_17_PIN |
		Pluck_Lock_nSleep_PIN_18_PIN |
		Pluck_Lock_Direction_PIN_19_PIN |
		RCD_Reset_PIN_24_PIN |
		IO_Out_3_PIN_26_PIN);
    DL_GPIO_enableOutput(GPIOB, EN_Charging_PIN_2_PIN |
		REQ_Charge_PIN_3_PIN |
		IO_Out_2_PIN_4_PIN |
		CdM_d1_PIN_12_PIN |
		GBT_EV_S2_PIN_13_PIN |
		GBT_EV_S2apo_PIN_15_PIN |
		GBT_S1_PIN_16_PIN |
		GBT_S0_PIN_17_PIN |
		Pluck_Lock_nSleep_PIN_18_PIN |
		Pluck_Lock_Direction_PIN_19_PIN |
		RCD_Reset_PIN_24_PIN |
		IO_Out_3_PIN_26_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
	.rDivClk2x              = 0,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_ENABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK0,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_SYSOSC,
	.qDiv                   = 9,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_2,
	
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be STOP2
    DL_SYSCTL_setPowerPolicySTOP2();
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    /* Enable SYSOSC FCL in External Resistor Mode */
    DL_SYSCTL_enableSYSOSCFCLExternalResistor();
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
	
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

}
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_CLK_init(void) {
    while ((DL_SYSCTL_getClockStatus() & (DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD
		 | DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
	       != (DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD
		 | DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
	{
		/* Ensure that clocks are in default POR configuration before initialization.
		* Additionally once LFXT is enabled, the internal LFOSC is disabled, and cannot
		* be re-enabled other than by executing a BOOTRST. */
		;
	}
}



/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   125000 Hz = 32000000 Hz / (1 * (255 + 1))
 */
static const DL_TimerG_ClockConfig gCCS_CPClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 255U
};

static const DL_TimerG_PWMConfig gCCS_CPConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_CENTER_ALIGN,
    .period = 100,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_CCS_CP_init(void) {

    DL_TimerG_setClockConfig(
        CCS_CP_INST, (DL_TimerG_ClockConfig *) &gCCS_CPClockConfig);

    DL_TimerG_initPWMMode(
        CCS_CP_INST, (DL_TimerG_PWMConfig *) &gCCS_CPConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(CCS_CP_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(CCS_CP_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(CCS_CP_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(CCS_CP_INST, 25, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(CCS_CP_INST);


    
    DL_TimerG_setCCPDirection(CCS_CP_INST , DL_TIMER_CC0_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   125000 Hz = 32000000 Hz / (1 * (255 + 1))
 */
static const DL_TimerA_ClockConfig gRelay_ENClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 255U
};

static const DL_TimerA_PWMConfig gRelay_ENConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_CENTER_ALIGN,
    .period = 100,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_Relay_EN_init(void) {

    DL_TimerA_setClockConfig(
        Relay_EN_INST, (DL_TimerA_ClockConfig *) &gRelay_ENClockConfig);

    DL_TimerA_initPWMMode(
        Relay_EN_INST, (DL_TimerA_PWMConfig *) &gRelay_ENConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(Relay_EN_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(Relay_EN_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Relay_EN_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(Relay_EN_INST, 25, DL_TIMER_CC_0_INDEX);

    DL_TimerA_enableClock(Relay_EN_INST);


    
    DL_TimerA_setCCPDirection(Relay_EN_INST , DL_TIMER_CC0_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   125000 Hz = 32000000 Hz / (1 * (255 + 1))
 */
static const DL_TimerG_ClockConfig gPluckLockClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 255U
};

static const DL_TimerG_PWMConfig gPluckLockConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_CENTER_ALIGN,
    .period = 100,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PluckLock_init(void) {

    DL_TimerG_setClockConfig(
        PluckLock_INST, (DL_TimerG_ClockConfig *) &gPluckLockClockConfig);

    DL_TimerG_initPWMMode(
        PluckLock_INST, (DL_TimerG_PWMConfig *) &gPluckLockConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PluckLock_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PluckLock_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PluckLock_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PluckLock_INST, 25, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(PluckLock_INST);


    
    DL_TimerG_setCCPDirection(PluckLock_INST , DL_TIMER_CC0_OUTPUT );


}


static const DL_I2C_ClockConfig gRCDClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_RCD_init(void) {

    DL_I2C_setClockConfig(RCD_INST,
        (DL_I2C_ClockConfig *) &gRCDClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(RCD_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(RCD_INST);
    DL_I2C_setDigitalGlitchFilterPulseWidth(RCD_INST,
        DL_I2C_DIGITAL_GLITCH_FILTER_WIDTH_CLOCKS_1);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(RCD_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(RCD_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(RCD_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_7);
    DL_I2C_setControllerRXFIFOThreshold(RCD_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_8);
    DL_I2C_enableControllerClockStretching(RCD_INST);


    /* Enable module */
    DL_I2C_enableController(RCD_INST);


}


static const DL_UART_Main_ClockConfig gAM62LClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gAM62LConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_AM62L_init(void)
{
    DL_UART_Main_setClockConfig(AM62L_INST, (DL_UART_Main_ClockConfig *) &gAM62LClockConfig);

    DL_UART_Main_init(AM62L_INST, (DL_UART_Main_Config *) &gAM62LConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9600.24
     */
    DL_UART_Main_setOversampling(AM62L_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(AM62L_INST, AM62L_IBRD_32_MHZ_9600_BAUD, AM62L_FBRD_32_MHZ_9600_BAUD);



    DL_UART_Main_enable(AM62L_INST);
}

static const DL_UART_Main_ClockConfig gDebugClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gDebugConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_Debug_init(void)
{
    DL_UART_Main_setClockConfig(Debug_INST, (DL_UART_Main_ClockConfig *) &gDebugClockConfig);

    DL_UART_Main_init(Debug_INST, (DL_UART_Main_Config *) &gDebugConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9600.24
     */
    DL_UART_Main_setOversampling(Debug_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(Debug_INST, Debug_IBRD_32_MHZ_9600_BAUD, Debug_FBRD_32_MHZ_9600_BAUD);



    DL_UART_Main_enable(Debug_INST);
}

/* ADC0 Initialization */
static const DL_ADC12_ClockConfig gADC0ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_2,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC0_init(void)
{
    DL_ADC12_setClockConfig(ADC0_INST, (DL_ADC12_ClockConfig *) &gADC0ClockConfig);

    DL_ADC12_initSeqSample(ADC0_INST,
        DL_ADC12_REPEAT_MODE_DISABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_03, DL_ADC12_SAMP_CONV_RES_8_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC0_INST, ADC0_ADCMEM_CCS_CP_FB,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC0_INST, ADC0_ADCMEM_CCS_PP,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC0_INST, ADC0_ADCMEM_GBT_DP1,
        DL_ADC12_INPUT_CHAN_1, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC0_INST, ADC0_ADCMEM_PL_FB,
        DL_ADC12_INPUT_CHAN_3, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(ADC0_INST,14);
    DL_ADC12_enableDMA(ADC0_INST);
    DL_ADC12_setDMASamplesCnt(ADC0_INST,6);
    DL_ADC12_enableDMATrigger(ADC0_INST,(DL_ADC12_DMA_MEM5_RESULT_LOADED));
    DL_ADC12_enableConversions(ADC0_INST);
}
/* ADC1_Temp Initialization */
static const DL_ADC12_ClockConfig gADC1_TempClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC1_Temp_init(void)
{
    DL_ADC12_setClockConfig(ADC1_Temp_INST, (DL_ADC12_ClockConfig *) &gADC1_TempClockConfig);

    DL_ADC12_initSeqSample(ADC1_Temp_INST,
        DL_ADC12_REPEAT_MODE_DISABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_01, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC1_Temp_INST, ADC1_Temp_ADCMEM_Temp1,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC1_Temp_INST, ADC1_Temp_ADCMEM_Temp2,
        DL_ADC12_INPUT_CHAN_1, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_enableDMA(ADC1_Temp_INST);
    DL_ADC12_setDMASamplesCnt(ADC1_Temp_INST,2);
    DL_ADC12_enableDMATrigger(ADC1_Temp_INST,(DL_ADC12_DMA_MEM1_RESULT_LOADED));
    DL_ADC12_enableConversions(ADC1_Temp_INST);
}


static const DL_VREF_Config gVREFConfig = {
    .vrefEnable     = DL_VREF_ENABLE_DISABLE,
    .bufConfig      = DL_VREF_BUFCONFIG_OUTPUT_2_5V,
    .shModeEnable   = DL_VREF_SHMODE_DISABLE,
    .holdCycleCount = DL_VREF_HOLD_MIN,
    .shCycleCount   = DL_VREF_SH_MIN,
};

SYSCONFIG_WEAK void SYSCFG_DL_VREF_init(void) {
    DL_VREF_configReference(VREF,
        (DL_VREF_Config *) &gVREFConfig);
}


static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_INCREMENT,
    .destWidth      = DL_DMA_WIDTH_WORD,
    .srcWidth       = DL_DMA_WIDTH_WORD,
    .trigger        = ADC0_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH0_init(void)
{
    DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID , (DL_DMA_Config *) &gDMA_CH0Config);
}
static const DL_DMA_Config gDMA_CH1Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_WORD,
    .srcWidth       = DL_DMA_WIDTH_WORD,
    .trigger        = ADC1_Temp_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH1_init(void)
{
    DL_DMA_initChannel(DMA, DMA_CH1_CHAN_ID , (DL_DMA_Config *) &gDMA_CH1Config);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void){
    SYSCFG_DL_DMA_CH0_init();
    SYSCFG_DL_DMA_CH1_init();
}


SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
}

