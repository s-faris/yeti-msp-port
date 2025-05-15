/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_ROSC_PORT                                                     GPIOA
#define GPIO_ROSC_PIN                                              DL_GPIO_PIN_2
#define GPIO_ROSC_IOMUX                                           (IOMUX_PINCM7)
#define CPUCLK_FREQ                                                     32000000



/* Defines for CCS_CP */
#define CCS_CP_INST                                                        TIMG0
#define CCS_CP_INST_IRQHandler                                  TIMG0_IRQHandler
#define CCS_CP_INST_INT_IRQN                                    (TIMG0_INT_IRQn)
#define CCS_CP_INST_CLK_FREQ                                              125000
/* GPIO defines for channel 0 */
#define GPIO_CCS_CP_C0_PORT                                                GPIOB
#define GPIO_CCS_CP_C0_PIN                                        DL_GPIO_PIN_10
#define GPIO_CCS_CP_C0_IOMUX                                     (IOMUX_PINCM27)
#define GPIO_CCS_CP_C0_IOMUX_FUNC                    IOMUX_PINCM27_PF_TIMG0_CCP0
#define GPIO_CCS_CP_C0_IDX                                   DL_TIMER_CC_0_INDEX

/* Defines for Relay_EN */
#define Relay_EN_INST                                                      TIMA1
#define Relay_EN_INST_IRQHandler                                TIMA1_IRQHandler
#define Relay_EN_INST_INT_IRQN                                  (TIMA1_INT_IRQn)
#define Relay_EN_INST_CLK_FREQ                                            125000
/* GPIO defines for channel 0 */
#define GPIO_Relay_EN_C0_PORT                                              GPIOB
#define GPIO_Relay_EN_C0_PIN                                       DL_GPIO_PIN_0
#define GPIO_Relay_EN_C0_IOMUX                                   (IOMUX_PINCM12)
#define GPIO_Relay_EN_C0_IOMUX_FUNC                  IOMUX_PINCM12_PF_TIMA1_CCP0
#define GPIO_Relay_EN_C0_IDX                                 DL_TIMER_CC_0_INDEX

/* Defines for PluckLock */
#define PluckLock_INST                                                     TIMG7
#define PluckLock_INST_IRQHandler                               TIMG7_IRQHandler
#define PluckLock_INST_INT_IRQN                                 (TIMG7_INT_IRQn)
#define PluckLock_INST_CLK_FREQ                                           125000
/* GPIO defines for channel 0 */
#define GPIO_PluckLock_C0_PORT                                             GPIOA
#define GPIO_PluckLock_C0_PIN                                     DL_GPIO_PIN_28
#define GPIO_PluckLock_C0_IOMUX                                   (IOMUX_PINCM3)
#define GPIO_PluckLock_C0_IOMUX_FUNC                  IOMUX_PINCM3_PF_TIMG7_CCP0
#define GPIO_PluckLock_C0_IDX                                DL_TIMER_CC_0_INDEX




/* Defines for RCD */
#define RCD_INST                                                            I2C0
#define RCD_INST_IRQHandler                                      I2C0_IRQHandler
#define RCD_INST_INT_IRQN                                          I2C0_INT_IRQn
#define RCD_BUS_SPEED_HZ                                                  100000
#define GPIO_RCD_SDA_PORT                                                  GPIOA
#define GPIO_RCD_SDA_PIN                                           DL_GPIO_PIN_0
#define GPIO_RCD_IOMUX_SDA                                        (IOMUX_PINCM1)
#define GPIO_RCD_IOMUX_SDA_FUNC                         IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_RCD_SCL_PORT                                                  GPIOA
#define GPIO_RCD_SCL_PIN                                           DL_GPIO_PIN_1
#define GPIO_RCD_IOMUX_SCL                                        (IOMUX_PINCM2)
#define GPIO_RCD_IOMUX_SCL_FUNC                         IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for AM62L */
#define AM62L_INST                                                         UART0
#define AM62L_INST_FREQUENCY                                            32000000
#define AM62L_INST_IRQHandler                                   UART0_IRQHandler
#define AM62L_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_AM62L_RX_PORT                                                 GPIOA
#define GPIO_AM62L_TX_PORT                                                 GPIOA
#define GPIO_AM62L_RX_PIN                                         DL_GPIO_PIN_11
#define GPIO_AM62L_TX_PIN                                         DL_GPIO_PIN_10
#define GPIO_AM62L_IOMUX_RX                                      (IOMUX_PINCM22)
#define GPIO_AM62L_IOMUX_TX                                      (IOMUX_PINCM21)
#define GPIO_AM62L_IOMUX_RX_FUNC                       IOMUX_PINCM22_PF_UART0_RX
#define GPIO_AM62L_IOMUX_TX_FUNC                       IOMUX_PINCM21_PF_UART0_TX
#define AM62L_BAUD_RATE                                                   (9600)
#define AM62L_IBRD_32_MHZ_9600_BAUD                                        (208)
#define AM62L_FBRD_32_MHZ_9600_BAUD                                         (21)
/* Defines for Debug */
#define Debug_INST                                                         UART1
#define Debug_INST_FREQUENCY                                            32000000
#define Debug_INST_IRQHandler                                   UART1_IRQHandler
#define Debug_INST_INT_IRQN                                       UART1_INT_IRQn
#define GPIO_Debug_RX_PORT                                                 GPIOA
#define GPIO_Debug_TX_PORT                                                 GPIOA
#define GPIO_Debug_RX_PIN                                          DL_GPIO_PIN_9
#define GPIO_Debug_TX_PIN                                          DL_GPIO_PIN_8
#define GPIO_Debug_IOMUX_RX                                      (IOMUX_PINCM20)
#define GPIO_Debug_IOMUX_TX                                      (IOMUX_PINCM19)
#define GPIO_Debug_IOMUX_RX_FUNC                       IOMUX_PINCM20_PF_UART1_RX
#define GPIO_Debug_IOMUX_TX_FUNC                       IOMUX_PINCM19_PF_UART1_TX
#define Debug_BAUD_RATE                                                   (9600)
#define Debug_IBRD_32_MHZ_9600_BAUD                                        (208)
#define Debug_FBRD_32_MHZ_9600_BAUD                                         (21)





/* Defines for ADC0 */
#define ADC0_INST                                                           ADC0
#define ADC0_INST_IRQHandler                                     ADC0_IRQHandler
#define ADC0_INST_INT_IRQN                                       (ADC0_INT_IRQn)
#define ADC0_ADCMEM_CCS_CP_FB                                 DL_ADC12_MEM_IDX_0
#define ADC0_ADCMEM_CCS_CP_FB_REF                DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC0_ADCMEM_CCS_CP_FB_REF_VOLTAGE_V                                     3.3
#define ADC0_ADCMEM_CCS_PP                                    DL_ADC12_MEM_IDX_1
#define ADC0_ADCMEM_CCS_PP_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC0_ADCMEM_CCS_PP_REF_VOLTAGE_V                                     3.3
#define ADC0_ADCMEM_GBT_DP1                                   DL_ADC12_MEM_IDX_2
#define ADC0_ADCMEM_GBT_DP1_REF                  DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC0_ADCMEM_GBT_DP1_REF_VOLTAGE_V                                     3.3
#define ADC0_ADCMEM_PL_FB                                     DL_ADC12_MEM_IDX_3
#define ADC0_ADCMEM_PL_FB_REF                    DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC0_ADCMEM_PL_FB_REF_VOLTAGE_V                                      3.3
#define GPIO_ADC0_C0_PORT                                                  GPIOA
#define GPIO_ADC0_C0_PIN                                          DL_GPIO_PIN_27
#define GPIO_ADC0_C2_PORT                                                  GPIOA
#define GPIO_ADC0_C2_PIN                                          DL_GPIO_PIN_25
#define GPIO_ADC0_C1_PORT                                                  GPIOA
#define GPIO_ADC0_C1_PIN                                          DL_GPIO_PIN_26
#define GPIO_ADC0_C3_PORT                                                  GPIOA
#define GPIO_ADC0_C3_PIN                                          DL_GPIO_PIN_24

/* Defines for ADC1_Temp */
#define ADC1_Temp_INST                                                      ADC1
#define ADC1_Temp_INST_IRQHandler                                ADC1_IRQHandler
#define ADC1_Temp_INST_INT_IRQN                                  (ADC1_INT_IRQn)
#define ADC1_Temp_ADCMEM_Temp1                                DL_ADC12_MEM_IDX_0
#define ADC1_Temp_ADCMEM_Temp1_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC1_Temp_ADCMEM_Temp1_REF_VOLTAGE_V                                     3.3
#define ADC1_Temp_ADCMEM_Temp2                                DL_ADC12_MEM_IDX_1
#define ADC1_Temp_ADCMEM_Temp2_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC1_Temp_ADCMEM_Temp2_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC1_Temp_C0_PORT                                             GPIOA
#define GPIO_ADC1_Temp_C0_PIN                                     DL_GPIO_PIN_15
#define GPIO_ADC1_Temp_C1_PORT                                             GPIOA
#define GPIO_ADC1_Temp_C1_PIN                                     DL_GPIO_PIN_16


/* Defines for VREF */
#define VREF_VOLTAGE_MV                                                     3300
#define GPIO_VREF_VREFPOS_PORT                                             GPIOA
#define GPIO_VREF_VREFPOS_PIN                                     DL_GPIO_PIN_23
#define GPIO_VREF_IOMUX_VREFPOS                                  (IOMUX_PINCM53)
#define GPIO_VREF_IOMUX_VREFPOS_FUNC                IOMUX_PINCM53_PF_UNCONNECTED
#define GPIO_VREF_VREFNEG_PORT                                             GPIOA
#define GPIO_VREF_VREFNEG_PIN                                     DL_GPIO_PIN_21
#define GPIO_VREF_IOMUX_VREFNEG                                  (IOMUX_PINCM46)
#define GPIO_VREF_IOMUX_VREFNEG_FUNC                IOMUX_PINCM46_PF_UNCONNECTED




/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (1)
#define ADC0_INST_DMA_TRIGGER                         (DMA_ADC0_EVT_GEN_BD_TRIG)
/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (0)
#define ADC1_Temp_INST_DMA_TRIGGER                    (DMA_ADC1_EVT_GEN_BD_TRIG)


/* Port definition for Pin Group BSL */
#define BSL_PORT                                                         (GPIOA)

/* Defines for PIN_0: GPIOA.18 with pinCMx 40 on package pin 11 */
#define BSL_PIN_0_PIN                                           (DL_GPIO_PIN_18)
#define BSL_PIN_0_IOMUX                                          (IOMUX_PINCM40)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for PIN_1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define LED_PIN_1_PIN                                            (DL_GPIO_PIN_7)
#define LED_PIN_1_IOMUX                                          (IOMUX_PINCM14)
/* Port definition for Pin Group EN_Charging */
#define EN_Charging_PORT                                                 (GPIOB)

/* Defines for PIN_2: GPIOB.3 with pinCMx 16 on package pin 51 */
#define EN_Charging_PIN_2_PIN                                    (DL_GPIO_PIN_3)
#define EN_Charging_PIN_2_IOMUX                                  (IOMUX_PINCM16)
/* Port definition for Pin Group REQ_Charge */
#define REQ_Charge_PORT                                                  (GPIOB)

/* Defines for PIN_3: GPIOB.15 with pinCMx 32 on package pin 3 */
#define REQ_Charge_PIN_3_PIN                                    (DL_GPIO_PIN_15)
#define REQ_Charge_PIN_3_IOMUX                                   (IOMUX_PINCM32)
/* Port definition for Pin Group IO_Out_2 */
#define IO_Out_2_PORT                                                    (GPIOB)

/* Defines for PIN_4: GPIOB.13 with pinCMx 30 on package pin 1 */
#define IO_Out_2_PIN_4_PIN                                      (DL_GPIO_PIN_13)
#define IO_Out_2_PIN_4_IOMUX                                     (IOMUX_PINCM30)
/* Port definition for Pin Group IO_IN_1 */
#define IO_IN_1_PORT                                                     (GPIOB)

/* Defines for PIN_5: GPIOB.16 with pinCMx 33 on package pin 4 */
#define IO_IN_1_PIN_5_PIN                                       (DL_GPIO_PIN_16)
#define IO_IN_1_PIN_5_IOMUX                                      (IOMUX_PINCM33)
/* Port definition for Pin Group IO_Out_1 */
#define IO_Out_1_PORT                                                    (GPIOA)

/* Defines for PIN_6: GPIOA.12 with pinCMx 34 on package pin 5 */
#define IO_Out_1_PIN_6_PIN                                      (DL_GPIO_PIN_12)
#define IO_Out_1_PIN_6_IOMUX                                     (IOMUX_PINCM34)
/* Port definition for Pin Group IO_IN_2 */
#define IO_IN_2_PORT                                                     (GPIOA)

/* Defines for PIN_7: GPIOA.13 with pinCMx 35 on package pin 6 */
#define IO_IN_2_PIN_7_PIN                                       (DL_GPIO_PIN_13)
#define IO_IN_2_PIN_7_IOMUX                                      (IOMUX_PINCM35)
/* Port definition for Pin Group CdM_j */
#define CdM_j_PORT                                                       (GPIOB)

/* Defines for PIN_9: GPIOB.18 with pinCMx 44 on package pin 15 */
#define CdM_j_PIN_9_PIN                                         (DL_GPIO_PIN_18)
#define CdM_j_PIN_9_IOMUX                                        (IOMUX_PINCM44)
/* Port definition for Pin Group CdM_PD */
#define CdM_PD_PORT                                                      (GPIOB)

/* Defines for PIN_10: GPIOB.19 with pinCMx 45 on package pin 16 */
#define CdM_PD_PIN_10_PIN                                       (DL_GPIO_PIN_19)
#define CdM_PD_PIN_10_IOMUX                                      (IOMUX_PINCM45)
/* Port definition for Pin Group CdM_d2 */
#define CdM_d2_PORT                                                      (GPIOA)

/* Defines for PIN_11: GPIOA.22 with pinCMx 47 on package pin 18 */
#define CdM_d2_PIN_11_PIN                                       (DL_GPIO_PIN_22)
#define CdM_d2_PIN_11_IOMUX                                      (IOMUX_PINCM47)
/* Port definition for Pin Group CdM_d1 */
#define CdM_d1_PORT                                                      (GPIOB)

/* Defines for PIN_12: GPIOB.20 with pinCMx 48 on package pin 19 */
#define CdM_d1_PIN_12_PIN                                       (DL_GPIO_PIN_20)
#define CdM_d1_PIN_12_IOMUX                                      (IOMUX_PINCM48)
/* Port definition for Pin Group GBT_EV_S2 */
#define GBT_EV_S2_PORT                                                   (GPIOB)

/* Defines for PIN_13: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GBT_EV_S2_PIN_13_PIN                                    (DL_GPIO_PIN_21)
#define GBT_EV_S2_PIN_13_IOMUX                                   (IOMUX_PINCM49)
/* Port definition for Pin Group Pluck_Lock_CdM_FB */
#define Pluck_Lock_CdM_FB_PORT                                           (GPIOB)

/* Defines for PIN_14: GPIOB.22 with pinCMx 50 on package pin 21 */
#define Pluck_Lock_CdM_FB_PIN_14_PIN                            (DL_GPIO_PIN_22)
#define Pluck_Lock_CdM_FB_PIN_14_IOMUX                           (IOMUX_PINCM50)
/* Port definition for Pin Group GBT_EV_S2apo */
#define GBT_EV_S2apo_PORT                                                (GPIOB)

/* Defines for PIN_15: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GBT_EV_S2apo_PIN_15_PIN                                 (DL_GPIO_PIN_23)
#define GBT_EV_S2apo_PIN_15_IOMUX                                (IOMUX_PINCM51)
/* Port definition for Pin Group GBT_S1 */
#define GBT_S1_PORT                                                      (GPIOB)

/* Defines for PIN_16: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GBT_S1_PIN_16_PIN                                       (DL_GPIO_PIN_24)
#define GBT_S1_PIN_16_IOMUX                                      (IOMUX_PINCM52)
/* Port definition for Pin Group GBT_S0 */
#define GBT_S0_PORT                                                      (GPIOB)

/* Defines for PIN_17: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GBT_S0_PIN_17_PIN                                       (DL_GPIO_PIN_25)
#define GBT_S0_PIN_17_IOMUX                                      (IOMUX_PINCM56)
/* Port definition for Pin Group Pluck_Lock_nSleep */
#define Pluck_Lock_nSleep_PORT                                           (GPIOB)

/* Defines for PIN_18: GPIOB.26 with pinCMx 57 on package pin 28 */
#define Pluck_Lock_nSleep_PIN_18_PIN                            (DL_GPIO_PIN_26)
#define Pluck_Lock_nSleep_PIN_18_IOMUX                           (IOMUX_PINCM57)
/* Port definition for Pin Group Pluck_Lock_Direction */
#define Pluck_Lock_Direction_PORT                                        (GPIOB)

/* Defines for PIN_19: GPIOB.27 with pinCMx 58 on package pin 29 */
#define Pluck_Lock_Direction_PIN_19_PIN                         (DL_GPIO_PIN_27)
#define Pluck_Lock_Direction_PIN_19_IOMUX                        (IOMUX_PINCM58)
/* Port definition for Pin Group R_Mirror_Out */
#define R_Mirror_Out_PORT                                                (GPIOA)

/* Defines for PIN_20: GPIOA.31 with pinCMx 6 on package pin 39 */
#define R_Mirror_Out_PIN_20_PIN                                 (DL_GPIO_PIN_31)
#define R_Mirror_Out_PIN_20_IOMUX                                 (IOMUX_PINCM6)
/* Port definition for Pin Group Relay_Check */
#define Relay_Check_PORT                                                 (GPIOA)

/* Defines for PIN_21: GPIOA.4 with pinCMx 9 on package pin 44 */
#define Relay_Check_PIN_21_PIN                                   (DL_GPIO_PIN_4)
#define Relay_Check_PIN_21_IOMUX                                  (IOMUX_PINCM9)
/* Port definition for Pin Group RCD_Reset */
#define RCD_Reset_PORT                                                   (GPIOB)

/* Defines for PIN_24: GPIOB.4 with pinCMx 17 on package pin 52 */
#define RCD_Reset_PIN_24_PIN                                     (DL_GPIO_PIN_4)
#define RCD_Reset_PIN_24_IOMUX                                   (IOMUX_PINCM17)
/* Port definition for Pin Group Super_Cap_Char */
#define Super_Cap_Char_PORT                                              (GPIOA)

/* Defines for PIN_25: GPIOA.3 with pinCMx 8 on package pin 43 */
#define Super_Cap_Char_PIN_25_PIN                                (DL_GPIO_PIN_3)
#define Super_Cap_Char_PIN_25_IOMUX                               (IOMUX_PINCM8)
/* Port definition for Pin Group IO_Out_3 */
#define IO_Out_3_PORT                                                    (GPIOB)

/* Defines for PIN_26: GPIOB.7 with pinCMx 24 on package pin 59 */
#define IO_Out_3_PIN_26_PIN                                      (DL_GPIO_PIN_7)
#define IO_Out_3_PIN_26_IOMUX                                    (IOMUX_PINCM24)
/* Port definition for Pin Group RCD_nFault */
#define RCD_nFault_PORT                                                  (GPIOB)

/* Defines for PIN_27: GPIOB.1 with pinCMx 13 on package pin 48 */
#define RCD_nFault_PIN_27_PIN                                    (DL_GPIO_PIN_1)
#define RCD_nFault_PIN_27_IOMUX                                  (IOMUX_PINCM13)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);
void SYSCFG_DL_CCS_CP_init(void);
void SYSCFG_DL_Relay_EN_init(void);
void SYSCFG_DL_PluckLock_init(void);
void SYSCFG_DL_RCD_init(void);
void SYSCFG_DL_AM62L_init(void);
void SYSCFG_DL_Debug_init(void);
void SYSCFG_DL_ADC0_init(void);
void SYSCFG_DL_ADC1_Temp_init(void);
void SYSCFG_DL_VREF_init(void);
void SYSCFG_DL_DMA_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
