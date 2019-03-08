/*
    __accel8_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __accel8_driver.h
@brief    Accel8 Driver
@mainpage Accel8 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   ACCEL8
@brief      Accel8 Click Driver
@{

| Global Library Prefix | **ACCEL8** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **okt 2018.**      |
| Developer             | **Katarina Perendic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _ACCEL8_H_
#define _ACCEL8_H_

/** 
 * @macro T_ACCEL8_P
 * @brief Driver Abstract type 
 */
#define T_ACCEL8_P    const uint8_t*

/** @defgroup ACCEL8_COMPILE Compilation Config */              /** @{ */

//  #define   __ACCEL8_DRV_SPI__                            /**<     @macro __ACCEL8_DRV_SPI__  @brief SPI driver selector */
   #define   __ACCEL8_DRV_I2C__                            /**<     @macro __ACCEL8_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __ACCEL8_DRV_UART__                           /**<     @macro __ACCEL8_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup ACCEL8_VAR Variables */                           /** @{ */

/* Register */
extern const uint8_t _ACCEL8_REG_SELF_TEST_X          ;
extern const uint8_t _ACCEL8_REG_SELF_TEST_Y          ;
extern const uint8_t _ACCEL8_REG_SELF_TEST_Z          ;
extern const uint8_t _ACCEL8_REG_SELF_TEST_A          ;
extern const uint8_t _ACCEL8_REG_SMPLRT_DIV           ;
extern const uint8_t _ACCEL8_REG_CONFIG               ;
extern const uint8_t _ACCEL8_REG_GYRO_CONFIG          ;
extern const uint8_t _ACCEL8_REG_ACCEL_CONFIG         ;
extern const uint8_t _ACCEL8_REG_FIFO_EN              ;
extern const uint8_t _ACCEL8_REG_I2C_MST_CTRL         ;
extern const uint8_t _ACCEL8_REG_I2C_SLV0_ADDR        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV0_REG         ;
extern const uint8_t _ACCEL8_REG_I2C_SLV0_CTRL        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV1_ADDR        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV1_REG         ;
extern const uint8_t _ACCEL8_REG_I2C_SLV1_CTRL        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV2_ADDR        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV2_REG         ;
extern const uint8_t _ACCEL8_REG_I2C_SLV2_CTRL        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV3_ADDR        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV3_REG         ;
extern const uint8_t _ACCEL8_REG_I2C_SLV3_CTRL        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_ADDR        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_REG         ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_DO          ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_CTRL        ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_DI          ;
extern const uint8_t _ACCEL8_REG_I2C_MST_STATUS       ;
extern const uint8_t _ACCEL8_REG_INT_PIN_CFG          ;
extern const uint8_t _ACCEL8_REG_INT_ENABLE           ;
extern const uint8_t _ACCEL8_REG_INT_STATUS           ;
extern const uint8_t _ACCEL8_REG_ACCEL_XOUT_H         ;
extern const uint8_t _ACCEL8_REG_ACCEL_XOUT_L         ;
extern const uint8_t _ACCEL8_REG_ACCEL_YOUT_H         ;
extern const uint8_t _ACCEL8_REG_ACCEL_YOUT_L         ;
extern const uint8_t _ACCEL8_REG_ACCEL_ZOUT_H         ;
extern const uint8_t _ACCEL8_REG_ACCEL_ZOUT_L         ;
extern const uint8_t _ACCEL8_REG_TEMP_OUT_H           ;
extern const uint8_t _ACCEL8_REG_TEMP_OUT_L           ;
extern const uint8_t _ACCEL8_REG_GYRO_XOUT_H          ;
extern const uint8_t _ACCEL8_REG_GYRO_XOUT_L          ;
extern const uint8_t _ACCEL8_REG_GYRO_YOUT_H          ;
extern const uint8_t _ACCEL8_REG_GYRO_YOUT_L          ;
extern const uint8_t _ACCEL8_REG_GYRO_ZOUT_H          ;
extern const uint8_t _ACCEL8_REG_GYRO_ZOUT_L          ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_00     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_01     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_02     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_03     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_04     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_05     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_06     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_07     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_08     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_09     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_10     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_11     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_12     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_13     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_14     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_15     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_16     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_17     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_18     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_19     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_20     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_21     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_22     ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_23     ;
extern const uint8_t _ACCEL8_REG_I2C_SLV0_DO          ;
extern const uint8_t _ACCEL8_REG_I2C_SLV1_DO          ;
extern const uint8_t _ACCEL8_REG_I2C_SLV2_DO          ;
extern const uint8_t _ACCEL8_REG_I2C_SLV3_DO          ;
extern const uint8_t _ACCEL8_REG_I2C_MST_DELAY_CTRL   ;
extern const uint8_t _ACCEL8_REG_SIGNAL_PATH_RESET    ;
extern const uint8_t _ACCEL8_REG_MOT_DETECT_CTRL      ;
extern const uint8_t _ACCEL8_REG_USER_CTRL            ;
extern const uint8_t _ACCEL8_REG_PWR_MGMT_1           ;
extern const uint8_t _ACCEL8_REG_PWR_MGMT_2           ;
extern const uint8_t _ACCEL8_REG_FIFO_COUNTH          ;
extern const uint8_t _ACCEL8_REG_FIFO_COUNTL          ;
extern const uint8_t _ACCEL8_REG_FIFO_R_W             ;
extern const uint8_t _ACCEL8_REG_WHO_AM_I             ;

/* Configuration */
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_INPUT_DISABLED ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_TEMP_OUTPUT    ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_X_OUTPUT  ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_Y_OUTPUT  ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_Z_OUTPUT  ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_X_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_Y_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_Z_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A260_G256   ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A184_G188   ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A94_G98     ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A44_G42     ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A21_G20     ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A10_G10     ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A5_G5       ;

/* Gyroscope Configuration */
extern const uint8_t _ACCEL8_GYRO_CFG_X_SELF_TEST              ;
extern const uint8_t _ACCEL8_GYRO_CFG_Y_SELF_TEST              ;
extern const uint8_t _ACCEL8_GYRO_CFG_Z_SELF_TEST              ;
extern const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_250dbs  ;
extern const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_500dbs  ;
extern const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_1000dbs ;
extern const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_2000dbs ;

/* Accelerometer Configuration */
extern const uint8_t _ACCEL8_ACCEL_CFG_X_SELF_TEST          ;
extern const uint8_t _ACCEL8_ACCEL_CFG_Y_SELF_TEST          ;
extern const uint8_t _ACCEL8_ACCEL_CFG_Z_SELF_TEST          ;
extern const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_2g  ;
extern const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_4g  ;
extern const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_8g  ;
extern const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_16g ;

/* FIFO Enable */
extern const uint8_t _ACCEL8_FIFO_ENABLE_TEMP               ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_X_AXIS_GYRO        ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_Y_AXIS_GYRO        ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_Z_AXIS_GYRO        ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_ACCEL              ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV2 ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV1 ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV0 ;

/* I2C Master Control */
extern const uint8_t _ACCEL8_I2C_MST_CTRL_MUL_MST_ENABLE            ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_WAIT_FOR_ES               ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_EXT_SENS_DATA_SLV3        ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_RESTART_BETWEEN_READS ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_STOP_AND_START        ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_348kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_333kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_320kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_308kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_296kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_286kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_276kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_367kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_358kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_500kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_471kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_444kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_421kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_400kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_381kHz          ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_364kHz          ;

/* INT Pin / Bypass Enable Configuration */
extern const uint8_t _ACCEL8_INTC_INT_PIN_IS_ACTIVE_HIGH               ;
extern const uint8_t _ACCEL8_INTC_INT_PIN_IS_ACTIVE_LOW                ;
extern const uint8_t _ACCEL8_INTC_INT_PIN_IS_CONFIGURED_AS_PUSH_PULL   ;
extern const uint8_t _ACCEL8_INTC_INT_PIN_IS_CONFIGURED_AS_OPEN_DRAIN  ;
extern const uint8_t _ACCEL8_INTC_LATCH_INT_ENABLE                     ;
extern const uint8_t _ACCEL8_INTC_INT_READ_CLEAR                       ;
extern const uint8_t _ACCEL8_INTC_FSYNC_INT_LEVEL_ACTIVE_HIGH          ;
extern const uint8_t _ACCEL8_INTC_FSYNC_INT_LEVEL_ACTIVE_LOW           ;
extern const uint8_t _ACCEL8_INTC_FSYNC_INT_ENABLE                     ;
extern const uint8_t _ACCEL8_INTC_I2C_BYPASS_ENABLE                    ;
extern const uint8_t _ACCEL8_INTC_I2C_BYPASS_DISABLE                   ;

/* Interrupt Enable */
extern const uint8_t _ACCEL8_INTE_FIFO_OFLOW_ENABLE  ;
extern const uint8_t _ACCEL8_INTE_I2C_MST_INT_ENABLE ;
extern const uint8_t _ACCEL8_INTE_DATA_RDY_ENABLE    ;

/* Interrupt Status */
extern const uint8_t _ACCEL8_INTS_FIFO_OFLOW  ;
extern const uint8_t _ACCEL8_INTS_I2C_MST_INT ;
extern const uint8_t _ACCEL8_INTS_DATA_RDY    ;

/* DATA OUTPUT */
extern const uint8_t _ACCEL8_ACCEL_X_AXIS_DATA ;
extern const uint8_t _ACCEL8_ACCEL_Y_AXIS_DATA ;
extern const uint8_t _ACCEL8_ACCEL_Z_AXIS_DATA ;
extern const uint8_t _ACCEL8_TEMP_DATA         ;
extern const uint8_t _ACCEL8_GYRO_X_AXIS_DATA  ;
extern const uint8_t _ACCEL8_GYRO_Y_AXIS_DATA  ;
extern const uint8_t _ACCEL8_GYRO_Z_AXIS_DATA  ;

/* RESET */
extern const uint8_t _ACCEL8_GYRO_RESET ;
extern const uint8_t _ACCEL8_ACCEL_RESET;
extern const uint8_t _ACCEL8_TEMP_RESET ;

/* USER CONTROL */
extern const uint8_t _ACCEL8_UC_FIFO_ENABLE      ;
extern const uint8_t _ACCEL8_UC_I2C_MASTER_MODE  ;
extern const uint8_t _ACCEL8_UC_FIFO_RESET       ;
extern const uint8_t _ACCEL8_UC_I2C_MASTER_RESET ;
extern const uint8_t _ACCEL8_UC_SIG_COND_RESET   ;

/* Power Management 1 */
extern const uint8_t _ACCEL8_PM1_DEVICE_RESET                       ;
extern const uint8_t _ACCEL8_PM1_GO_TO_SLEEP                        ;
extern const uint8_t _ACCEL8_PM1_CYCLE                              ;
extern const uint8_t _ACCEL8_PM1_TEMP_DISABLE                       ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_INTERNAL_8MHZ_OSCILLATOR    ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_X_AXIS_GYROSCOPE   ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_Y_AXIS_GYROSCOPE   ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_Z_AXIS_GYROSCOPE   ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_EXTERNAL_32_768kHz ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_EXTERNAL_19_2kHz   ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_STOPS_THE_CLOCK             ;

/* Power Management 2 */
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_1_25Hz         ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_5Hz            ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_20Hz           ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_40Hz           ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_X_AXIS_ACCEL_STANDBY_MODE   ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Y_AXIS_ACCEL_STANDBY_MODE   ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Z_AXIS_ACCEL_STANDBY_MODE   ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_X_AXIS_GYRO_STANDBY_MODE    ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Y_AXIS_GYRO_STANDBY_MODE    ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Z_AXIS_GYRO_STANDBY_MODE    ;

/* ADDRESS */
extern const uint8_t _ACCEL8_DEVICE_SLAVE_ADDRESS_ADD ;
extern const uint8_t _ACCEL8_DEVICE_SLAVE_ADDRESS_SEL ;


                                                                       /** @} */
/** @defgroup ACCEL8_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup ACCEL8_INIT Driver Initialization */              /** @{ */

#ifdef   __ACCEL8_DRV_SPI__
void accel8_spiDriverInit(T_ACCEL8_P gpioObj, T_ACCEL8_P spiObj);
#endif
#ifdef   __ACCEL8_DRV_I2C__
void accel8_i2cDriverInit(T_ACCEL8_P gpioObj, T_ACCEL8_P i2cObj, uint8_t slave);
#endif
#ifdef   __ACCEL8_DRV_UART__
void accel8_uartDriverInit(T_ACCEL8_P gpioObj, T_ACCEL8_P uartObj);
#endif

// GPIO Only Drivers - remove in other cases
void accel8_gpioDriverInit(T_ACCEL8_P gpioObj);
                                                                       /** @} */
/** @defgroup ACCEL8_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Functions for write one byte in register
 *
 * @param[in] reg    Register in which the data will be written
 * @param[in] _data  Data which be written in the register
 */
void accel8_writeByte(uint8_t reg, uint8_t _data);

/**
 * @brief Functions for read byte from register
 *
 * @param[in] reg    Register which will be read
 * @retval one byte data which is read from the register
 */
uint8_t accel8_readByte(uint8_t reg);

/**
 * @brief Functions for read data from register
 *
 * @param[in] reg    Register which will be read
 * @retval 2 byte data which is read from the register
 */
uint16_t accel8_readData(uint8_t reg);

/**
 * @brief Functions for read Temperature data in C
 *
 * @retval Temperature data
 *
 * @Note
     Formule :
         Temperature = Temp_OUT / 340 + 36.53
         Temp_OUT = 16bit Temp data
 */
float accel8_getTemperature();

/**
 * @brief Functions for read Accel axis data
 *
 * @param[out] x_axis    Accel X axis data
 * @param[out] y_axis    Accel Y axis data
 * @param[out] z_axis    Accel Z axis data
 *
 */
void accel8_getAccelAxis(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis);

/**
 * @brief Functions for read Gyro axis data
 *
 * @param[out] x_axis    Gyro X axis data
 * @param[out] y_axis    Gyro Y axis data
 * @param[out] z_axis    Gyro Z axis data
 *
 */
void accel8_getGyroAxis(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis);

/**
 * @brief Functions for accel configuration
 *
 * @param[in] cfg        Accel configuration
 * @return Accel output data range in g
 */
uint8_t accel8_accelConfig(uint8_t cfg);

/**
 * @brief Functions for Gyro configuration
 *
 * @param[in] cfg        Gyro configuration
 * @return Gyro output data range in degrees per second( dps )
 */
uint16_t accel8_gyroConfig(uint8_t cfg);

/**
 * @brief Functions for read INT pin state
 *
 * @retval Interrupt state
 */
uint8_t accel8_getInterrupt();
 
                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Accel8_STM.c
    @example Click_Accel8_TIVA.c
    @example Click_Accel8_CEC.c
    @example Click_Accel8_KINETIS.c
    @example Click_Accel8_MSP.c
    @example Click_Accel8_PIC.c
    @example Click_Accel8_PIC32.c
    @example Click_Accel8_DSPIC.c
    @example Click_Accel8_AVR.c
    @example Click_Accel8_FT90x.c
    @example Click_Accel8_STM.mbas
    @example Click_Accel8_TIVA.mbas
    @example Click_Accel8_CEC.mbas
    @example Click_Accel8_KINETIS.mbas
    @example Click_Accel8_MSP.mbas
    @example Click_Accel8_PIC.mbas
    @example Click_Accel8_PIC32.mbas
    @example Click_Accel8_DSPIC.mbas
    @example Click_Accel8_AVR.mbas
    @example Click_Accel8_FT90x.mbas
    @example Click_Accel8_STM.mpas
    @example Click_Accel8_TIVA.mpas
    @example Click_Accel8_CEC.mpas
    @example Click_Accel8_KINETIS.mpas
    @example Click_Accel8_MSP.mpas
    @example Click_Accel8_PIC.mpas
    @example Click_Accel8_PIC32.mpas
    @example Click_Accel8_DSPIC.mpas
    @example Click_Accel8_AVR.mpas
    @example Click_Accel8_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __accel8_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */