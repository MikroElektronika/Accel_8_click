/*
    __accel8_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__accel8_driver.h"
#include "__accel8_hal.c"

/* ------------------------------------------------------------------- MACROS */

/* Register */
const uint8_t _ACCEL8_REG_SELF_TEST_X          = 0x0D; 
const uint8_t _ACCEL8_REG_SELF_TEST_Y          = 0x0E; 
const uint8_t _ACCEL8_REG_SELF_TEST_Z          = 0x0F; 
const uint8_t _ACCEL8_REG_SELF_TEST_A          = 0x10; 
const uint8_t _ACCEL8_REG_SMPLRT_DIV           = 0x19;
const uint8_t _ACCEL8_REG_CONFIG               = 0x1A;
const uint8_t _ACCEL8_REG_GYRO_CONFIG          = 0x1B;
const uint8_t _ACCEL8_REG_ACCEL_CONFIG         = 0x1C;
const uint8_t _ACCEL8_REG_FIFO_EN              = 0x23; 
const uint8_t _ACCEL8_REG_I2C_MST_CTRL         = 0x24;
const uint8_t _ACCEL8_REG_I2C_SLV0_ADDR        = 0x25;
const uint8_t _ACCEL8_REG_I2C_SLV0_REG         = 0x26;
const uint8_t _ACCEL8_REG_I2C_SLV0_CTRL        = 0x27;
const uint8_t _ACCEL8_REG_I2C_SLV1_ADDR        = 0x28;
const uint8_t _ACCEL8_REG_I2C_SLV1_REG         = 0x29;
const uint8_t _ACCEL8_REG_I2C_SLV1_CTRL        = 0x2A;
const uint8_t _ACCEL8_REG_I2C_SLV2_ADDR        = 0x2B;
const uint8_t _ACCEL8_REG_I2C_SLV2_REG         = 0x2C;
const uint8_t _ACCEL8_REG_I2C_SLV2_CTRL        = 0x2D;
const uint8_t _ACCEL8_REG_I2C_SLV3_ADDR        = 0x2E;
const uint8_t _ACCEL8_REG_I2C_SLV3_REG         = 0x2F;
const uint8_t _ACCEL8_REG_I2C_SLV3_CTRL        = 0x30;
const uint8_t _ACCEL8_REG_I2C_SLV4_ADDR        = 0x31;
const uint8_t _ACCEL8_REG_I2C_SLV4_REG         = 0x32;
const uint8_t _ACCEL8_REG_I2C_SLV4_DO          = 0x33;
const uint8_t _ACCEL8_REG_I2C_SLV4_CTRL        = 0x34;
const uint8_t _ACCEL8_REG_I2C_SLV4_DI          = 0x35;
const uint8_t _ACCEL8_REG_I2C_MST_STATUS       = 0x36;
const uint8_t _ACCEL8_REG_INT_PIN_CFG          = 0x37;
const uint8_t _ACCEL8_REG_INT_ENABLE           = 0x38;
const uint8_t _ACCEL8_REG_INT_STATUS           = 0x3A;
const uint8_t _ACCEL8_REG_ACCEL_XOUT_H         = 0x3B;
const uint8_t _ACCEL8_REG_ACCEL_XOUT_L         = 0x3C;
const uint8_t _ACCEL8_REG_ACCEL_YOUT_H         = 0x3D;
const uint8_t _ACCEL8_REG_ACCEL_YOUT_L         = 0x3E;
const uint8_t _ACCEL8_REG_ACCEL_ZOUT_H         = 0x3F;
const uint8_t _ACCEL8_REG_ACCEL_ZOUT_L         = 0x40;
const uint8_t _ACCEL8_REG_TEMP_OUT_H           = 0x41;
const uint8_t _ACCEL8_REG_TEMP_OUT_L           = 0x42;
const uint8_t _ACCEL8_REG_GYRO_XOUT_H          = 0x43;
const uint8_t _ACCEL8_REG_GYRO_XOUT_L          = 0x44;
const uint8_t _ACCEL8_REG_GYRO_YOUT_H          = 0x45;
const uint8_t _ACCEL8_REG_GYRO_YOUT_L          = 0x46;
const uint8_t _ACCEL8_REG_GYRO_ZOUT_H          = 0x47;
const uint8_t _ACCEL8_REG_GYRO_ZOUT_L          = 0x48;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_00     = 0x49;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_01     = 0x4A;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_02     = 0x4B;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_03     = 0x4C;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_04     = 0x4D;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_05     = 0x4E;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_06     = 0x4F;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_07     = 0x50;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_08     = 0x51;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_09     = 0x52;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_10     = 0x53;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_11     = 0x54;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_12     = 0x55;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_13     = 0x56;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_14     = 0x57;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_15     = 0x58;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_16     = 0x59;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_17     = 0x5A;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_18     = 0x5B;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_19     = 0x5C;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_20     = 0x5D;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_21     = 0x5E;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_22     = 0x5F;
const uint8_t _ACCEL8_REG_EXT_SENS_DATA_23     = 0x60;
const uint8_t _ACCEL8_REG_I2C_SLV0_DO          = 0x63;
const uint8_t _ACCEL8_REG_I2C_SLV1_DO          = 0x64;
const uint8_t _ACCEL8_REG_I2C_SLV2_DO          = 0x65;
const uint8_t _ACCEL8_REG_I2C_SLV3_DO          = 0x66;
const uint8_t _ACCEL8_REG_I2C_MST_DELAY_CTRL   = 0x67;
const uint8_t _ACCEL8_REG_SIGNAL_PATH_RESET    = 0x68;
const uint8_t _ACCEL8_REG_USER_CTRL            = 0x6A;
const uint8_t _ACCEL8_REG_PWR_MGMT_1           = 0x6B;
const uint8_t _ACCEL8_REG_PWR_MGMT_2           = 0x6C;
const uint8_t _ACCEL8_REG_FIFO_COUNTH          = 0x72;
const uint8_t _ACCEL8_REG_FIFO_COUNTL          = 0x73;
const uint8_t _ACCEL8_REG_FIFO_R_W             = 0x74;
const uint8_t _ACCEL8_REG_WHO_AM_I             = 0x75;

/* Configuration */
const uint8_t _ACCEL8_CFG_EXT_SYNC_INPUT_DISABLED = 0x00 << 3;
const uint8_t _ACCEL8_CFG_EXT_SYNC_TEMP_OUTPUT    = 0x01 << 3;
const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_X_OUTPUT  = 0x02 << 3;
const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_Y_OUTPUT  = 0x03 << 3;
const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_Z_OUTPUT  = 0x04 << 3;
const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_X_OUTPUT = 0x05 << 3;
const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_Y_OUTPUT = 0x06 << 3;
const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_Z_OUTPUT = 0x07 << 3;
const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A260_G256   = 0x00;
const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A184_G188   = 0x01;
const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A94_G98     = 0x02;
const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A44_G42     = 0x03;
const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A21_G20     = 0x04;
const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A10_G10     = 0x05;
const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A5_G5       = 0x06;

/* Gyroscope Configuration */
const uint8_t _ACCEL8_GYRO_CFG_X_SELF_TEST              = 0x80;
const uint8_t _ACCEL8_GYRO_CFG_Y_SELF_TEST              = 0x40;
const uint8_t _ACCEL8_GYRO_CFG_Z_SELF_TEST              = 0x20;
const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_250dbs  = 0x00;
const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_500dbs  = 0x08;
const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_1000dbs = 0x10;
const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_2000dbs = 0x18;

/* Accelerometer Configuration */
const uint8_t _ACCEL8_ACCEL_CFG_X_SELF_TEST          = 0x80;
const uint8_t _ACCEL8_ACCEL_CFG_Y_SELF_TEST          = 0x40;
const uint8_t _ACCEL8_ACCEL_CFG_Z_SELF_TEST          = 0x20;
const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_2g  = 0x00;
const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_4g  = 0x08;
const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_8g  = 0x10;
const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_16g = 0x18;

/* FIFO Enable */
const uint8_t _ACCEL8_FIFO_ENABLE_TEMP               = 0x80;
const uint8_t _ACCEL8_FIFO_ENABLE_X_AXIS_GYRO        = 0x40;
const uint8_t _ACCEL8_FIFO_ENABLE_Y_AXIS_GYRO        = 0x20;
const uint8_t _ACCEL8_FIFO_ENABLE_Z_AXIS_GYRO        = 0x10;
const uint8_t _ACCEL8_FIFO_ENABLE_ACCEL              = 0x08;
const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV2 = 0x04;
const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV1 = 0x02;
const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV0 = 0x01;

/* I2C Master Control */
const uint8_t _ACCEL8_I2C_MST_CTRL_MUL_MST_ENABLE            = 0x80;
const uint8_t _ACCEL8_I2C_MST_CTRL_WAIT_FOR_ES               = 0x40;
const uint8_t _ACCEL8_I2C_MST_CTRL_EXT_SENS_DATA_SLV3        = 0x20;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_RESTART_BETWEEN_READS = 0x00;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_STOP_AND_START        = 0x10;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_348kHz          = 0x00;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_333kHz          = 0x01;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_320kHz          = 0x02;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_308kHz          = 0x03;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_296kHz          = 0x04;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_286kHz          = 0x05;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_276kHz          = 0x06;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_367kHz          = 0x07;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_358kHz          = 0x08;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_500kHz          = 0x09;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_471kHz          = 0x0A;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_444kHz          = 0x0B;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_421kHz          = 0x0C;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_400kHz          = 0x0D;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_381kHz          = 0x0E;
const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_364kHz          = 0x0F;

/* INT Pin / Bypass Enable Configuration */
const uint8_t _ACCEL8_INTC_INT_PIN_IS_ACTIVE_HIGH               = 0x00;
const uint8_t _ACCEL8_INTC_INT_PIN_IS_ACTIVE_LOW                = 0x80;
const uint8_t _ACCEL8_INTC_INT_PIN_IS_CONFIGURED_AS_PUSH_PULL   = 0x00;
const uint8_t _ACCEL8_INTC_INT_PIN_IS_CONFIGURED_AS_OPEN_DRAIN  = 0x40;
const uint8_t _ACCEL8_INTC_LATCH_INT_ENABLE                     = 0x20;
const uint8_t _ACCEL8_INTC_INT_READ_CLEAR                       = 0x10;
const uint8_t _ACCEL8_INTC_FSYNC_INT_LEVEL_ACTIVE_HIGH          = 0x00;
const uint8_t _ACCEL8_INTC_FSYNC_INT_LEVEL_ACTIVE_LOW           = 0x08;
const uint8_t _ACCEL8_INTC_FSYNC_INT_ENABLE                     = 0x04;
const uint8_t _ACCEL8_INTC_I2C_BYPASS_ENABLE                    = 0x02;
const uint8_t _ACCEL8_INTC_I2C_BYPASS_DISABLE                   = 0x00;

/* Interrupt Enable */
const uint8_t _ACCEL8_INTE_FIFO_OFLOW_ENABLE   = 0x10;
const uint8_t _ACCEL8_INTE_I2C_MST_INT_ENABLE  = 0x08;
const uint8_t _ACCEL8_INTE_DATA_RDY_ENABLE     = 0x01;

/* Interrupt Status */
const uint8_t _ACCEL8_INTS_FIFO_OFLOW  = 0x10;
const uint8_t _ACCEL8_INTS_I2C_MST_INT = 0x08;
const uint8_t _ACCEL8_INTS_DATA_RDY    = 0x01;

/* DATA OUTPUT */
const uint8_t _ACCEL8_ACCEL_X_AXIS_DATA = 0x3B;
const uint8_t _ACCEL8_ACCEL_Y_AXIS_DATA = 0x3D;
const uint8_t _ACCEL8_ACCEL_Z_AXIS_DATA = 0x3F;
const uint8_t _ACCEL8_TEMP_DATA   = 0x41;
const uint8_t _ACCEL8_GYRO_X_AXIS_DATA  = 0x43;
const uint8_t _ACCEL8_GYRO_Y_AXIS_DATA  = 0x45;
const uint8_t _ACCEL8_GYRO_Z_AXIS_DATA  = 0x47;

/* RESET */
const uint8_t _ACCEL8_GYRO_RESET   = 0x04;
const uint8_t _ACCEL8_ACCEL_RESET  = 0x02;
const uint8_t _ACCEL8_TEMP_RESET   = 0x01;

/* USER CONTROL */
const uint8_t _ACCEL8_UC_FIFO_ENABLE       = 0x40;
const uint8_t _ACCEL8_UC_I2C_MASTER_MODE   = 0x20;
const uint8_t _ACCEL8_UC_FIFO_RESET        = 0x04;
const uint8_t _ACCEL8_UC_I2C_MASTER_RESET  = 0x02;
const uint8_t _ACCEL8_UC_SIG_COND_RESET    = 0x01;

/* Power Management 1 */
const uint8_t _ACCEL8_PM1_DEVICE_RESET                       = 0x80;
const uint8_t _ACCEL8_PM1_GO_TO_SLEEP                        = 0x40;
const uint8_t _ACCEL8_PM1_CYCLE                              = 0x20;
const uint8_t _ACCEL8_PM1_TEMP_DISABLE                       = 0x08;
const uint8_t _ACCEL8_PM1_CLKSEL_INTERNAL_8MHZ_OSCILLATOR    = 0x00;
const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_X_AXIS_GYROSCOPE   = 0x01;
const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_Y_AXIS_GYROSCOPE   = 0x02;
const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_Z_AXIS_GYROSCOPE   = 0x03;
const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_EXTERNAL_32_768kHz = 0x04;
const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_EXTERNAL_19_2kHz   = 0x05;
const uint8_t _ACCEL8_PM1_CLKSEL_STOPS_THE_CLOCK             = 0x07;

/* Power Management 2 */
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_1_25Hz         = 0x00;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_5Hz            = 0x40;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_20Hz           = 0x80;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_40Hz           = 0xC0;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_X_AXIS_ACCEL_STANDBY_MODE   = 0x20;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Y_AXIS_ACCEL_STANDBY_MODE   = 0x10;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Z_AXIS_ACCEL_STANDBY_MODE   = 0x08;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_X_AXIS_GYRO_STANDBY_MODE    = 0x04;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Y_AXIS_GYRO_STANDBY_MODE    = 0x02;
const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Z_AXIS_GYRO_STANDBY_MODE    = 0x01;

/* ADDRESS */
const uint8_t _ACCEL8_DEVICE_SLAVE_ADDRESS_ADD = 0x69;
const uint8_t _ACCEL8_DEVICE_SLAVE_ADDRESS_SEL = 0x68;

/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __ACCEL8_DRV_I2C__
static uint8_t _slaveAddress;
#endif

static uint8_t _accelRange;
static uint16_t _gyroRange;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __ACCEL8_DRV_SPI__

void accel8_spiDriverInit(T_ACCEL8_P gpioObj, T_ACCEL8_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );
}

#endif
#ifdef   __ACCEL8_DRV_I2C__

void accel8_i2cDriverInit(T_ACCEL8_P gpioObj, T_ACCEL8_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );
}

#endif
#ifdef   __ACCEL8_DRV_UART__

void accel8_uartDriverInit(T_ACCEL8_P gpioObj, T_ACCEL8_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );
}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */

void accel8_writeByte(uint8_t reg, uint8_t _data)
{
    uint8_t writeReg[ 2 ];

    writeReg[ 0 ] = reg;
    writeReg[ 1 ] = _data;
    
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, writeReg, 2, END_MODE_STOP);
}

uint8_t accel8_readByte(uint8_t reg)
{
    uint8_t writeReg[ 1 ];
    uint8_t readReg[ 1 ];
    
    writeReg[ 0 ] = reg;
    
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, writeReg, 1, END_MODE_RESTART);
    hal_i2cRead(_slaveAddress, readReg, 1, END_MODE_STOP);
    
    return readReg[ 0 ];
}

uint16_t accel8_readData(uint8_t reg)
{
    uint8_t writeReg[ 1 ];
    uint8_t readReg[ 2 ];
    uint16_t readData;

    writeReg[ 0 ] = reg;

    hal_i2cStart();
    hal_i2cWrite(_slaveAddress, writeReg, 1, END_MODE_RESTART);
    hal_i2cRead(_slaveAddress, readReg, 2, END_MODE_STOP);

    readData = readReg[ 0 ];
    readData = readData << 8;
    readData = readData | readReg[ 1 ];
    
    return readData;
}

float accel8_getTemperature()
{
    // NE RADI - int16_t Temp_out;
    volatile int16_t Temp_out;

    Temp_out = accel8_readData(_ACCEL8_TEMP_DATA);
    return (((float)Temp_out / 340.0f) + 36.53f);
}

void accel8_getAccelAxis(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis)
{
    *x_axis = accel8_readData(_ACCEL8_ACCEL_X_AXIS_DATA);
    *y_axis = accel8_readData(_ACCEL8_ACCEL_Y_AXIS_DATA);
    *z_axis = accel8_readData(_ACCEL8_ACCEL_Z_AXIS_DATA);
}

void accel8_getGyroAxis(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis)
{
    *x_axis = accel8_readData(_ACCEL8_GYRO_X_AXIS_DATA);
    *y_axis = accel8_readData(_ACCEL8_GYRO_Y_AXIS_DATA);
    *z_axis = accel8_readData(_ACCEL8_GYRO_Z_AXIS_DATA);
}

uint8_t accel8_accelConfig(uint8_t cfg)
{
    uint8_t range;
     
    accel8_writeByte(_ACCEL8_REG_ACCEL_CONFIG, cfg);
    range = cfg >> 3;
     
    switch( range )
    {
        case 0:
        {
            _accelRange = 2;
            break;
        }
        case 1:
        {
            _accelRange = 4;
            break;
        }
        case 2:
        {
            _accelRange = 8;
            break;
        }
        case 3:
        {
            _accelRange = 16;
            break;
        }
    }
    return _accelRange;
}

uint16_t accel8_gyroConfig(uint8_t cfg)
{
    uint8_t range;
    accel8_writeByte(_ACCEL8_REG_GYRO_CONFIG, cfg);
    
    range = cfg >> 3;

    switch( range )
    {
        case 0:
        {
            _gyroRange = 250;
            break;
        }
        case 1:
        {
            _gyroRange = 500;
            break;
        }
        case 2:
        {
            _gyroRange = 1000;
            break;
        }
        case 3:
        {
            _gyroRange = 2000;
            break;
        }
    }
    return _gyroRange;
}

uint8_t accel8_getInterrupt()
{
    return hal_gpio_intGet();
}


/* -------------------------------------------------------------------------- */
/*
  __accel8_driver.c

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