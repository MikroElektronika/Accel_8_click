/*
Example for Accel8 Click

    Date          : okt 2018.
    Author        : Katarina Perendic

Test configuration TIVA :
    
    MCU              : TM4C129XNCZAD
    Dev. Board       : EasyMx PRO v7 for TIVA ARM
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes I2C module and sets INT pin as INPUT
- Application Initialization - Initialization driver init, reset chip and start configuration chip for measurement
- Application Task - (code snippet) - Reads Accel X/Y/Z axis, Gyro X/Y/Z axis and device Temperature. 
                                      All data logs on the USBUART every 2 sec.

*/

#include "Click_Accel8_types.h"
#include "Click_Accel8_config.h"

float Temperature;
int16_t X_gyroAxis;
int16_t Y_gyroAxis;
int16_t Z_gyroAxis;
int16_t X_accelAxis;
int16_t Y_accelAxis;
int16_t Z_accelAxis;

uint16_t  gyroRange;
uint8_t accelRange;

char demoText[ 50 ];

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
    mikrobus_i2cInit( _MIKROBUS1, &_ACCEL8_I2C_CFG[0] );
    mikrobus_logInit( _LOG_USBUART_A, 115200 );
    mikrobus_logWrite(" --- System Init ---", _LOG_LINE);
    Delay_ms( 100 );
}

void applicationInit()
{
    accel8_i2cDriverInit( (T_ACCEL8_P)&_MIKROBUS1_GPIO, (T_ACCEL8_P)&_MIKROBUS1_I2C, _ACCEL8_DEVICE_SLAVE_ADDRESS_ADD );
    
    mikrobus_logWrite(" --- Device Reset --- ", _LOG_LINE );
    accel8_writeByte(_ACCEL8_REG_PWR_MGMT_1, _ACCEL8_PM1_DEVICE_RESET);
    accel8_writeByte(_ACCEL8_REG_SIGNAL_PATH_RESET, _ACCEL8_GYRO_RESET | _ACCEL8_ACCEL_RESET | _ACCEL8_TEMP_RESET );
    
    mikrobus_logWrite(" --- Device Configuration --- ", _LOG_LINE );
    accel8_writeByte(_ACCEL8_REG_PWR_MGMT_1, _ACCEL8_PM1_CLKSEL_INTERNAL_8MHZ_OSCILLATOR);
    accel8_writeByte(_ACCEL8_REG_INT_ENABLE, _ACCEL8_INTE_DATA_RDY_ENABLE );
    
    gyroRange = accel8_gyroConfig(_ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_250dbs);
    accelRange = accel8_accelConfig(_ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_2g);
   
    accel8_writeByte(_ACCEL8_REG_FIFO_EN, _ACCEL8_FIFO_ENABLE_TEMP | 
                                          _ACCEL8_FIFO_ENABLE_X_AXIS_GYRO |
                                          _ACCEL8_FIFO_ENABLE_Y_AXIS_GYRO | 
                                          _ACCEL8_FIFO_ENABLE_Z_AXIS_GYRO |
                                          _ACCEL8_FIFO_ENABLE_ACCEL );
    Delay_ms( 1000 );
    mikrobus_logWrite(" --- Start Measurement --- ", _LOG_LINE);
}

void applicationTask()
{
    accel8_getAccelAxis(&X_accelAxis, &Y_accelAxis, &Z_accelAxis);
    accel8_getGyroAxis(&X_gyroAxis, &Y_gyroAxis, &Z_gyroAxis);
    Temperature = accel8_getTemperature();
    
    // LOGS DATA
    mikrobus_logWrite("________________ Accel 8 click _________________", _LOG_LINE);
    mikrobus_logWrite("|  Data   | X axis | Y axis | Z axis |  Range  |", _LOG_LINE);
    mikrobus_logWrite("|_________|________|________|________|_________|", _LOG_LINE);
    mikrobus_logWrite("|  Accel  |", _LOG_TEXT);
    IntToStr(X_accelAxis, demoText);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite("  |", _LOG_TEXT);
    IntToStr(Y_accelAxis, demoText);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite("  |", _LOG_TEXT);
    IntToStr(Z_accelAxis, demoText);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite("  |", _LOG_TEXT);
    IntToStr(accelRange, demoText);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite(" g |", _LOG_LINE);
    mikrobus_logWrite("|_________|________|________|________|_________|", _LOG_LINE);
    mikrobus_logWrite("|  Gyro   |", _LOG_TEXT);
    IntToStr(X_gyroAxis, demoText);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite("  |", _LOG_TEXT);
    IntToStr(Y_gyroAxis, demoText);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite("  |", _LOG_TEXT);
    IntToStr(Z_gyroAxis, demoText);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite("  |", _LOG_TEXT);
    IntToStr(gyroRange, demoText);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite("dps|", _LOG_LINE);
    mikrobus_logWrite("|_________|________|________|________|_________|", _LOG_LINE);
    FloatToStr(Temperature, demoText);
    demoText[ 5 ] = 0;
    mikrobus_logWrite("|  Temp   |", _LOG_TEXT);
    mikrobus_logWrite(demoText, _LOG_TEXT);
    mikrobus_logWrite("  C         |", _LOG_LINE);
    mikrobus_logWrite("|_________|_________________|", _LOG_LINE);

    mikrobus_logWrite("  ", _LOG_LINE);
    Delay_ms( 2000 );
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}