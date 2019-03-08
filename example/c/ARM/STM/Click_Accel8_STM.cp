#line 1 "D:/Clicks_git/A/Accel_8_Click/SW/example/c/ARM/STM/Click_Accel8_STM.c"
#line 1 "d:/clicks_git/a/accel_8_click/sw/example/c/arm/stm/click_accel8_types.h"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for arm/include/stdint.h"





typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int int32_t;
typedef signed long long int64_t;


typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
typedef unsigned long long uint64_t;


typedef signed char int_least8_t;
typedef signed int int_least16_t;
typedef signed long int int_least32_t;
typedef signed long long int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
typedef unsigned long long uint_least64_t;



typedef signed long int int_fast8_t;
typedef signed long int int_fast16_t;
typedef signed long int int_fast32_t;
typedef signed long long int_fast64_t;


typedef unsigned long int uint_fast8_t;
typedef unsigned long int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
typedef unsigned long long uint_fast64_t;


typedef signed long int intptr_t;
typedef unsigned long int uintptr_t;


typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;
#line 1 "d:/clicks_git/a/accel_8_click/sw/example/c/arm/stm/click_accel8_config.h"
#line 1 "d:/clicks_git/a/accel_8_click/sw/example/c/arm/stm/click_accel8_types.h"
#line 19 "d:/clicks_git/a/accel_8_click/sw/example/c/arm/stm/click_accel8_config.h"
const uint32_t _ACCEL8_I2C_CFG[ 1 ] =
{
 100000
};
#line 1 "d:/clicks_git/a/accel_8_click/sw/library/__accel8_driver.h"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for arm/include/stdint.h"
#line 58 "d:/clicks_git/a/accel_8_click/sw/library/__accel8_driver.h"
extern const uint8_t _ACCEL8_REG_SELF_TEST_X ;
extern const uint8_t _ACCEL8_REG_SELF_TEST_Y ;
extern const uint8_t _ACCEL8_REG_SELF_TEST_Z ;
extern const uint8_t _ACCEL8_REG_SELF_TEST_A ;
extern const uint8_t _ACCEL8_REG_SMPLRT_DIV ;
extern const uint8_t _ACCEL8_REG_CONFIG ;
extern const uint8_t _ACCEL8_REG_GYRO_CONFIG ;
extern const uint8_t _ACCEL8_REG_ACCEL_CONFIG ;
extern const uint8_t _ACCEL8_REG_FIFO_EN ;
extern const uint8_t _ACCEL8_REG_I2C_MST_CTRL ;
extern const uint8_t _ACCEL8_REG_I2C_SLV0_ADDR ;
extern const uint8_t _ACCEL8_REG_I2C_SLV0_REG ;
extern const uint8_t _ACCEL8_REG_I2C_SLV0_CTRL ;
extern const uint8_t _ACCEL8_REG_I2C_SLV1_ADDR ;
extern const uint8_t _ACCEL8_REG_I2C_SLV1_REG ;
extern const uint8_t _ACCEL8_REG_I2C_SLV1_CTRL ;
extern const uint8_t _ACCEL8_REG_I2C_SLV2_ADDR ;
extern const uint8_t _ACCEL8_REG_I2C_SLV2_REG ;
extern const uint8_t _ACCEL8_REG_I2C_SLV2_CTRL ;
extern const uint8_t _ACCEL8_REG_I2C_SLV3_ADDR ;
extern const uint8_t _ACCEL8_REG_I2C_SLV3_REG ;
extern const uint8_t _ACCEL8_REG_I2C_SLV3_CTRL ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_ADDR ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_REG ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_DO ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_CTRL ;
extern const uint8_t _ACCEL8_REG_I2C_SLV4_DI ;
extern const uint8_t _ACCEL8_REG_I2C_MST_STATUS ;
extern const uint8_t _ACCEL8_REG_INT_PIN_CFG ;
extern const uint8_t _ACCEL8_REG_INT_ENABLE ;
extern const uint8_t _ACCEL8_REG_INT_STATUS ;
extern const uint8_t _ACCEL8_REG_ACCEL_XOUT_H ;
extern const uint8_t _ACCEL8_REG_ACCEL_XOUT_L ;
extern const uint8_t _ACCEL8_REG_ACCEL_YOUT_H ;
extern const uint8_t _ACCEL8_REG_ACCEL_YOUT_L ;
extern const uint8_t _ACCEL8_REG_ACCEL_ZOUT_H ;
extern const uint8_t _ACCEL8_REG_ACCEL_ZOUT_L ;
extern const uint8_t _ACCEL8_REG_TEMP_OUT_H ;
extern const uint8_t _ACCEL8_REG_TEMP_OUT_L ;
extern const uint8_t _ACCEL8_REG_GYRO_XOUT_H ;
extern const uint8_t _ACCEL8_REG_GYRO_XOUT_L ;
extern const uint8_t _ACCEL8_REG_GYRO_YOUT_H ;
extern const uint8_t _ACCEL8_REG_GYRO_YOUT_L ;
extern const uint8_t _ACCEL8_REG_GYRO_ZOUT_H ;
extern const uint8_t _ACCEL8_REG_GYRO_ZOUT_L ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_00 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_01 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_02 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_03 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_04 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_05 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_06 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_07 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_08 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_09 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_10 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_11 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_12 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_13 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_14 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_15 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_16 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_17 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_18 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_19 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_20 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_21 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_22 ;
extern const uint8_t _ACCEL8_REG_EXT_SENS_DATA_23 ;
extern const uint8_t _ACCEL8_REG_I2C_SLV0_DO ;
extern const uint8_t _ACCEL8_REG_I2C_SLV1_DO ;
extern const uint8_t _ACCEL8_REG_I2C_SLV2_DO ;
extern const uint8_t _ACCEL8_REG_I2C_SLV3_DO ;
extern const uint8_t _ACCEL8_REG_I2C_MST_DELAY_CTRL ;
extern const uint8_t _ACCEL8_REG_SIGNAL_PATH_RESET ;
extern const uint8_t _ACCEL8_REG_MOT_DETECT_CTRL ;
extern const uint8_t _ACCEL8_REG_USER_CTRL ;
extern const uint8_t _ACCEL8_REG_PWR_MGMT_1 ;
extern const uint8_t _ACCEL8_REG_PWR_MGMT_2 ;
extern const uint8_t _ACCEL8_REG_FIFO_COUNTH ;
extern const uint8_t _ACCEL8_REG_FIFO_COUNTL ;
extern const uint8_t _ACCEL8_REG_FIFO_R_W ;
extern const uint8_t _ACCEL8_REG_WHO_AM_I ;


extern const uint8_t _ACCEL8_CFG_EXT_SYNC_INPUT_DISABLED ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_TEMP_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_X_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_Y_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_GYRO_Z_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_X_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_Y_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_EXT_SYNC_ACCEL_Z_OUTPUT ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A260_G256 ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A184_G188 ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A94_G98 ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A44_G42 ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A21_G20 ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A10_G10 ;
extern const uint8_t _ACCEL8_CFG_DLPF_CFG_BW_A5_G5 ;


extern const uint8_t _ACCEL8_GYRO_CFG_X_SELF_TEST ;
extern const uint8_t _ACCEL8_GYRO_CFG_Y_SELF_TEST ;
extern const uint8_t _ACCEL8_GYRO_CFG_Z_SELF_TEST ;
extern const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_250dbs ;
extern const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_500dbs ;
extern const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_1000dbs ;
extern const uint8_t _ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_2000dbs ;


extern const uint8_t _ACCEL8_ACCEL_CFG_X_SELF_TEST ;
extern const uint8_t _ACCEL8_ACCEL_CFG_Y_SELF_TEST ;
extern const uint8_t _ACCEL8_ACCEL_CFG_Z_SELF_TEST ;
extern const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_2g ;
extern const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_4g ;
extern const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_8g ;
extern const uint8_t _ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_16g ;


extern const uint8_t _ACCEL8_FIFO_ENABLE_TEMP ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_X_AXIS_GYRO ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_Y_AXIS_GYRO ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_Z_AXIS_GYRO ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_ACCEL ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV2 ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV1 ;
extern const uint8_t _ACCEL8_FIFO_ENABLE_EXT_SENS_DATA_SLV0 ;


extern const uint8_t _ACCEL8_I2C_MST_CTRL_MUL_MST_ENABLE ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_WAIT_FOR_ES ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_EXT_SENS_DATA_SLV3 ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_RESTART_BETWEEN_READS ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_STOP_AND_START ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_348kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_333kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_320kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_308kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_296kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_286kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_276kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_367kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_358kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_500kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_471kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_444kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_421kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_400kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_381kHz ;
extern const uint8_t _ACCEL8_I2C_MST_CTRL_I2C_CLOCK_364kHz ;


extern const uint8_t _ACCEL8_INTC_INT_PIN_IS_ACTIVE_HIGH ;
extern const uint8_t _ACCEL8_INTC_INT_PIN_IS_ACTIVE_LOW ;
extern const uint8_t _ACCEL8_INTC_INT_PIN_IS_CONFIGURED_AS_PUSH_PULL ;
extern const uint8_t _ACCEL8_INTC_INT_PIN_IS_CONFIGURED_AS_OPEN_DRAIN ;
extern const uint8_t _ACCEL8_INTC_LATCH_INT_ENABLE ;
extern const uint8_t _ACCEL8_INTC_INT_READ_CLEAR ;
extern const uint8_t _ACCEL8_INTC_FSYNC_INT_LEVEL_ACTIVE_HIGH ;
extern const uint8_t _ACCEL8_INTC_FSYNC_INT_LEVEL_ACTIVE_LOW ;
extern const uint8_t _ACCEL8_INTC_FSYNC_INT_ENABLE ;
extern const uint8_t _ACCEL8_INTC_I2C_BYPASS_ENABLE ;
extern const uint8_t _ACCEL8_INTC_I2C_BYPASS_DISABLE ;


extern const uint8_t _ACCEL8_INTE_FIFO_OFLOW_ENABLE ;
extern const uint8_t _ACCEL8_INTE_I2C_MST_INT_ENABLE ;
extern const uint8_t _ACCEL8_INTE_DATA_RDY_ENABLE ;


extern const uint8_t _ACCEL8_INTS_FIFO_OFLOW ;
extern const uint8_t _ACCEL8_INTS_I2C_MST_INT ;
extern const uint8_t _ACCEL8_INTS_DATA_RDY ;


extern const uint8_t _ACCEL8_ACCEL_X_AXIS_DATA ;
extern const uint8_t _ACCEL8_ACCEL_Y_AXIS_DATA ;
extern const uint8_t _ACCEL8_ACCEL_Z_AXIS_DATA ;
extern const uint8_t _ACCEL8_TEMP_DATA ;
extern const uint8_t _ACCEL8_GYRO_X_AXIS_DATA ;
extern const uint8_t _ACCEL8_GYRO_Y_AXIS_DATA ;
extern const uint8_t _ACCEL8_GYRO_Z_AXIS_DATA ;


extern const uint8_t _ACCEL8_GYRO_RESET ;
extern const uint8_t _ACCEL8_ACCEL_RESET;
extern const uint8_t _ACCEL8_TEMP_RESET ;


extern const uint8_t _ACCEL8_UC_FIFO_ENABLE ;
extern const uint8_t _ACCEL8_UC_I2C_MASTER_MODE ;
extern const uint8_t _ACCEL8_UC_FIFO_RESET ;
extern const uint8_t _ACCEL8_UC_I2C_MASTER_RESET ;
extern const uint8_t _ACCEL8_UC_SIG_COND_RESET ;


extern const uint8_t _ACCEL8_PM1_DEVICE_RESET ;
extern const uint8_t _ACCEL8_PM1_GO_TO_SLEEP ;
extern const uint8_t _ACCEL8_PM1_CYCLE ;
extern const uint8_t _ACCEL8_PM1_TEMP_DISABLE ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_INTERNAL_8MHZ_OSCILLATOR ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_X_AXIS_GYROSCOPE ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_Y_AXIS_GYROSCOPE ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_Z_AXIS_GYROSCOPE ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_EXTERNAL_32_768kHz ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_PLL_WITH_EXTERNAL_19_2kHz ;
extern const uint8_t _ACCEL8_PM1_CLKSEL_STOPS_THE_CLOCK ;


extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_1_25Hz ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_5Hz ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_20Hz ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_WAKE_UP_FREQ_40Hz ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_X_AXIS_ACCEL_STANDBY_MODE ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Y_AXIS_ACCEL_STANDBY_MODE ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Z_AXIS_ACCEL_STANDBY_MODE ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_X_AXIS_GYRO_STANDBY_MODE ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Y_AXIS_GYRO_STANDBY_MODE ;
extern const uint8_t _ACCEL8_PM2_LP_WAKE_CTRL_Z_AXIS_GYRO_STANDBY_MODE ;


extern const uint8_t _ACCEL8_DEVICE_SLAVE_ADDRESS_ADD ;
extern const uint8_t _ACCEL8_DEVICE_SLAVE_ADDRESS_SEL ;
#line 300 "d:/clicks_git/a/accel_8_click/sw/library/__accel8_driver.h"
void accel8_i2cDriverInit( const uint8_t*  gpioObj,  const uint8_t*  i2cObj, uint8_t slave);
#line 307 "d:/clicks_git/a/accel_8_click/sw/library/__accel8_driver.h"
void accel8_gpioDriverInit( const uint8_t*  gpioObj);



void accel8_writeByte(uint8_t reg, uint8_t _data);

uint8_t accel8_readByte(uint8_t reg);

uint16_t accel8_readData(uint8_t reg);

float accel8_getTemperature();

void accel8_getAccelAxis(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis);

void accel8_getGyroAxis(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis);

uint8_t accel8_accelConfig(uint8_t cfg);

uint16_t accel8_gyroConfig(uint8_t cfg);
#line 31 "D:/Clicks_git/A/Accel_8_Click/SW/example/c/ARM/STM/Click_Accel8_STM.c"
float Temperature;
int16_t X_gyroAxis;
int16_t Y_gyroAxis;
int16_t Z_gyroAxis;
int16_t X_accelAxis;
int16_t Y_accelAxis;
int16_t Z_accelAxis;

uint16_t gyroRange;
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
 accel8_i2cDriverInit( ( const uint8_t* )&_MIKROBUS1_GPIO, ( const uint8_t* )&_MIKROBUS1_I2C, _ACCEL8_DEVICE_SLAVE_ADDRESS_ADD );

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
