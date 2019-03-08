_systemInit:
;Click_Accel8_STM.c,44 :: 		void systemInit()
SUB	SP, SP, #4
STR	LR, [SP, #0]
;Click_Accel8_STM.c,46 :: 		mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
MOVS	R2, #1
MOVS	R1, #7
MOVS	R0, #0
BL	_mikrobus_gpioInit+0
;Click_Accel8_STM.c,47 :: 		mikrobus_i2cInit( _MIKROBUS1, &_ACCEL8_I2C_CFG[0] );
MOVW	R0, #lo_addr(__ACCEL8_I2C_CFG+0)
MOVT	R0, #hi_addr(__ACCEL8_I2C_CFG+0)
MOV	R1, R0
MOVS	R0, #0
BL	_mikrobus_i2cInit+0
;Click_Accel8_STM.c,48 :: 		mikrobus_logInit( _LOG_USBUART_A, 115200 );
MOV	R1, #115200
MOVS	R0, #32
BL	_mikrobus_logInit+0
;Click_Accel8_STM.c,49 :: 		mikrobus_logWrite(" --- System Init ---", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr1_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr1_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,50 :: 		Delay_ms( 100 );
MOVW	R7, #20351
MOVT	R7, #18
NOP
NOP
L_systemInit0:
SUBS	R7, R7, #1
BNE	L_systemInit0
NOP
NOP
NOP
;Click_Accel8_STM.c,51 :: 		}
L_end_systemInit:
LDR	LR, [SP, #0]
ADD	SP, SP, #4
BX	LR
; end of _systemInit
_applicationInit:
;Click_Accel8_STM.c,53 :: 		void applicationInit()
SUB	SP, SP, #4
STR	LR, [SP, #0]
;Click_Accel8_STM.c,55 :: 		accel8_i2cDriverInit( (T_ACCEL8_P)&_MIKROBUS1_GPIO, (T_ACCEL8_P)&_MIKROBUS1_I2C, _ACCEL8_DEVICE_SLAVE_ADDRESS_ADD );
MOVS	R2, __ACCEL8_DEVICE_SLAVE_ADDRESS_ADD
MOVW	R1, #lo_addr(__MIKROBUS1_I2C+0)
MOVT	R1, #hi_addr(__MIKROBUS1_I2C+0)
MOVW	R0, #lo_addr(__MIKROBUS1_GPIO+0)
MOVT	R0, #hi_addr(__MIKROBUS1_GPIO+0)
BL	_accel8_i2cDriverInit+0
;Click_Accel8_STM.c,57 :: 		mikrobus_logWrite(" --- Device Reset --- ", _LOG_LINE );
MOVW	R0, #lo_addr(?lstr2_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr2_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,58 :: 		accel8_writeByte(_ACCEL8_REG_PWR_MGMT_1, _ACCEL8_PM1_DEVICE_RESET);
MOVS	R1, __ACCEL8_PM1_DEVICE_RESET
MOVS	R0, __ACCEL8_REG_PWR_MGMT_1
BL	_accel8_writeByte+0
;Click_Accel8_STM.c,59 :: 		accel8_writeByte(_ACCEL8_REG_SIGNAL_PATH_RESET, _ACCEL8_GYRO_RESET | _ACCEL8_ACCEL_RESET | _ACCEL8_TEMP_RESET );
MOVS	R0, __ACCEL8_GYRO_RESET
ORR	R0, R0, __ACCEL8_ACCEL_RESET
UXTB	R0, R0
ORR	R0, R0, __ACCEL8_TEMP_RESET
UXTB	R1, R0
MOVS	R0, __ACCEL8_REG_SIGNAL_PATH_RESET
BL	_accel8_writeByte+0
;Click_Accel8_STM.c,61 :: 		mikrobus_logWrite(" --- Device Configuration --- ", _LOG_LINE );
MOVW	R0, #lo_addr(?lstr3_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr3_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,62 :: 		accel8_writeByte(_ACCEL8_REG_PWR_MGMT_1, _ACCEL8_PM1_CLKSEL_INTERNAL_8MHZ_OSCILLATOR);
MOVS	R1, __ACCEL8_PM1_CLKSEL_INTERNAL_8MHZ_OSCILLATOR
MOVS	R0, __ACCEL8_REG_PWR_MGMT_1
BL	_accel8_writeByte+0
;Click_Accel8_STM.c,63 :: 		accel8_writeByte(_ACCEL8_REG_INT_ENABLE, _ACCEL8_INTE_DATA_RDY_ENABLE );
MOVS	R1, __ACCEL8_INTE_DATA_RDY_ENABLE
MOVS	R0, __ACCEL8_REG_INT_ENABLE
BL	_accel8_writeByte+0
;Click_Accel8_STM.c,65 :: 		gyroRange = accel8_gyroConfig(_ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_250dbs);
MOVS	R0, __ACCEL8_GYRO_CFG_FULL_SCALE_RANGE_250dbs
BL	_accel8_gyroConfig+0
MOVW	R1, #lo_addr(_gyroRange+0)
MOVT	R1, #hi_addr(_gyroRange+0)
STRH	R0, [R1, #0]
;Click_Accel8_STM.c,66 :: 		accelRange = accel8_accelConfig(_ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_2g);
MOVS	R0, __ACCEL8_ACCEL_CFG_FULL_SCALE_RANGE_2g
BL	_accel8_accelConfig+0
MOVW	R1, #lo_addr(_accelRange+0)
MOVT	R1, #hi_addr(_accelRange+0)
STRB	R0, [R1, #0]
;Click_Accel8_STM.c,69 :: 		_ACCEL8_FIFO_ENABLE_X_AXIS_GYRO |
MOVS	R0, __ACCEL8_FIFO_ENABLE_TEMP
ORR	R0, R0, __ACCEL8_FIFO_ENABLE_X_AXIS_GYRO
UXTB	R0, R0
;Click_Accel8_STM.c,70 :: 		_ACCEL8_FIFO_ENABLE_Y_AXIS_GYRO |
ORR	R0, R0, __ACCEL8_FIFO_ENABLE_Y_AXIS_GYRO
UXTB	R0, R0
;Click_Accel8_STM.c,71 :: 		_ACCEL8_FIFO_ENABLE_Z_AXIS_GYRO |
ORR	R0, R0, __ACCEL8_FIFO_ENABLE_Z_AXIS_GYRO
UXTB	R0, R0
;Click_Accel8_STM.c,72 :: 		_ACCEL8_FIFO_ENABLE_ACCEL );
ORR	R0, R0, __ACCEL8_FIFO_ENABLE_ACCEL
UXTB	R1, R0
;Click_Accel8_STM.c,68 :: 		accel8_writeByte(_ACCEL8_REG_FIFO_EN, _ACCEL8_FIFO_ENABLE_TEMP |
MOVS	R0, __ACCEL8_REG_FIFO_EN
;Click_Accel8_STM.c,72 :: 		_ACCEL8_FIFO_ENABLE_ACCEL );
BL	_accel8_writeByte+0
;Click_Accel8_STM.c,73 :: 		Delay_ms( 1000 );
MOVW	R7, #6911
MOVT	R7, #183
NOP
NOP
L_applicationInit2:
SUBS	R7, R7, #1
BNE	L_applicationInit2
NOP
NOP
NOP
;Click_Accel8_STM.c,74 :: 		mikrobus_logWrite(" --- Start Measurement --- ", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr4_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr4_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,75 :: 		}
L_end_applicationInit:
LDR	LR, [SP, #0]
ADD	SP, SP, #4
BX	LR
; end of _applicationInit
_applicationTask:
;Click_Accel8_STM.c,77 :: 		void applicationTask()
SUB	SP, SP, #4
STR	LR, [SP, #0]
;Click_Accel8_STM.c,79 :: 		accel8_getAccelAxis(&X_accelAxis, &Y_accelAxis, &Z_accelAxis);
MOVW	R2, #lo_addr(_Z_accelAxis+0)
MOVT	R2, #hi_addr(_Z_accelAxis+0)
MOVW	R1, #lo_addr(_Y_accelAxis+0)
MOVT	R1, #hi_addr(_Y_accelAxis+0)
MOVW	R0, #lo_addr(_X_accelAxis+0)
MOVT	R0, #hi_addr(_X_accelAxis+0)
BL	_accel8_getAccelAxis+0
;Click_Accel8_STM.c,80 :: 		accel8_getGyroAxis(&X_gyroAxis, &Y_gyroAxis, &Z_gyroAxis);
MOVW	R2, #lo_addr(_Z_gyroAxis+0)
MOVT	R2, #hi_addr(_Z_gyroAxis+0)
MOVW	R1, #lo_addr(_Y_gyroAxis+0)
MOVT	R1, #hi_addr(_Y_gyroAxis+0)
MOVW	R0, #lo_addr(_X_gyroAxis+0)
MOVT	R0, #hi_addr(_X_gyroAxis+0)
BL	_accel8_getGyroAxis+0
;Click_Accel8_STM.c,81 :: 		Temperature = accel8_getTemperature();
BL	_accel8_getTemperature+0
MOVW	R1, #lo_addr(_Temperature+0)
MOVT	R1, #hi_addr(_Temperature+0)
STR	R0, [R1, #0]
;Click_Accel8_STM.c,84 :: 		mikrobus_logWrite("________________ Accel 8 click _________________", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr5_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr5_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,85 :: 		mikrobus_logWrite("|  Data   | X axis | Y axis | Z axis |  Range  |", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr6_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr6_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,86 :: 		mikrobus_logWrite("|_________|________|________|________|_________|", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr7_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr7_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,87 :: 		mikrobus_logWrite("|  Accel  |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr8_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr8_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,88 :: 		IntToStr(X_accelAxis, demoText);
MOVW	R0, #lo_addr(_X_accelAxis+0)
MOVT	R0, #hi_addr(_X_accelAxis+0)
LDRSH	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_IntToStr+0
;Click_Accel8_STM.c,89 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,90 :: 		mikrobus_logWrite("  |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr9_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr9_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,91 :: 		IntToStr(Y_accelAxis, demoText);
MOVW	R0, #lo_addr(_Y_accelAxis+0)
MOVT	R0, #hi_addr(_Y_accelAxis+0)
LDRSH	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_IntToStr+0
;Click_Accel8_STM.c,92 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,93 :: 		mikrobus_logWrite("  |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr10_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr10_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,94 :: 		IntToStr(Z_accelAxis, demoText);
MOVW	R0, #lo_addr(_Z_accelAxis+0)
MOVT	R0, #hi_addr(_Z_accelAxis+0)
LDRSH	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_IntToStr+0
;Click_Accel8_STM.c,95 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,96 :: 		mikrobus_logWrite("  |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr11_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr11_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,97 :: 		IntToStr(accelRange, demoText);
MOVW	R0, #lo_addr(_accelRange+0)
MOVT	R0, #hi_addr(_accelRange+0)
LDRB	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_IntToStr+0
;Click_Accel8_STM.c,98 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,99 :: 		mikrobus_logWrite(" g |", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr12_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr12_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,100 :: 		mikrobus_logWrite("|_________|________|________|________|_________|", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr13_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr13_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,101 :: 		mikrobus_logWrite("|  Gyro   |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr14_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr14_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,102 :: 		IntToStr(X_gyroAxis, demoText);
MOVW	R0, #lo_addr(_X_gyroAxis+0)
MOVT	R0, #hi_addr(_X_gyroAxis+0)
LDRSH	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_IntToStr+0
;Click_Accel8_STM.c,103 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,104 :: 		mikrobus_logWrite("  |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr15_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr15_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,105 :: 		IntToStr(Y_gyroAxis, demoText);
MOVW	R0, #lo_addr(_Y_gyroAxis+0)
MOVT	R0, #hi_addr(_Y_gyroAxis+0)
LDRSH	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_IntToStr+0
;Click_Accel8_STM.c,106 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,107 :: 		mikrobus_logWrite("  |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr16_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr16_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,108 :: 		IntToStr(Z_gyroAxis, demoText);
MOVW	R0, #lo_addr(_Z_gyroAxis+0)
MOVT	R0, #hi_addr(_Z_gyroAxis+0)
LDRSH	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_IntToStr+0
;Click_Accel8_STM.c,109 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,110 :: 		mikrobus_logWrite("  |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr17_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr17_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,111 :: 		IntToStr(gyroRange, demoText);
MOVW	R0, #lo_addr(_gyroRange+0)
MOVT	R0, #hi_addr(_gyroRange+0)
LDRH	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_IntToStr+0
;Click_Accel8_STM.c,112 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,113 :: 		mikrobus_logWrite("dps|", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr18_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr18_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,114 :: 		mikrobus_logWrite("|_________|________|________|________|_________|", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr19_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr19_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,115 :: 		FloatToStr(Temperature, demoText);
MOVW	R0, #lo_addr(_Temperature+0)
MOVT	R0, #hi_addr(_Temperature+0)
LDR	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_FloatToStr+0
;Click_Accel8_STM.c,116 :: 		demoText[ 5 ] = 0;
MOVS	R1, #0
MOVW	R0, #lo_addr(_demoText+5)
MOVT	R0, #hi_addr(_demoText+5)
STRB	R1, [R0, #0]
;Click_Accel8_STM.c,117 :: 		mikrobus_logWrite("|  Temp   |", _LOG_TEXT);
MOVW	R0, #lo_addr(?lstr20_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr20_Click_Accel8_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,118 :: 		mikrobus_logWrite(demoText, _LOG_TEXT);
MOVS	R1, #1
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,119 :: 		mikrobus_logWrite("  C         |", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr21_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr21_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,120 :: 		mikrobus_logWrite("|_________|_________________|", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr22_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr22_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,122 :: 		mikrobus_logWrite("  ", _LOG_LINE);
MOVW	R0, #lo_addr(?lstr23_Click_Accel8_STM+0)
MOVT	R0, #hi_addr(?lstr23_Click_Accel8_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Accel8_STM.c,123 :: 		Delay_ms( 2000 );
MOVW	R7, #13823
MOVT	R7, #366
NOP
NOP
L_applicationTask4:
SUBS	R7, R7, #1
BNE	L_applicationTask4
NOP
NOP
NOP
;Click_Accel8_STM.c,124 :: 		}
L_end_applicationTask:
LDR	LR, [SP, #0]
ADD	SP, SP, #4
BX	LR
; end of _applicationTask
_main:
;Click_Accel8_STM.c,126 :: 		void main()
;Click_Accel8_STM.c,128 :: 		systemInit();
BL	_systemInit+0
;Click_Accel8_STM.c,129 :: 		applicationInit();
BL	_applicationInit+0
;Click_Accel8_STM.c,131 :: 		while (1)
L_main6:
;Click_Accel8_STM.c,133 :: 		applicationTask();
BL	_applicationTask+0
;Click_Accel8_STM.c,134 :: 		}
IT	AL
BAL	L_main6
;Click_Accel8_STM.c,135 :: 		}
L_end_main:
L__main_end_loop:
B	L__main_end_loop
; end of _main
