![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Accel8 Click

- **CIC Prefix**  : ACCEL8
- **Author**      : Katarina Perendic
- **Verison**     : 1.0.0
- **Date**        : okt 2018.



### Software Support

We provide a library for the Accel8 Click on our [LibStock](https://libstock.mikroe.com/projects/view/2584/accel-8-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library initializes and defines the I2C bus driver and drivers that offer a choice for writing data in register and reads data form register.
The library includes function for read Accel X/Y/Z axis data, Gyro X/Y/Z axis data and device Temperature data.
The user also has the function for configuration Accel and Gyro and function for read interrupt state.

Key functions :

- ``` void accel8_getAccelAxis(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis) ``` - Functions for read Accel axis data
- ``` void accel8_getGyroAxis(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis) ``` - Functions for read Gyro axis data
- ``` float accel8_getTemperature() ``` - Functions for read Temperature data in C

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes I2C module and sets INT pin as INPUT
- Application Initialization - Initialization driver init, reset chip and start configuration chip for measurement
- Application Task - (code snippet) - Reads Accel X/Y/Z axis, Gyro X/Y/Z axis and device Temperature. 
                                      All data logs on the USBUART every 2 sec.

```.c
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
````

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2584/accel-8-click) page.

Other mikroE Libraries used in the example:

- I2C

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
