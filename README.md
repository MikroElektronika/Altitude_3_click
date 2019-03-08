![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Altitude_3 Click

- **CIC Prefix**  : ALTITUDE3
- **Author**      : Nemanja Medakovic
- **Verison**     : 1.0.0
- **Date**        : Dec 2018.

---


### Software Support

We provide a library for the Altitude_3 Click on our [LibStock](https://libstock.mikroe.com/projects/view/2695/altitude-3-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library performs a control of the Altitude 3 Click board.
This library can read a calibration data from OTP sensor, also a temperature and pressure results of AD conversion.
The all converted results can be calculated to standard units, and depending on the temperature and pressure data 
the library can calculate altitude.
For more details check documentation.

Key functions :

- ``` T_ALTITUDE3_RETVAL altitude3_measurementMode( uint16_t modeCmd ) ``` - Function sends a command which selects a measurement mode and data reading order.
- ``` void altitude3_init( T_altitude3_param *s ) ``` - Function sends a command to read calibration data from OTP sensor and other initialization data, 
  which is necessary for calculations.
- ``` T_ALTITUDE3_RETVAL altitude3_getData( uint8_t read_order, int8_t *temperature, uint16_t *pressure, int16_t *altitude ) ``` - Function performs a calibration data reading, 
  only once, and then reads a temperature and pressure data and calculates these values to standard units. Also calculates the altitude depending on the temperature and pressure data.

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes peripherals and pins.
- Application Initialization - Initializes I2C interface and performs a SW Reset of the device.
- Application Task - (code snippet) - Selects the desired measurement mode and data reading order, and after that 
  calculates the temperature, pressure and altitude data to standard units and shows results to uart terminal.


```.c
void applicationTask()
{
    response = altitude3_measurementMode( _ALTITUDE3_NORMAL_T_FIRST );

    response = altitude3_getData( response, &_temperature, &_pressure, &_altitude );
    
    if (response == _ALTITUDE3_OK)
    {
        ShortToStr( _temperature, text );
        mikrobus_logWrite( "Temperature is : ", _LOG_TEXT );
        mikrobus_logWrite( text, _LOG_TEXT );
        logUnit();

        WordToStr( _pressure, text );
        mikrobus_logWrite( "Pressure is : ", _LOG_TEXT );
        mikrobus_logWrite( text, _LOG_TEXT );
        mikrobus_logWrite( " mbar[hPa]", _LOG_LINE );
    
        IntToStr( _altitude, text );
        mikrobus_logWrite( "Altitude is : ", _LOG_TEXT );
        mikrobus_logWrite( text, _LOG_TEXT );
        mikrobus_logWrite( " m", _LOG_LINE );
        mikrobus_logWrite( "", _LOG_LINE );

        Delay_ms( 400 );
    }
}
```

Additional Functions :

- logUnit - Shows Celsius degrees symbol to uart terminal.

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2695/altitude-3-click) page.

Other mikroE Libraries used in the example:

- Conversions
- I2C
- UART

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
