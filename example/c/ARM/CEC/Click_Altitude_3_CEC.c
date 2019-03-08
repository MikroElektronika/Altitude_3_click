/*
Example for Altitude_3 Click

    Date          : Dec 2018.
    Author        : Nemanja Medakovic

Test configuration CEC :
    
    MCU              : CEC1702
    Dev. Board       : Clicker 2 for CEC1702
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes peripherals and pins.
- Application Initialization - Initializes I2C interface and performs a SW Reset of the device.
- Application Task - (code snippet) - Selects the desired measurement mode and data reading order, and after that 
  calculates the temperature, pressure and altitude data to standard units and shows results to uart terminal.

Additional Functions :

- logUnit - Shows Celsius degrees symbol to uart terminal.

*/

#include "Click_Altitude_3_types.h"
#include "Click_Altitude_3_config.h"

int8_t _temperature;
uint16_t _pressure;
int16_t _altitude;
T_ALTITUDE3_RETVAL response;
char text[ 30 ];

void logUnit()
{
    text[ 0 ] = ' ';
    text[ 1 ] = 176;
    text[ 2 ] = 'C';
    text[ 3 ] = 0;
    
    mikrobus_logWrite( text, _LOG_LINE );
}

void systemInit()
{
    mikrobus_i2cInit( _MIKROBUS1, &_ALTITUDE3_I2C_CFG[0] );

    mikrobus_logInit( _MIKROBUS2, 9600 );
    mikrobus_logWrite( "*** Initializing... ***", _LOG_LINE );

    Delay_ms( 100 );
}

void applicationInit()
{
    altitude3_i2cDriverInit( (T_ALTITUDE3_P)&_MIKROBUS1_GPIO, (T_ALTITUDE3_P)&_MIKROBUS1_I2C, _ALTITUDE3_SLAVE_ADDR );
    Delay_ms( 500 );

    altitude3_softReset();
    Delay_ms( 500 );

    mikrobus_logWrite( "** Altitude 3 click is initialized **", _LOG_LINE );
    mikrobus_logWrite( "", _LOG_LINE );
}

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

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
        applicationTask();
    }
}
