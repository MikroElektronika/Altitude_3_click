/*
    __altitude3_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__altitude3_driver.h"
#include "__altitude3_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __ALTITUDE3_DRV_I2C__
static uint8_t _slaveAddress;
#endif

#define DBL_MAX       3.40282347e+38
#define DBL_MAX_PIC   6.80564713e+38
#define EXCESS        126
#define MAX_EXPONENT  255

#define _FRNDINT(x)       ((double)(long)(x))
#define DBL_MANT_DIG      23
#define DBL_MANT_DIG_PIC  24
#define CHAR_BIT          8

#define EXP_MAX  89.416
#define EXP_MIN  -87.33655

static union both
{
    struct flt
    {
        unsigned char   mant[2];
        unsigned        hmant:7;
        unsigned        exp:8;
        unsigned        sign:1;

    } flt;
    
    double fl;
};

static uint8_t updateCheck;

const uint8_t _ALTITUDE3_SLAVE_ADDR                       = 0x63;

const uint16_t _ALTITUDE3_LOW_POWER_T_FIRST               = 0x609C;
const uint16_t _ALTITUDE3_LOW_POWER_P_FIRST               = 0x401A;
const uint16_t _ALTITUDE3_NORMAL_T_FIRST                  = 0x6825;
const uint16_t _ALTITUDE3_NORMAL_P_FIRST                  = 0x48A3;
const uint16_t _ALTITUDE3_LOW_NOISE_T_FIRST               = 0x70DF;
const uint16_t _ALTITUDE3_LOW_NOISE_P_FIRST               = 0x5059;
const uint16_t _ALTITUDE3_ULTRA_LOW_NOISE_T_FIRST         = 0x7866;
const uint16_t _ALTITUDE3_ULTRA_LOW_NOISE_P_FIRST         = 0x58E0;

const uint8_t _ALTITUDE3_OK                               = 0x00;
const uint8_t _ALTITUDE3_T_FIRST_ORDER                    = 0x01;
const uint8_t _ALTITUDE3_P_FIRST_ORDER                    = 0x02;
const uint8_t _ALTITUDE3_INITIALIZED                      = 0x03;
const uint8_t _ALTITUDE3_ERROR                            = 0xFF;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */

/**
 * @brief OTP Calibration Read function
 *
 * Function sends a command to read a 16bit calibration data from OTP sensor.
 */
static void _read_otp( uint16_t *outData );

/**
 * @brief Calculate Conversion Constants function
 *
 * Function performs the all necessary calculations to get necessary conversion constants.
 */
static void calc_conv_const( uint32_t *p_Pa, float *p_LUT, float *outData );

/* ---------- The math functions ---------- */

static double _pow( double x, double y );

static double _log( double x );

static double _exp( double x );

static double _eval_poly( double x, const double code * d, int n ) ;

static double _ldexp( double value, int newexp );

static double _frexp( double value, int * eptr );

static double _floor( double x );

/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */

static void _read_otp( uint16_t *outData )
{
    uint8_t tempData[ 5 ];
    uint8_t cnt;
    
    tempData[ 0 ] = 0xC5;
    tempData[ 1 ] = 0x95;
    tempData[ 2 ] = 0x00;
    tempData[ 3 ] = 0x66;
    tempData[ 4 ] = 0x9C;
    
    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, 5, END_MODE_STOP );
    Delay_10ms();
    
    for (cnt = 0; cnt < 4; cnt++)
    {
        tempData[ 0 ] = 0xC7;
        tempData[ 1 ] = 0xF7;
        
        hal_i2cStart();
        hal_i2cWrite( _slaveAddress, tempData, 2, END_MODE_RESTART );
        hal_i2cRead( _slaveAddress, tempData, 3, END_MODE_STOP );
        Delay_1ms();
        
        outData[ cnt ] = tempData[ 0 ];
        outData[ cnt ] <<= 8;
        outData[ cnt ] |= tempData[ 1 ];
    }
}

static void calc_conv_const( uint32_t *p_Pa, float *p_LUT, float *outData )
{
    float tmp;
    float c;
    float b;
    float a;

    c = (float)(p_LUT[0] * p_LUT[1] * (int32_t)(p_Pa[0] - p_Pa[1]));
    c += (float)(p_LUT[2] * p_LUT[0] * (int32_t)(p_Pa[2] - p_Pa[0]));
    c += (float)(p_LUT[1] * p_LUT[2] * (int32_t)(p_Pa[1] - p_Pa[2]));
    tmp = (float)(p_LUT[2] * (int32_t)(p_Pa[0] - p_Pa[1]));
    tmp += (float)(p_LUT[1] * (int32_t)(p_Pa[2] - p_Pa[0]));
    tmp += (float)(p_LUT[0] * (int32_t)(p_Pa[1] - p_Pa[2]));
    c /= tmp;

    a = (float)(p_Pa[0] * p_LUT[0]);
    a -= (float)(p_Pa[1] * p_LUT[1]);
    a -= (float)((int32_t)(p_Pa[1] - p_Pa[0]) * c);
    a /= (float)(p_LUT[0] - p_LUT[1]);
    
    b = (float)(p_Pa[0] - a);
    b *= (float)(p_LUT[0] + c);
    
    outData[ 2 ] = c;
    outData[ 1 ] = b;
    outData[ 0 ] = a;
}

static double _pow( double x, double y )
{
    unsigned char sign = 0;
    long yi;

    if (y == 0.0)
    {
        return 1.0;
    }
    if (x == 0.0)
    {
        return 0.0;
    }

    if (x < 0.0)
    {
        yi = (long)y;

        if ((double)yi != y)
        {
            return 0.0;
        }
        sign = yi & 1;
        x = -x;
    }

    x = _log(x);
    x = x * y;
    x = _exp(x);

    if (sign)
    {
        return -x;
    }
    return x;
}

static double _log( double x )
{
    int exp;
    static const double coeff[9] = {
     0.0000000001,      // a0 //
     0.9999964239,      // a1 //
    -0.4998741238,      // a2 //
     0.3317990258,      // a3 //
    -0.2407338084,      // a4 //
     0.1676540711,      // a5 //
    -0.0953293897,      // a6 //
     0.0360884937,      // a7 //
    -0.0064535442,      // a8 //
    };

    if (x <= 0.0)
    {
        return 0.0;
    }

    x = _frexp( x, &exp ) * 2.0 - 1.0;
    exp--;
    x = _eval_poly( x, coeff, sizeof coeff/sizeof coeff[0] - 1 );

    return x + 0.69314718055995 * exp;
}

static double _exp( double x )
{
    int exp;
    char sign;

    const static double coeff[10] = {
    1.0000000000e+00,
    6.9314718056e-01,
    2.4022650695e-01,
    5.5504108945e-02,
    9.6181261779e-03,
    1.3333710529e-03,
    1.5399104432e-04,
    1.5327675257e-05,
    1.2485143336e-06,
    1.3908092221e-07,
    };

    if (x == 0.0)
    {
        return 1.0;
    }
    if (x > EXP_MAX)
    {
#ifndef __MIKROC_PRO_FOR_PIC__
        return DBL_MAX;
#endif
#ifdef __MIKROC_PRO_FOR_PIC__
        return DBL_MAX_PIC;
#endif
    }
    if (x < EXP_MIN)
    {
        return 0.0;
    }

    sign = x < 0.0;
    if (sign)
    {
        x = -x;
    }

    x *= 1.4426950409;
    exp = (int)_floor( x );
    x -= (double)exp;
    x = _ldexp( _eval_poly( x, coeff, sizeof coeff/sizeof coeff[0] - 1 ), exp );

    if (sign)
    {
#ifndef __MIKROC_PRO_FOR_PIC__
        if (x == DBL_MAX)
        {
            return 0.0;
        }
#endif
#ifdef __MIKROC_PRO_FOR_PIC__
        if (x == DBL_MAX_PIC)
        {
            return 0.0;
        }
#endif
        return 1.0 / x;
    }

    return x;
}

static double _eval_poly( double x, const double code *d, int n )
{
    double res;

    res = d[n];
    
    while (n)
    {
        res = x * res + d[--n];
    }

    return res;
}

static double _ldexp( double value, int newexp )
{
#ifndef __MIKROC_PRO_FOR_PIC__
    union both uv;

    uv.fl = value;
    newexp += uv.flt.exp;
    
    if (newexp < 0)
    {
        return 0.0;
    }
    else if (newexp > MAX_EXPONENT)
    {
        if (value < 0.0)
        {
            return -DBL_MAX;
        }
        else
        {
            return DBL_MAX;
        }
    }
    else
    {
        uv.flt.exp = newexp;
    }

    return uv.fl;
#endif
#ifdef __MIKROC_PRO_FOR_PIC__
    char *pom;

    pom = &value;
    newexp += pom[3];
    
    if (newexp < 0)
    {
        return 0.0;
    }
    else if (newexp > MAX_EXPONENT)
    {
        if (value < 0.0)
        {
            return -DBL_MAX_PIC;
        }
        else
        {
            return DBL_MAX_PIC;
        }
    }
    else
    {
        pom[3] = newexp;
    }

    return value;
#endif
}

static double _frexp( double value, int *eptr )
{
#ifndef __MIKROC_PRO_FOR_PIC__
    union both uv;
    volatile int bb;

    uv.fl = value;
    bb = uv.flt.exp - EXCESS;
    *eptr = bb;
    uv.flt.exp = EXCESS;

    return uv.fl;
#endif
#ifdef __MIKROC_PRO_FOR_PIC__
    char *pom;

    pom = &value;
    *eptr = pom[3] - EXCESS;
    pom[3] = EXCESS;

    return value;
#endif
}

static double _floor( double x )
{
    double i;
    int expon;

#ifndef __MIKROC_PRO_FOR_PIC__
    expon = ((*(unsigned long *)&x >> DBL_MANT_DIG) & 255);
#endif
#ifdef __MIKROC_PRO_FOR_PIC__
    expon = (*(unsigned long *)&x >> DBL_MANT_DIG_PIC) & 255;
#endif
    expon = expon - 127;

    if (expon < 0)
    {
        if (x < 0.0)
        {
            return -1.0;
        }
        else
        {
            return  0.0;
        }
    }

    if ((unsigned)expon > sizeof(double) * CHAR_BIT - 8)
    {
        return x;
    }
    i = _FRNDINT( x );

    if (i > x)
    {
        return i - 1.0;
    }

    return i;
}

/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __ALTITUDE3_DRV_SPI__

void altitude3_spiDriverInit(T_ALTITUDE3_P gpioObj, T_ALTITUDE3_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __ALTITUDE3_DRV_I2C__

void altitude3_i2cDriverInit(T_ALTITUDE3_P gpioObj, T_ALTITUDE3_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    updateCheck = 0;
}

#endif
#ifdef   __ALTITUDE3_DRV_UART__

void altitude3_uartDriverInit(T_ALTITUDE3_P gpioObj, T_ALTITUDE3_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */

T_ALTITUDE3_RETVAL altitude3_measurementMode( uint16_t modeCmd )
{
    T_ALTITUDE3_RETVAL retVal;
    uint8_t tempData[ 2 ];
    
    if ((modeCmd == _ALTITUDE3_LOW_POWER_T_FIRST) || (modeCmd == _ALTITUDE3_NORMAL_T_FIRST) || (modeCmd == _ALTITUDE3_LOW_NOISE_T_FIRST) || (modeCmd == _ALTITUDE3_ULTRA_LOW_NOISE_T_FIRST))
    {
        retVal = _ALTITUDE3_T_FIRST_ORDER;
    }
    else if ((modeCmd == _ALTITUDE3_LOW_POWER_P_FIRST) || (modeCmd == _ALTITUDE3_NORMAL_P_FIRST) || (modeCmd == _ALTITUDE3_LOW_NOISE_P_FIRST) || (modeCmd == _ALTITUDE3_ULTRA_LOW_NOISE_P_FIRST))
    {
        retVal = _ALTITUDE3_P_FIRST_ORDER;
    }
    else
    {
        return _ALTITUDE3_ERROR;
    }
    
    tempData[ 0 ] = modeCmd >> 8;
    tempData[ 1 ] = modeCmd;
    
    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, 2, END_MODE_STOP );
    
    return retVal;
}

void altitude3_softReset( void )
{
    uint8_t tempData[ 2 ];
    
    tempData[ 0 ] = 0x80;
    tempData[ 1 ] = 0x5D;
    
    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, 2, END_MODE_STOP );
    
    updateCheck = 0;
}

void altitude3_readID( uint16_t *deviceID )
{
    uint8_t tempData[ 3 ];

    tempData[ 0 ] = 0xEF;
    tempData[ 1 ] = 0xC8;
    
    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, 2, END_MODE_RESTART );
    hal_i2cRead( _slaveAddress, tempData, 3, END_MODE_STOP );
    
    *deviceID = tempData[ 0 ];
    *deviceID <<= 8;
    *deviceID |= tempData[ 1 ];
}

T_ALTITUDE3_RETVAL altitude3_readADCResults( uint8_t readOrder, int16_t *temperature, uint32_t *pressure )
{
    uint8_t tempData[ 9 ];
    uint8_t tempIndx;
    uint8_t pressIndx;
    
    hal_i2cStart();
    hal_i2cRead( _slaveAddress, tempData, 9, END_MODE_STOP );
    
    if (readOrder == _ALTITUDE3_T_FIRST_ORDER)
    {
        tempIndx = 0;
        pressIndx = 3;
    }
    else if (readOrder == _ALTITUDE3_P_FIRST_ORDER)
    {
        pressIndx = 0;
        tempIndx = 6;
    }
    else
    {
        return _ALTITUDE3_ERROR;
    }
    
    *temperature = tempData[ tempIndx ];
    *temperature <<= 8;
    *temperature |= tempData[ tempIndx + 1 ];
    
    *pressure = tempData[ pressIndx ];
    *pressure <<= 8;
    *pressure |= tempData[ pressIndx + 1 ];
    *pressure <<= 8;
    *pressure |= tempData[ pressIndx + 2 ];
    
    return _ALTITUDE3_OK;
}

void altitude3_init( T_altitude3_param *s )
{
    uint16_t otpData[ 4 ];
    uint8_t cnt;
    
    _read_otp( otpData );
    
    for (cnt = 0; cnt < 4; cnt++)
    {
        s->sensor_const[ cnt ] = otpData[ cnt ];
    }
    
    s->p_Pa_calib[ 0 ] = 45000;
    s->p_Pa_calib[ 1 ] = 80000;
    s->p_Pa_calib[ 2 ] = 105000;
    s->LUT_lower = 3.5 * 1048576.0;
    s->LUT_upper = 11.5 * 1048576.0;
    s->quadr_factor = 1.0 / 16777216.0;
    s->offst_factor = 2048;
    s->min_delay_us = 100000;
}

T_ALTITUDE3_RETVAL altitude3_getData( uint8_t read_order, int8_t *temperature, uint16_t *pressure, int16_t *altitude )
{
    int16_t temp;
    uint32_t press;
    T_ALTITUDE3_RETVAL errorCheck;
    static T_altitude3_param otpParam;
    int16_t t;
    float s1;
    float sbuff[ 3 ];
    float abc_const[ 3 ];
    float res;
    
    Delay_100ms();
    
    if (updateCheck == 0)
    {
        altitude3_init( &otpParam );
        updateCheck = 1;
        
        return _ALTITUDE3_INITIALIZED;
    }
    
    errorCheck = altitude3_readADCResults( read_order, &temp, &press );
    
    if (errorCheck == _ALTITUDE3_ERROR)
    {
        return errorCheck;
    }
    
    t = (int16_t)(temp - 32768);
    s1 = otpParam.sensor_const[0];
    s1 *= t;
    s1 *= t; 
    s1 *= otpParam.quadr_factor;
    s1 += otpParam.LUT_lower;
    sbuff[ 0 ] = s1;
    res = otpParam.offst_factor;
    res *= otpParam.sensor_const[3];
    s1 = otpParam.sensor_const[1];
    s1 *= t;
    s1 *= t;
    s1 *= otpParam.quadr_factor;
    s1 += res;
    sbuff[ 1 ] = s1;
    s1 = otpParam.sensor_const[2];
    s1 *= t;
    s1 *= t;
    s1 *= otpParam.quadr_factor;
    s1 += otpParam.LUT_upper;
    sbuff[ 2 ] = s1;
    
    calc_conv_const( otpParam.p_Pa_calib, sbuff, abc_const );
    
    res = -45.0 + 175.0/65536.0 * temp;
    *temperature = res;

    res = abc_const[2];
    res += press;
    res = abc_const[1] / res;
    res += abc_const[0];
    res /= 100;
    *pressure = res;
    
    res = *pressure / 1013.96;
    res = _pow( res, 0.19022 );
    res = 1.0 - res;
    s1 = *temperature;
    s1 += 273.15;
    s1 /= 0.0065;
    res *= s1;
    *altitude = res;
    
    return errorCheck;
}

/* -------------------------------------------------------------------------- */
/*
  __altitude3_driver.c

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