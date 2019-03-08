/*
    __altitude3_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __altitude3_driver.h
@brief    Altitude_3 Driver
@mainpage Altitude_3 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   ALTITUDE3
@brief      Altitude_3 Click Driver
@{

| Global Library Prefix | **ALTITUDE3** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Dec 2018.**      |
| Developer             | **Nemanja Medakovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _ALTITUDE3_H_
#define _ALTITUDE3_H_

/** 
 * @macro T_ALTITUDE3_P
 * @brief Driver Abstract type 
 */
#define T_ALTITUDE3_P    const uint8_t*
#define T_ALTITUDE3_RETVAL     uint8_t

/** @defgroup ALTITUDE3_COMPILE Compilation Config */              /** @{ */

//  #define   __ALTITUDE3_DRV_SPI__                            /**<     @macro __ALTITUDE3_DRV_SPI__  @brief SPI driver selector */
   #define   __ALTITUDE3_DRV_I2C__                            /**<     @macro __ALTITUDE3_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __ALTITUDE3_DRV_UART__                           /**<     @macro __ALTITUDE3_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup ALTITUDE3_VAR Variables */                           /** @{ */

/** I2C Slave Address */
extern const uint8_t _ALTITUDE3_SLAVE_ADDR                 ;

/** Settings for measurement mode */
extern const uint16_t _ALTITUDE3_LOW_POWER_T_FIRST         ;
extern const uint16_t _ALTITUDE3_LOW_POWER_P_FIRST         ;
extern const uint16_t _ALTITUDE3_NORMAL_T_FIRST            ;
extern const uint16_t _ALTITUDE3_NORMAL_P_FIRST            ;
extern const uint16_t _ALTITUDE3_LOW_NOISE_T_FIRST         ;
extern const uint16_t _ALTITUDE3_LOW_NOISE_P_FIRST         ;
extern const uint16_t _ALTITUDE3_ULTRA_LOW_NOISE_T_FIRST   ;
extern const uint16_t _ALTITUDE3_ULTRA_LOW_NOISE_P_FIRST   ;

/** Returned values of functions */
extern const uint8_t _ALTITUDE3_OK                         ;
extern const uint8_t _ALTITUDE3_T_FIRST_ORDER              ;
extern const uint8_t _ALTITUDE3_P_FIRST_ORDER              ;
extern const uint8_t _ALTITUDE3_INITIALIZED                ;
extern const uint8_t _ALTITUDE3_ERROR                      ;

                                                                       /** @} */
/** @defgroup ALTITUDE3_TYPES Types */                             /** @{ */

typedef struct {

    uint32_t  min_delay_us;
    uint16_t  sensor_const[4];
    uint32_t  p_Pa_calib[3];
    float     LUT_lower;
    float     LUT_upper;
    double    quadr_factor;
    uint16_t  offst_factor;

} T_altitude3_param;

                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup ALTITUDE3_INIT Driver Initialization */              /** @{ */

#ifdef   __ALTITUDE3_DRV_SPI__
void altitude3_spiDriverInit(T_ALTITUDE3_P gpioObj, T_ALTITUDE3_P spiObj);
#endif
#ifdef   __ALTITUDE3_DRV_I2C__
void altitude3_i2cDriverInit(T_ALTITUDE3_P gpioObj, T_ALTITUDE3_P i2cObj, uint8_t slave);
#endif
#ifdef   __ALTITUDE3_DRV_UART__
void altitude3_uartDriverInit(T_ALTITUDE3_P gpioObj, T_ALTITUDE3_P uartObj);
#endif

                                                                       /** @} */
/** @defgroup ALTITUDE3_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Set Measurement Mode function
 *
 * @param[in] modeCmd  Command which selects a measurement mode
 *
 * @returns 0x01 - Temperature goes first, 0x02 - Pressure goes first, 0xFF - Command Error
 *
 * Function sends a command which selects a measurement mode and data reading order.
 */
T_ALTITUDE3_RETVAL altitude3_measurementMode( uint16_t modeCmd );

/**
 * @brief Software Reset function
 *
 * Function sends a command to perform a SW Reset of the device.
 * @note This command triggers the sensor to reset all internal state machines and reload calibration data from the memory.
 */
void altitude3_softReset( void );

/**
 * @brief Read ID function
 *
 * @param[out] deviceID  Memory where device ID be stored
 *
 * Function sends a command to read the device ID.
 */
void altitude3_readID( uint16_t *deviceID );

/**
 * @brief Read AD Conversion function
 *
 * @param[in] readOrder  Determines in which order data be read
 * @param[out] temperature  Memory where temperature converted data be stored
 * @param[out] pressure  Memory where pressure converted data be stored
 *
 * @returns 0x00 - OK, 0xFF - Data Order Error
 *
 * Function reads results of AD conversion, which consists of the 16bit temperature and 24bit pressure data in determined order.
 */
T_ALTITUDE3_RETVAL altitude3_readADCResults( uint8_t readOrder, int16_t *temperature, uint32_t *pressure );

/**
 * @brief Init function
 *
 * @param[out] s  Object where initialization data and data from OTP sensor be stored
 *
 * Function sends a command to read calibration data from OTP sensor and other initialization data, which is necessary for calculations.
 * @note The reading from OTP sensor should be performed after power up or after SW reset.
 */
void altitude3_init( T_altitude3_param *s );

/**
 * @brief Get Data function
 *
 * @param[in] read_order  Determines in which order data be read
 * @param[out] temperature  Memory where temperature data calculated to Celsius degrees be stored
 * @param[out] pressure  Memory where pressure data calculated to mbar[hPa] be stored
 * @param[out] altitude  Memory where altitude data calculated to meters be stored
 *
 * @returns 0x00 - OK, 0x03 - Calibration Done, 0xFF - Data Order Error
 *
 * Function performs a calibration data reading, only once, and then reads a temperature and pressure data and calculates these values
 * to standard units. Also calculates the altitude depending on the temperature and pressure data.
 */
T_ALTITUDE3_RETVAL altitude3_getData( uint8_t read_order, int8_t *temperature, uint16_t *pressure, int16_t *altitude );

                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Altitude_3_STM.c
    @example Click_Altitude_3_TIVA.c
    @example Click_Altitude_3_CEC.c
    @example Click_Altitude_3_KINETIS.c
    @example Click_Altitude_3_MSP.c
    @example Click_Altitude_3_PIC.c
    @example Click_Altitude_3_PIC32.c
    @example Click_Altitude_3_DSPIC.c
    @example Click_Altitude_3_AVR.c
    @example Click_Altitude_3_FT90x.c
    @example Click_Altitude_3_STM.mbas
    @example Click_Altitude_3_TIVA.mbas
    @example Click_Altitude_3_CEC.mbas
    @example Click_Altitude_3_KINETIS.mbas
    @example Click_Altitude_3_MSP.mbas
    @example Click_Altitude_3_PIC.mbas
    @example Click_Altitude_3_PIC32.mbas
    @example Click_Altitude_3_DSPIC.mbas
    @example Click_Altitude_3_AVR.mbas
    @example Click_Altitude_3_FT90x.mbas
    @example Click_Altitude_3_STM.mpas
    @example Click_Altitude_3_TIVA.mpas
    @example Click_Altitude_3_CEC.mpas
    @example Click_Altitude_3_KINETIS.mpas
    @example Click_Altitude_3_MSP.mpas
    @example Click_Altitude_3_PIC.mpas
    @example Click_Altitude_3_PIC32.mpas
    @example Click_Altitude_3_DSPIC.mpas
    @example Click_Altitude_3_AVR.mpas
    @example Click_Altitude_3_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __altitude3_driver.h

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