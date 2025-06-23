/**************************************************************************/
/*!
    @file     LTC2326_16.cpp
    @author   Daniel Berard
    @license  MIT (see license.txt)

    This is a library for the Linear Technology LTC2326-16 ADC in Teensy 4.1.

    @section  HISTORY

    v1.0 - First release by Daniel Berard
*/
/**************************************************************************/

#include "Arduino.h"
#include <SPI.h>
#include "LTC2326_16.hpp"

/**
 * @brief Construct a new ltc2326 16::ltc2326 16 object
 *
 * @param cs Chip select pin
 * @param cnv Conversion start pin
 * @param busy Busy status indicator pin
 */

LTC2326_16::LTC2326_16(byte cs, byte cnv, byte busy)
{
    pinMode(cs, OUTPUT);
    pinMode(cnv, OUTPUT);
    pinMode(busy, INPUT);
    digitalWrite(cs, HIGH);
    digitalWrite(cnv, LOW);
    _cs = cs;
    _cnv = cnv;
    _busy = busy;
}

/**
 * @brief Initiates a conversion by setting the CNV pin HIGH.
 */

void LTC2326_16::convert()
{
    digitalWrite(_cnv, HIGH);
}

/**
 * @brief Checks if the ADC is currently performing a conversion.
 *
 * @return true if the BUSY pin is HIGH, indicating a conversion is in progress.
 * @return false if the BUSY pin is LOW, indicating the ADC is ready.
 */

bool LTC2326_16::busy()
{
    bool status;
    status = (bool)digitalRead(_busy);
    return status;
}

/**
 * @brief Reads the 16-bit conversion result from the ADC via SPI.
 *
 * This function resets the CNV pin to LOW to prepare for the next conversion,
 * then reads the 16-bit data from the SPI bus.
 *
 * @return int16_t The signed 16-bit result from the ADC.
 */

int16_t LTC2326_16::read()
{
    int16_t val;

    digitalWrite(_cnv, LOW); // Reset CNV for another conversion later on
    SPI1.beginTransaction(_spi_settings);
    digitalWrite(_cs, HIGH);
    val = SPI1.transfer16(0x00);
    digitalWrite(_cs, LOW);
    return val;
}

/**
 * @brief Reads the ADC conversion result and converts it to volts.
 *
 * @return float The calculated voltage, based on the ADC reading and the reference voltage.
 */
float LTC2326_16::read_volts()
{
    int16_t val = read();
    return val * _ref_buffer_volts;
}