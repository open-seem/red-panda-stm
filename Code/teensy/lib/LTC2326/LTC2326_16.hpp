/**************************************************************************/
/*

LTC2326-16 library for Teensy 3.1
Last updated Oct 14, 2015
http://dberard.com/home-built-stm/


 * Copyright (c) Daniel Berard, daniel.berard@mail.mcgill.ca
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.

*/
/**************************************************************************/

#ifndef LTC2326_16_h
#define LTC2326_16_h

#include <Arduino.h>
#include <SPI.h> // include the SPI library:

#define ADC_BITS 16                                  ///< ADC resolution in bits
const int MAX_ADC_OUT = (1 << (ADC_BITS - 1)) - 1; ///< Maximum ADC output value
const int MIN_ADC_OUT = -(1 << (ADC_BITS - 1));    ///< Minimum ADC output value

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          LTC2326-16 ADC
 */
class LTC2326_16
{

public:
  /**
   * @brief Construct a new LTC2326_16 object
   *
   * @param cs Chip select pin
   * @param cnv Conversion start pin
   * @param busy Busy status indicator pin
   */
  LTC2326_16(byte cs, byte cnv, byte busy);
  /**
   * @brief Initiate a conversion by pulsing the CNV pin.
   */
  void convert();
  /**
   * @brief Check the busy status of the ADC.
   *
   * @return true if a conversion is in progress, false otherwise.
   */
  bool busy();
  /**
   * @brief Read the raw 16-bit value from the ADC.
   *
   * @return The signed 16-bit integer result of the conversion.
   */
  int16_t read();
  /**
   * @brief Read the ADC value and convert it to a voltage.
   *
   * @return The voltage corresponding to the ADC reading.
   */
  float read_volts();

private:
  byte _cs;   ///< Chip select pin
  byte _cnv;  ///< Conversion start pin
  byte _busy; ///< Busy status indicator pin
  const SPISettings _spi_settings = SPISettings(40000000, MSBFIRST, SPI_MODE2); ///< SPI settings
  const float _ref_buffer_volts = 4.096f; ///< Reference voltage
};

#endif // LTC2326_16_h
