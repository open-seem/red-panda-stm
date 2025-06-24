/**************************************************************************/
/*!
    @file     AD5761.hpp
    @license  MIT (see license.txt)

    This is a library for the Analog Devices AD5761 16-Bit DAC.

    @section  HISTORY

    v1.0 - First release
    v1.1 - Corrected bugs related to SPI command formatting and data coding.
*/
/**************************************************************************/

#ifndef AD5761_H
#define AD5761_H

#include "Arduino.h"
#include <SPI.h>

/*=========================================================================
    INPUT SHIFT REGISTER COMMANDS (4-BIT VALUES)
    -----------------------------------------------------------------------*/
#define CMD_NOP 0x0             ///< No operation
#define CMD_WR_TO_INPUT_REG 0x1 ///< Write to Input Register n
#define CMD_UPDATE_DAC_REG 0x2  ///< Update DAC Register n with contents of Input Register n
#define CMD_WR_UPDATE_DAC_REG 0x3 ///< Write to and update DAC Channel n
#define CMD_WR_CTRL_REG 0x4     ///< Write to Control Register
#define CMD_NOP_ALT_1 0x5       ///< No operation
#define CMD_NOP_ALT_2 0x6       ///< No operation
#define CMD_SW_DATA_RESET 0x7   ///< Software Data Reset
#define CMD_RESERVED 0x8        ///< Reserved
#define CMD_DIS_DAISY_CHAIN 0x9 ///< Disable Daisy-Chain
#define CMD_RD_INPUT_REG 0xA    ///< Read Input Register n
#define CMD_RD_DAC_REG 0xB      ///< Read DAC Register n
#define CMD_RD_CTRL_REG 0xC     ///< Read Control Register
#define CMD_NOP_ALT_3 0xD       ///< No operation
#define CMD_NOP_ALT_4 0xE       ///< No operation
#define CMD_SW_FULL_RESET 0xF   ///< Software Full Reset
/*=========================================================================*/

/*!
 * @brief  Class that stores state and functions for interacting with
 * AD5761 DAC
 */
class AD5761
{

public:
  /**
   * @brief Construct a new AD5761 object
   *
   * @param cs The chip select pin
   * @param mode The 16-bit control register setting. Determines voltage range and data coding.
   *
   * @note Example Control Register Mode Bits (see datasheet Table 11 & 12):
   * RA[2:0] (Range):
   * - 0b000: +/-10V
   * - 0b010: +/-5V
   * - 0b101: +/-3V
   * B2C (Coding):
   * - Bit 6 = 1 for Two's Complement
   * - Bit 6 = 0 for Straight Binary
   */
  AD5761(byte cs, uint16_t mode);

  /**
   * @brief Write data to a register
   *
   * @param reg_addr_cmd The 4-bit command (see command definitions)
   * @param reg_data The 16-bit data to write
   */
  void write(uint8_t reg_addr_cmd, uint16_t reg_data);

  /**
   * @brief Read data from a register
   *
   * @param reg_addr_cmd The 4-bit read command
   * @return The 24-bit word read from the device.
   */
  uint32_t read(uint8_t reg_addr_cmd);

  /**
   * @brief Initialize the SPI communication
   */
  void spi_init();
  
  /**
   * @brief Reset the device and set its operating mode
   */
  void reset();

private:
  byte _cs;         ///< Chip select pin
  uint16_t _mode;   ///< Control register mode setting
  SPISettings _spi_settings = SPISettings(50000000, MSBFIRST, SPI_MODE2); ///< SPI settings for AD5761
};

#endif