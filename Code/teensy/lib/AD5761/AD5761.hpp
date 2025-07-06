/**************************************************************************/
/*!
    @file     AD5761.hpp
    @license  MIT (see license.txt)

    This is a library for the Analog Devices AD5761 16-Bit DAC.

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#ifndef AD5761_H
#define AD5761_H

#include "Arduino.h"
#include <SPI.h>

/*=========================================================================
    INPUT SHIFT REGISTER COMMANDS
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
 *  @brief  Class that stores state and functions for interacting with
 *          AD5761 DAC
 */
class AD5761
{

public:
  // mode
  // 0b0000000101000 -10V, +10V
  // 0b0000000101101 -3 to 3V
  /**
   * @brief Construct a new AD5761 object
   *
   * @param cs The chip select pin
   * @param mode The control register mode setting. Determines voltage range.
   */
  AD5761(byte cs, uint16_t mode);

  /**
   * @brief Write data to a register
   *
   * @param reg_addr_cmd The command byte, including the register address
   * @param reg_data The 16-bit data to write
   */
  void write(uint8_t reg_addr_cmd, uint16_t reg_data);
  /**
   * @brief Set the output voltage
   *
   * @param voltage The desired output voltage. The valid range depends on the mode.
   */
  void write_volt(float voltage);
  /**
   * @brief Read data from a register
   *
   * @param reg_addr_cmd The command byte, including the register address
   */
  void read(uint8_t reg_addr_cmd);

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
  SPISettings _spi_settings = SPISettings(40000000, MSBFIRST, SPI_MODE2); ///< SPI settings
  byte _spi_buffer[3]; ///< Buffer for SPI communication
};

#endif
