/**************************************************************************/
/*!
    @file     AD5761.cpp
    @license  MIT (see license.txt)

    This is a library for the Analog Devices AD5761 16-Bit DAC.

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "Arduino.h"
#include "AD5761.hpp"
#include "SPI.h"

/**
 * @brief Construct a new AD5761::AD5761 object
 *
 * @param cs The chip select pin
 * @param mode The control register mode setting. Determines voltage range.
 *
 * This constructor initializes the AD5761 object with the specified chip select pin and control register mode.
 * The chip select pin is set as an output and initially set high.
 */
AD5761::AD5761(byte cs, uint16_t mode)
{
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    _cs = cs;
    _mode = mode;
}

/**
 * @brief Initialize the SPI communication and the AD5761 device.
 *
 * This function initializes the SPI communication and the AD5761 device.
 * It sets the chip select pin as an output, begins the SPI hardware, and performs a software reset of the AD5761.
 * The control register settings are then reapplied.
 */
void AD5761::spi_init()
{
    pinMode(_cs, OUTPUT); // Set the SS0 pin as an output
    SPI.begin();          // Begin SPI hardware
    delay(100);
    // AD5761 software reset
    write(CMD_SW_FULL_RESET, 0);
    delay(100);
    // Set the Mode of AD5761
    write(CMD_WR_CTRL_REG, _mode);
}

/**
 * @brief Performs a software reset of the AD5761 and re-applies the control register settings.
 *
 * This function performs a software reset of the AD5761 and re-applies the control register settings.
 * It writes the software reset command to the device and then reapplies the control register settings.
 */
void AD5761::reset()
{
    // AD5761 software reset
    write(CMD_SW_FULL_RESET, 0);
    delay(100);
    // Set the Mode of AD5761
    write(CMD_WR_CTRL_REG, _mode);
}

/**
 * @brief Writes a 16-bit value to the specified register.
 *
 * @param reg_addr_cmd The 8-bit command, including the register address.
 * @param reg_data The 16-bit data to be written to the register.
 *
 * This function writes a 16-bit value to the specified register.
 * It begins an SPI transaction, sets the chip select pin low, and transfers the command and data bytes.
 */
void AD5761::write(uint8_t reg_addr_cmd, uint16_t reg_data)
{
    uint8_t data[3];
    SPI.beginTransaction(_spi_settings);
    digitalWrite(_cs, LOW);
    data[0] = reg_addr_cmd;
    data[1] = (reg_data & 0xFF00) >> 8;
    data[2] = (reg_data & 0x00FF) >> 0;
    for (int i = 0; i < 3; i++)
    {
        SPI.transfer(data[i]);
    }
    digitalWrite(_cs, HIGH);
}

/**
 * @brief Sets the output voltage of the DAC.
 *
 * This function calculates the corresponding 16-bit DAC code for the given voltage
 * based on the device's configured range and writes it to the DAC register.
 * Note: The current formula `(voltage / 2.5 + 4) / 8 * 65536` seems specific
 * to a certain voltage range and may need to be generalized if other ranges are used.
 *
 * @param voltage The desired output voltage.
 */
void AD5761::write_volt(float voltage)
{
    int set_val = (int)((voltage / 2.5 + 4) / 8 * 65536);
    write(CMD_WR_UPDATE_DAC_REG, set_val);
}

/**
 * @brief Reads data from the specified register.
 *
 * @param reg_addr_cmd The 8-bit command, including the register address.
 * The read data is stored in the internal `_spi_buffer`.
 */
void AD5761::read(uint8_t reg_addr_cmd)
{
    digitalWrite(_cs, LOW);
    delay(1);
    _spi_buffer[0] = SPI.transfer(reg_addr_cmd);
    _spi_buffer[1] = SPI.transfer(0xFF); // dummy
    _spi_buffer[2] = SPI.transfer(0xFF); // dummy
    digitalWrite(_cs, HIGH);
    delay(1);
}