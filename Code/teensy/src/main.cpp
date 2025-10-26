/**
 * @file main.cpp
 * @brief Main firmware for the Teensy-based STM controller.
 * 
 * This file contains the main entry point for the firmware. It handles serial communication 
 * for receiving commands, and calls the appropriate functions in the STM class to control 
 * the scanning tunneling microscope.
 */
#include <Arduino.h>
#include <SPI.h> // include the SPI library
#include "stm_firmware.hpp"

#define CMD_LENGTH 4

/**
 * @brief Processes a serial command.
 * 
 * This function parses the received serial command and calls the corresponding STM function.
 * 
 * @param command The 4-character command string.
 * @param stm A reference to the STM object.
 */
void serialCommand(String command, STM &stm)
{

  if (command.length() == CMD_LENGTH)
  {
    // Reset
    if (command == "RSET")
    {
      stm.reset();
    }
    // Bias control
    if (command == "BIAS")
    {
      int value = Serial.parseInt();
      stm.set_dac_bias(value);
    }
    // Stepper motor control
    if (command == "MTMV")
    {
      int value = Serial.parseInt();
      stm.move_motor(value);
    }
    if (command == "STPF")
    {
      stm.move_motor(1);
    }
    if (command == "STPB")
    {
      stm.move_motor(-1);
    }
    if (command == "STPS" || command == "STOP")
    {
      stm.stop_motor();
      stm.stm_status.is_approaching = false;
      stm.stm_status.is_const_current = false;
      stm.stm_status.is_scanning = false;
    }
    // DAC control
    if (command == "DACX")
    {
      int value = Serial.parseInt();
      stm.set_dac_x(value);
    }
    if (command == "DACY")
    {
      int value = Serial.parseInt();
      stm.set_dac_y(value);
    }
    if (command == "DACZ")
    {
      int value = Serial.parseInt();
      stm.set_dac_z(value);
    }
    // ADC READ
    if (command == "ADCR")
    {
      int val = stm.read_adc();
      Serial.println(val);
    }
    // Get status
    if (command == "GSTS")
    {
      char buffer[100];
      stm.get_status().to_char(buffer);
      Serial.println(buffer);
    }

    // Approach
    if (command == "APRH")
    {
      int adc_target = Serial.parseInt();
      int max_steps = Serial.parseInt();
      int step_interval = Serial.parseInt();
      int direction = Serial.parseInt();
      if (direction == 0) direction = 1; // Default to forward if not specified
      stm.start_approach(adc_target, max_steps, step_interval, direction);
    }
    // MeasureIV
    if (command == "IVME")
    {
      int bias_start = Serial.parseInt();
      int bias_end = Serial.parseInt();
      int bias_step = Serial.parseInt();
      stm.generate_iv_curve(bias_start, bias_end, bias_step);
    }
    if (command == "IVGE")
    {
      stm.send_iv_curve();
    }
    // Start const current mode
    if (command == "CCON")
    {
      int adc_target = Serial.parseInt();
      stm.turn_on_const_current(adc_target);
    }
    // Turn off const current
    if (command == "CCOF")
    {
      stm.turn_off_const_current();
    }
    // Setup PID values
    if (command == "PIDS")
    {
      double Kp = Serial.parseFloat();
      double Ki = Serial.parseFloat();
      double Kd = Serial.parseFloat();
      stm.Kp = Kp;
      stm.Ki = Ki;
      stm.Kd = Kd;
    }
    if (command == "SCST")
    {
      int x_start = Serial.parseInt();
      int x_end = Serial.parseInt();
      int x_resolution = Serial.parseInt();
      int y_start = Serial.parseInt();
      int y_end = Serial.parseInt();
      int y_resolution = Serial.parseInt();
      int sample_per_pixel = Serial.parseInt();
      stm.start_scan(x_start, x_end, x_resolution, y_start, y_end, y_resolution, sample_per_pixel);
    }
    if (command == "TEST")
    {
      stm.test_piezo();
    }
    // PID Debug
    if (command == "PIDD")
    {
      stm.print_pid_debug();
    }
    // Approach Debug
    if (command == "APRD")
    {
      stm.print_approach_debug();
    }
    // Trigger Recovery
    if (command == "APRR")
    {
      stm.trigger_approach_recovery();
    }

    // Enable fine motor mode (single-step between Z sweeps)
    if (command == "FINE")
    {
      // Optional parameter: 0=disable, 1=enable. If no param provided, enable.
      int val = Serial.parseInt();
      if (val == 0)
      {
        stm.set_fine_mode(false);
      }
      else
      {
        stm.set_fine_mode(true);
      }
    }

    // Set approach fine step size (Z sweep interval)
    if (command == "APFS")
    {
      int step = Serial.parseInt();
      stm.set_approach_fine_step_size(step);
    }

    // Set approach Z search range
    if (command == "APRG")
    {
      int range = Serial.parseInt();
      stm.set_approach_z_search_range(range);
    }

    // Note: APST (stop approach) command removed; use STOP to halt operations.

  }
}

/**
 * @brief Checks for incoming serial data.
 * 
 * This function reads the serial port and, if a command of the correct length is received,
 * it calls serialCommand() to process it.
 * 
 * @param stm A reference to the STM object.
 */
void checkSerial(STM &stm)
{
  String serialString;
  if (Serial.available() >= CMD_LENGTH)
  {
    for (int i = 0; i < CMD_LENGTH; i++) // Read command with length CMD_LENGTH
    {
      char inChar = Serial.read();
      serialString += inChar;
    }
    serialCommand(serialString, stm);
  }
}

STM stm = STM(); /**< The main STM control object. */

/**
 * @brief Initializes the hardware and software.
 * 
 * This function is called once at startup. It initializes the serial port, SPI communication,
 * and resets the STM to a known state.
 */
void setup()
{
  // initialize the serial port
  Serial.begin(230400);
  // initialize SPI:
  SPI.begin();
  // Set Up SPI1 for Teensy 4.1
  SPI1.setSCK(27);
  SPI1.setCS(38);
  SPI1.setMISO(39);
  SPI1.begin();
  // Reset all;
  stm.reset();
  // Init
}

/**
 * @brief The main loop.
 * 
 * This function is called repeatedly. It checks for serial commands and calls the STM's update 
 * functions to handle ongoing tasks like the approach, constant current mode, and scanning.
 */
void loop()
{
  checkSerial(stm);
  stm.update();
  if (stm.stm_status.is_approaching)
  {
    stm.approach();
  }
  if (stm.stm_status.is_const_current)
  {
    stm.control_current(stm.read_adc_raw());
  }
}
