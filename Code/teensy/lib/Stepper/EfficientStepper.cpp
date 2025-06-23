/**************************************************************************/
/*!
    @file     EfficientStepper.cpp
    @license  MIT (see license.txt)

    This library provides an efficient way to control a stepper motor
    by cutting off the current to the motor when it is not moving,
    thus reducing heat and power consumption.

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "Arduino.h"
#include <SPI.h>
#include "EfficientStepper.hpp"

/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/

/**
 * @brief Construct a new Efficient Stepper:: Efficient Stepper object
 *
 * @param number_of_steps The total number of steps for one revolution of the motor.
 * @param motor_pin_1 The pin connected to the first motor wire.
 * @param motor_pin_2 The pin connected to the second motor wire.
 * @param motor_pin_3 The pin connected to the third motor wire.
 * @param motor_pin_4 The pin connected to the fourth motor wire.
 */
EfficientStepper::EfficientStepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                                   int motor_pin_3, int motor_pin_4) : _stepper_motor(Stepper(number_of_steps, motor_pin_1, motor_pin_2, motor_pin_3, motor_pin_4))
{
    _stepper_motor_pins[0] = motor_pin_1;
    _stepper_motor_pins[1] = motor_pin_2;
    _stepper_motor_pins[2] = motor_pin_3;
    _stepper_motor_pins[3] = motor_pin_4;
    _stepper_motor_enabled = true;
}

/**
 * @brief Saves the current digital state of the four motor pins.
 * This is called before disabling the motor to remember which coils were active.
 */
void EfficientStepper::_save_status()
{
    if (_stepper_motor_enabled)
    // If the motor is already disabled, then we should not try to read the current pins;
    {
        for (int i = 0; i < 4; ++i)
        {
            _stepper_motor_status[i] = digitalRead(_stepper_motor_pins[i]);
        }
    }
}

/**
 * @brief Re-enables the motor by restoring the saved pin states.
 * This allows the motor to hold its position and be ready to move again.
 */
void EfficientStepper::enable()
{
    for (int i = 0; i < 4; ++i)
    {
        digitalWrite(_stepper_motor_pins[i], _stepper_motor_status[i]);
    }

    _stepper_motor_enabled = true;
}

/**
 * @brief Disables the motor by setting all motor pins to LOW.
 * This de-energizes the coils, saving power and reducing heat.
 * The current pin states are saved before disabling.
 */
void EfficientStepper::disable()
{
    _save_status();
    for (int i = 0; i < 4; ++i)
    {
        digitalWrite(_stepper_motor_pins[i], LOW);
    }
    _stepper_motor_enabled = false;
}

/**
 * @brief Moves the motor by a specified number of steps.
 * If the motor is disabled, it is first enabled. The total step count is updated.
 *
 * @param steps The number of steps to move. Can be positive or negative.
 */
void EfficientStepper::step(int steps)
{
    if (!_stepper_motor_enabled)
    {
        enable();
    }
    _stepper_motor.step(steps);
    _total_steps = _total_steps + steps;
}

/**
 * @brief Sets the speed of the motor.
 *
 * @param speed The speed in RPM.
 */
void EfficientStepper::setSpeed(long speed)
{
    _stepper_motor.setSpeed(speed);
}

/**
 * @brief Retrieves the total number of steps taken since construction or the last reset.
 *
 * @return int The cumulative number of steps.
 */
int EfficientStepper::get_total_steps()
{
    return _total_steps;
}

/**
 * @brief Resets the cumulative step counter to zero.
 */
void EfficientStepper::reset()
{
    _total_steps = 0;
}