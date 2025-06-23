/**************************************************************************/
/*!
    @file     EfficientStepper.hpp
    @license  MIT (see license.txt)

    This library provides an efficient way to control a stepper motor
    by cutting off the current to the motor when it is not moving,
    thus reducing heat and power consumption.

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>
#include <Stepper.h>

/*!
 *  @brief  Class that provides efficient control of a stepper motor.
 */
class EfficientStepper
{

public:
    /**
     * @brief Construct a new Efficient Stepper object
     *
     * @param number_of_steps The total number of steps for one revolution of the motor.
     * @param motor_pin_1 The pin connected to the first motor wire.
     * @param motor_pin_2 The pin connected to the second motor wire.
     * @param motor_pin_3 The pin connected to the third motor wire.
     * @param motor_pin_4 The pin connected to the fourth motor wire.
     */
    EfficientStepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                     int motor_pin_3, int motor_pin_4);
    /**
     * @brief Move the motor a specified number of steps.
     *
     * @param steps The number of steps to move. Positive for one direction, negative for the other.
     */
    void step(int steps);
    /**
     * @brief Enable the motor by restoring power to the coils.
     */
    void enable();
    /**
     * @brief Disable the motor by cutting power to the coils to save energy.
     */
    void disable();
    /**
     * @brief Set the motor speed.
     *
     * @param speed The desired speed in revolutions per minute.
     */
    void setSpeed(long speed);
    /**
     * @brief Get the total number of steps moved since the last reset.
     *
     * @return int The total number of steps.
     */
    int get_total_steps();
    /**
     * @brief Reset the total step counter to zero.
     */
    void reset();

private:
    bool _stepper_motor_status[4] = {false, false, false, false}; ///< Stores the state of the motor pins before disabling.
    bool _stepper_motor_enabled = false; ///< Tracks if the motor is currently enabled or disabled.
    int _stepper_motor_pins[4]; ///< Array to hold the motor pin numbers.
    Stepper _stepper_motor; ///< The underlying Stepper object from the Arduino library.
    /**
     * @brief Save the current state of the motor pins before disabling.
     */
    void _save_status();
    int _total_steps = 0; ///< Counter for the total steps moved.
};

#endif // STEPPER_H
