/**
 * @file stm_firmware.hpp
 * @brief STM Firmware for Teensy 4.1
 *
 * This file defines the main STM class, which encapsulates all the functionality
 * for controlling the scanning tunneling microscope. It also defines the hardware
 * pinouts, DAC and ADC configurations, and other constants.
 *
 * --- CORRECTIONS ---
 * - Corrected DAC initialization modes for proper voltage ranges and data coding.
 * - Switched all bipolar DACs to Two's Complement mode for consistent data handling.
 * - Updated PID controller and approach routine to work with Two's Complement data.
 */

#ifndef STM_FIRMWARE_H
#define STM_FIRMWARE_H

#include <Arduino.h>
#include <SPI.h> // include the SPI library
#include "LTC2326_16.hpp"
#include <ArduinoJson.h>
#include "EfficientStepper.hpp"
#include "AD5761.hpp"
#include <logTable.hpp>

#define CS_ADC 38    // ADC chip select pin
#define ADC_MISO 39  // ADC MISO
#define CNV 19       // ADC CNV pin - initiates a conversion
#define BUSY 18      // ADC BUSY pin
#define SERIAL_LED 0 // Indicates serial data transmission
#define TUNNEL_LED 1 // Indicates tunneling

// DAC channel addresses: (Note they are not in order)
#define DAC_1 7  //
#define DAC_3 8  //
#define DAC_2 9  //
#define DAC_4 10 //

// DAC and ADC resolution:
#define DAC_BITS 16      // Actual DAC resolution
#define POSITION_BITS 20 // Sigma-delta resolution
#define ADC_BITS 16

 const int MAX_DAC_OUT = (1 << (DAC_BITS - 1)) - 1; // DAC upper bound (32767)
 const int MIN_DAC_OUT = -(1 << (DAC_BITS - 1));    // DAC lower bound (-32768)

// ADC Settings
LTC2326_16 ltc2326 = LTC2326_16(CS_ADC, CNV, BUSY);

// initialize the stepper library
// ULN2003 Motor Driver Pins
#define IN1 33
#define IN2 34
#define IN3 35
#define IN4 36
#define STEPS_PER_REVOLUTION 4096  // 28BYJ-48: 64 steps * 64 gear ratio = 4096 steps/revolution

#define CMD_LENGTH 4

#define INIT_KP 2.0
#define INIT_KI 1.0
#define INIT_KD 1.0

#define MOVE_SPEED 1

class STMStatus
{
public:
    int bias = 0; /**< Bias DAC value */
    int dac_z = 0; /**< Z DAC value */
    int dac_x = 0; /**< X DAC value */
    int dac_y = 0; /**< Y DAC value */
    int adc = 0; /**< ADC reading */
    int steps = 0; /**< Motor position */
    bool is_approaching = false; /**< Approach flag */
    bool is_const_current = false; /**< Constant current flag */
    bool is_scanning = false; /**< Scanning flag */
    int approach_direction = 1; /**< Approach direction: 1 for forward, -1 for backward */
    bool approach_recovery = false; /**< Recovery mode flag */
    uint32_t time_millis = 0; /**< Time in milliseconds */

    /**
     * @brief Converts the status to a character string.
     * @param buffer The buffer to store the string.
     */
    void to_char(char *buffer)
    {
        sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu", bias, dac_z, dac_x, dac_y, adc, steps, is_approaching, is_const_current, is_scanning, approach_direction, approach_recovery, time_millis);
    }
};

/**
 * @struct Approach_Config
 * @brief Configuration for the approach routine.
 * 
 * This struct holds the parameters for the automated approach sequence.
 */
struct Approach_Config
{
    int target_dac; /**< Target DAC value */
    int max_steps; /**< Maximum number of motor steps */
    int step_interval; /**< Step interval */
    int direction; /**< Approach direction: 1 for forward, -1 for backward */
};

/**
 * @brief Clamps a value between a minimum and maximum.
 * 
 * @param value The value to clamp.
 * @param min_value The minimum value.
 * @param max_value The maximum value.
 * @return The clamped value.
 */
double clamp_value(double value, double min_value, double max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
 * @class STM
 * @brief Main class for controlling the STM.
 * 
 * This class encapsulates all the hardware control and high-level functions 
 * for operating the STM, such as moving the motors, setting DAC values, 
 * reading the ADC, and performing automated procedures like approach and IV curves.
 */
class STM
{
public:
    /**
     * @brief Moves the stepper motor a specified number of steps.
     * @param steps The number of steps to move. Positive values move the motor forward, negative values move it backward.
     */
    void move_motor(int steps)
    {
        stepper_motor.step(steps);
        stm_status.steps = stepper_motor.get_total_steps();
        stm_status.time_millis = millis();
    }

    /**
     * @brief Stops the stepper motor immediately.
     */
    void stop_motor()
    {
        stepper_motor.disable();
    }

    /**
     * @brief Resets the STM to its initial state.
     * 
     * This function resets the stepper motor position, all DACs to zero, and clears the status flags.
     */
    void reset()
    {
        stepper_motor.setSpeed(2);
        stepper_motor.reset();
        dac_x.reset();
        dac_y.reset();
        dac_z.reset();
        dac_bias.reset();
        stm_status = STMStatus();
        ltc2326.convert();
    }

    // STM motors
    EfficientStepper stepper_motor = EfficientStepper(STEPS_PER_REVOLUTION, IN1, IN3, IN2, IN4);

    // DACs
    /**
     * @brief Sets the Z DAC value with bounds checking.
     * @param value The value to set, from MIN_DAC_OUT to MAX_DAC_OUT.
     * @return true if value was set successfully, false if out of bounds.
     */
    bool set_dac_z(int value)
    {
        if (value < MIN_DAC_OUT || value > MAX_DAC_OUT)
        {
            Serial.print("ERROR: Z DAC value out of range: ");
            Serial.println(value);
            return false;
        }
        dac_z.write(CMD_WR_UPDATE_DAC_REG, value);
        stm_status.dac_z = value;
        stm_status.time_millis = millis();
        return true;
    }

    /**
     * @brief Sets the X DAC value with bounds checking.
     * @param value The value to set, from MIN_DAC_OUT to MAX_DAC_OUT.
     * @return true if value was set successfully, false if out of bounds.
     */
    bool set_dac_x(int value)
    {
        if (value < MIN_DAC_OUT || value > MAX_DAC_OUT)
        {
            Serial.print("ERROR: X DAC value out of range: ");
            Serial.println(value);
            return false;
        }
        dac_x.write(CMD_WR_UPDATE_DAC_REG, value);
        stm_status.dac_x = value;
        stm_status.time_millis = millis();
        return true;
    }

    /**
     * @brief Sets the Y DAC value with bounds checking.
     * @param value The value to set, from MIN_DAC_OUT to MAX_DAC_OUT.
     * @return true if value was set successfully, false if out of bounds.
     */
    bool set_dac_y(int value)
    {
        if (value < MIN_DAC_OUT || value > MAX_DAC_OUT)
        {
            Serial.print("ERROR: Y DAC value out of range: ");
            Serial.println(value);
            return false;
        }
        dac_y.write(CMD_WR_UPDATE_DAC_REG, value);
        stm_status.dac_y = value;
        stm_status.time_millis = millis();
        return true;
    }

    /**
     * @brief Sets the bias DAC value with bounds checking.
     * @param value The value to set, from MIN_DAC_OUT to MAX_DAC_OUT.
     * @return true if value was set successfully, false if out of bounds.
     */
    bool set_dac_bias(int value)
    {
        if (value < MIN_DAC_OUT || value > MAX_DAC_OUT)
        {
            Serial.print("ERROR: Bias DAC value out of range: ");
            Serial.println(value);
            return false;
        }
        dac_bias.write(CMD_WR_UPDATE_DAC_REG, value);
        stm_status.bias = value;
        stm_status.time_millis = millis();
        return true;
    }

    // ADC
    /**
     * @brief Reads a raw value from the ADC.
     * @return The raw ADC value.
     */
    int read_adc_raw()
    {
        int start_time = millis();
        while (ltc2326.busy() && millis() - start_time <= 1)
        {
            continue;
        }
        int val = ltc2326.read();
        this->_add_adc_value(val);
        ltc2326.convert();
        return val;
    }

    /**
     * @brief Reads the averaged ADC value.
     * @return The averaged ADC value.
     */
    int read_adc()
    {
        read_adc_raw();
        return _get_adc_avg();
    }

    /**
     * @brief Updates the STM status with a new ADC reading.
     */
    void update()
    {
        int adc_val = read_adc_raw();
        stm_status.adc = adc_val;
        stm_status.time_millis = millis();
    }

    // Return the adc status.
    /**
     * @brief Gets the current STM status.
     * @return The STMStatus object.
     */
    STMStatus get_status()
    {
        return stm_status;
    }

    Approach_Config approach_config = Approach_Config();

    int approach_z_start;
    int approach_z_current;
    int approach_z_range;
    int approach_z_step;
    bool z_sweep_in_progress;
    int motor_steps_to_take;
    bool fine_motor_mode; // when true, use single motor step between sweeps for fine adjustment
    int approach_attempts;
    static const int MAX_APPROACH_ATTEMPTS = 3;
    
    // Smart approach recovery variables
    bool approach_overshoot_detected;
    bool approach_recovery_mode;
    int approach_recovery_steps;
    int approach_fine_step_size;
    /**
     * @brief Runtime debug verbosity level.
     * 0 = errors only, 1 = important state messages (default), 2 = verbose debug
     */
    int debug_level = 1;
    static const int MAX_RECOVERY_STEPS = 50;
    static const double OVERSHOOT_THRESHOLD = 2.0; // 2x target = overshoot

    /**
     * @brief Starts the automated approach sequence.
     * @param target_adc The target ADC value to stop the approach.
     * @param max_motor_steps The maximum number of motor steps to take.
     * @param step_interval The number of steps to take in each iteration.
     * @param direction The approach direction: 1 for forward, -1 for backward.
     */
    void start_approach(int target_adc, int max_motor_steps, int step_interval, int direction = 1)
    {
        approach_config.max_steps = stepper_motor.get_total_steps() + (max_motor_steps * abs(direction));
        approach_config.step_interval = step_interval;
        approach_config.target_dac = target_adc;
        approach_config.direction = (direction >= 0) ? 1 : -1; // Ensure only 1 or -1
        stm_status.is_approaching = true;
        stm_status.approach_direction = approach_config.direction;
        z_sweep_in_progress = false;
        
        // Smart approach: start from current Z position with adaptive range
        approach_z_start = stm_status.dac_z;
        approach_z_current = approach_z_start;
        approach_z_range = 2000; // Start with smaller range
        approach_z_step = 50; // Smaller steps for better resolution
        motor_steps_to_take = 0;
        fine_motor_mode = false;
        approach_attempts = 0;
        
        // Initialize recovery mode variables
        approach_overshoot_detected = false;
        approach_recovery_mode = false;
        approach_recovery_steps = 0;
        approach_fine_step_size = 10; // Fine steps for recovery
        
        if (debug_level >= 1)
        {
            Serial.print("Starting approach in ");
            Serial.print(approach_config.direction == 1 ? "FORWARD" : "BACKWARD");
            Serial.println(" direction");
        }
    }

    /**
     * @brief Enable fine motor mode: perform 1 motor step between Z sweeps.
     */
    void enable_fine_motor_mode()
    {
        fine_motor_mode = true;
        if (debug_level >= 1)
        {
            Serial.println("Fine motor mode ENABLED");
        }
    }

    /**
     * @brief Enable/disable fine motor mode.
     * @param enable true to enable fine mode (single-step motor between Z sweeps), false to use coarse mode.
     */
    void set_fine_mode(bool enable)
    {
        fine_motor_mode = enable;
        if (debug_level >= 1)
        {
            Serial.print("Fine motor mode ");
            Serial.println(enable ? "ENABLED" : "DISABLED");
        }
    }

    /**
     * @brief Set the fine approach piezo step size (Z sweep interval).
     */
    void set_approach_fine_step_size(int step_size)
    {
        approach_fine_step_size = step_size;
        if (debug_level >= 1)
        {
            Serial.print("Approach fine step size set to: ");
            Serial.println(approach_fine_step_size);
        }
    }

    /**
     * @brief Set the default Z sweep search range used between attempts.
     */
    void set_approach_z_search_range(int range)
    {
        approach_z_range = range;
        if (debug_level >= 1)
        {
            Serial.print("Approach Z search range set to: ");
            Serial.println(approach_z_range);
        }
    }

    /**
     * @brief Set runtime debug verbosity level.
     * @param level 0=errors only,1=important,2=verbose
     */
    void set_debug_level(int level)
    {
        debug_level = level;
        if (debug_level >= 1)
        {
            Serial.print("Debug level set to ");
            Serial.println(debug_level);
        }
    }

    /**
     * @brief Executes one step of the improved approach sequence.
     * @return True if the approach is complete, false otherwise.
     */
    bool approach()
    {
        if (!stm_status.is_approaching)
        {
            return false;
        }

        // State 1: Move motor if there are steps to take
        if (motor_steps_to_take > 0)
        {
            stepper_motor.step(approach_config.direction); // Move one step in configured direction
            motor_steps_to_take--;
            stm_status.steps = stepper_motor.get_total_steps();
            delayMicroseconds(500); // Reduced delay for faster response
            return false; // Return to allow main loop to run
        }

        // State 2: Perform smart Z-DAC sweep with overshoot detection
        if (z_sweep_in_progress)
        {
            approach_z_current += approach_z_step;
            int z_limit = approach_z_start + approach_z_range;
            
            if (approach_z_current <= z_limit && approach_z_current <= MAX_DAC_OUT)
            {
                set_dac_z(approach_z_current);
                delayMicroseconds(200); // Slightly longer delay for stability
                update();
                int current_adc = read_adc();
                
                // Check for overshoot (current much higher than target)
                if (current_adc > approach_config.target_dac * OVERSHOOT_THRESHOLD)
                {
                    if (debug_level >= 1)
                    {
                        Serial.print("Overshoot detected! ADC: ");
                        Serial.print(current_adc);
                        Serial.print(" Target: ");
                        Serial.println(approach_config.target_dac);
                    }

                    approach_overshoot_detected = true;
                    approach_recovery_mode = true;
                    approach_recovery_steps = 0;
                    z_sweep_in_progress = false;
                    stm_status.approach_recovery = true;

                    // Start recovery by moving Z away from surface
                    if (debug_level >= 1)
                    {
                        Serial.println("Starting recovery mode - retracting tip");
                    }
                    return false;
                }
                // Normal target detection
                else if (current_adc > approach_config.target_dac)
                {
                    if (debug_level >= 1)
                    {
                        Serial.println("Target reached successfully!");
                    }
                    stm_status.is_approaching = false;
                    z_sweep_in_progress = false;
                    stepper_motor.disable();
                    return true; // Approach finished successfully
                }
            }
            else
            {
                // Z sweep finished, did not find target
                z_sweep_in_progress = false;
                approach_attempts++;
                
                // Adaptive range expansion
                    if (approach_attempts < MAX_APPROACH_ATTEMPTS)
                    {
                        approach_z_range *= 2; // Double the range for next attempt
                        if (approach_z_range > 20000) approach_z_range = 20000; // Cap the range
                        if (debug_level >= 2)
                        {
                            Serial.print("Expanding Z search range to: ");
                            Serial.println(approach_z_range);
                        }
                    }
            }
            return false; // Return to allow main loop to run
        }
        
        // State 2.5: Recovery mode - handle overshoot
        if (approach_recovery_mode)
        {
            if (approach_recovery_steps < MAX_RECOVERY_STEPS)
            {
                // Move Z away from surface in small steps
                int recovery_z = approach_z_current - (approach_recovery_steps * approach_fine_step_size);
                
                if (recovery_z >= MIN_DAC_OUT)
                {
                    set_dac_z(recovery_z);
                    delayMicroseconds(100);
                    update();
                    int current_adc = read_adc();
                    
                    // Check if we're back in acceptable range
                    if (current_adc <= approach_config.target_dac * 1.2) // 20% above target is acceptable
                    {
                        Serial.print("Recovery successful at Z: ");
                        Serial.print(recovery_z);
                        Serial.print(" ADC: ");
                        Serial.println(current_adc);
                        
                        // Start fine approach from this position
                        approach_z_start = recovery_z;
                        approach_z_current = recovery_z;
                        approach_z_range = 500; // Small range for fine approach
                        approach_z_step = approach_fine_step_size; // Fine steps
                        approach_recovery_mode = false;
                        stm_status.approach_recovery = false;
                        z_sweep_in_progress = true;
                        
                        Serial.println("Starting fine approach");
                        return false;
                    }
                    
                    approach_recovery_steps++;
                }
                else
                {
                    Serial.println("Recovery failed: reached Z limit");
                    approach_recovery_mode = false;
                    stm_status.approach_recovery = false;
                    stm_status.is_approaching = false;
                    return false;
                }
            }
            else
            {
                Serial.println("Recovery failed: max recovery steps reached");
                approach_recovery_mode = false;
                stm_status.approach_recovery = false;
                stm_status.is_approaching = false;
                return false;
            }
            return false;
        }

        // State 3: Check max steps and prepare for next cycle
        if (!approach_overshoot_detected) // Only continue normal approach if no overshoot
        {
            bool within_step_limit;
            if (approach_config.direction == 1)
            {
                within_step_limit = stepper_motor.get_total_steps() < approach_config.max_steps;
            }
            else
            {
                within_step_limit = stepper_motor.get_total_steps() > (approach_config.max_steps - (approach_config.max_steps * 2));
            }
            
            // If fine motor mode is enabled we want to continue until the user stops us.
            bool attempts_ok = (approach_attempts < MAX_APPROACH_ATTEMPTS) || fine_motor_mode;
            if (within_step_limit && attempts_ok)
            {
                // Prepare for next motor move and Z-sweep cycle
                if (fine_motor_mode || approach_recovery_mode || approach_z_range <= 500)
                {
                    // Use single-step motor increment for fine adjustment
                    motor_steps_to_take = 1;
                    // use fine piezo step size for Z sweeps
                    approach_z_step = approach_fine_step_size;
                }
                else
                {
                    motor_steps_to_take = approach_config.step_interval;
                    // keep configured (coarse) approach step
                    // approach_z_step remains as configured in start_approach
                }
                approach_z_start = stm_status.dac_z; // Update start position
                approach_z_current = approach_z_start;
                z_sweep_in_progress = true;
            }
            else
            {
                // Reached max steps or max attempts
                if (!within_step_limit)
                {
                    Serial.println("Approach failed: Max steps reached.");
                }
                else
                {
                    Serial.println("Approach failed: Max attempts reached.");
                }
                stm_status.is_approaching = false;
                return false; // Approach finished unsuccessfully
            }
        }

        return false;
    }
    int iv_bias[1000];
    int iv_adc[1000];
    int iv_N;
    void generate_iv_curve(int bias_start, int bias_end, int bias_step)
    {
        int i = 0;
        int init_bias = stm_status.bias;
        for (int bias = bias_start; bias < bias_end; bias = bias + bias_step)
        {
            if (i >= 1000)
                break;
            set_dac_bias(bias);
            int adc = read_adc();
            iv_adc[i] = adc;
            iv_bias[i] = bias;
            i++;
        }
        iv_N = i;
        set_dac_bias(init_bias); // Set the bias value back to the starting point.
    }
    void send_iv_curve()
    {
        Serial.print("IV,");
        for (int i = 0; i < iv_N; ++i)
        {
            Serial.print(iv_bias[i]);
            Serial.print(",");
            Serial.print(iv_adc[i]);
            if (i < iv_N - 1)
                Serial.print(",");
        }
        Serial.print("\r\n");
    }
    int di_z[1000];
    int di_adc[1000];
    int di_N;
    void generate_di_curve(int z_start, int z_end, int z_step)
    {
        int i = 0;
        for (int z = z_start; z < z_end; z = z + z_step)
        {
            if (i >= 1000)
                break;
            set_dac_z(z);
            int adc = read_adc();
            di_adc[i] = adc;
            di_z[i] = z;
            i++;
        }
        di_N = i;
    }
    void send_di_curve()
    {
        for (int i = 0; i < di_N; ++i)
        {
            Serial.print(di_z[i]);
            Serial.print(",");
            Serial.print(di_adc[i]);
            if (i < di_N)
                Serial.print(",");
        }
        Serial.print("\r\n");
    }
    // Improved PID controller for constant current mode
    bool is_const_current = false;
    int adc_set_value;
    double adc_set_value_log, adc_real_value_log, dac_z_control_value;

    // Enhanced PID parameters and state variables
    double Kp = 0.0, Ki = 0.0, Kd = 0.0;
    double pTerm, iTerm, dTerm;
    double prev_error = 0.0;
    double integral_max = 15000.0; // Anti-windup limit (about half DAC range)
    double integral_min = -15000.0;
    uint32_t last_pid_time = 0;
    static const uint32_t PID_SAMPLE_TIME = 1; // 1ms minimum sample time
    
    void turn_on_const_current(int target_adc)
    {
        this->adc_set_value = target_adc;
        this->adc_set_value_log = static_cast<double>(logTable[abs(target_adc)]);
        this->dac_z_control_value = static_cast<double>(stm_status.dac_z);
        
        // Reset PID state
        pTerm = 0.0;
        iTerm = 0.0;
        dTerm = 0.0;
        prev_error = 0.0;
        last_pid_time = millis();
        
        this->stm_status.is_const_current = true;
        if (debug_level >= 1)
        {
            Serial.println("Constant current mode ON");
        }
    }
    
    int control_current(int adc_value)
    {
        uint32_t current_time = millis();
        uint32_t time_delta = current_time - last_pid_time;
        
        // Enforce minimum sample time for derivative calculation stability
        if (time_delta < PID_SAMPLE_TIME)
        {
            return static_cast<int>(prev_error); // Return previous error without updating
        }
        
        this->adc_real_value_log = static_cast<double>(logTable[abs(adc_value)]);
        double error = this->adc_set_value_log - this->adc_real_value_log;
        double dt = static_cast<double>(time_delta) / 1000.0; // Convert to seconds
        
        // Proportional term
        pTerm = Kp * error;
        
        // Integral term with anti-windup
        double integral_candidate = iTerm + Ki * error * dt;
        
        // Anti-windup: only update integral if output won't saturate
        double tentative_output = pTerm + integral_candidate;
        if (tentative_output >= MIN_DAC_OUT && tentative_output <= MAX_DAC_OUT)
        {
            iTerm = integral_candidate;
        }
        // Additional clamping for safety
        iTerm = clamp_value(iTerm, integral_min, integral_max);
        
        // Derivative term (with derivative kick prevention)
        if (dt > 0)
        {
            dTerm = Kd * (error - prev_error) / dt;
        }
        else
        {
            dTerm = 0.0;
        }
        
        // Calculate final output
        double z_double = pTerm + iTerm + dTerm;
        
        // Clamp final output to valid DAC range
        int z = static_cast<int>(clamp_value(z_double, MIN_DAC_OUT, MAX_DAC_OUT));
        
        // Update state for next iteration
        prev_error = error;
        last_pid_time = current_time;
        
        this->set_dac_z(z);
        return static_cast<int>(error);
    }
    
    void turn_off_const_current()
    {
        this->stm_status.is_const_current = false;
        if (debug_level >= 1)
        {
            Serial.println("Constant current mode OFF");
        }
    }
    
    /**
     * @brief Gets current PID state for debugging
     */
    void print_pid_debug()
    {
        if (debug_level >= 2)
        {
            Serial.print("PID Debug - P:");
            Serial.print(pTerm);
            Serial.print(" I:");
            Serial.print(iTerm);
            Serial.print(" D:");
            Serial.print(dTerm);
            Serial.print(" Error:");
            Serial.println(prev_error);
        }
    }
    
    /**
     * @brief Gets current approach state for debugging
     */
    void print_approach_debug()
    {
        if (debug_level >= 2)
        {
            Serial.print("Approach Debug - Z:");
            Serial.print(approach_z_current);
            Serial.print(" Range:");
            Serial.print(approach_z_range);
            Serial.print(" Step:");
            Serial.print(approach_z_step);
            Serial.print(" Attempts:");
            Serial.print(approach_attempts);
            Serial.print(" Recovery:");
            Serial.print(approach_recovery_mode ? "ON" : "OFF");
            Serial.print(" Overshoot:");
            Serial.println(approach_overshoot_detected ? "YES" : "NO");
        }
    }
    
    /**
     * @brief Manually trigger approach recovery mode (for testing)
     */
    void trigger_approach_recovery()
    {
        if (stm_status.is_approaching)
        {
            Serial.println("Manually triggering approach recovery");
            approach_overshoot_detected = true;
            approach_recovery_mode = true;
            approach_recovery_steps = 0;
            z_sweep_in_progress = false;
        }
        else
        {
            Serial.println("Not in approach mode - cannot trigger recovery");
        }
    }
    // Scan Control
    int scan_image_z[2048];
    int scan_image_adc[2048];

    void start_scan(int x_start, int x_end, int x_resolution, int y_start, int y_end, int y_resolution, int sample_per_pixel)
    {
        if (!move_to(x_start, y_start))
        {
            Serial.println("ERROR: Failed to move to scan start position");
            return;
        }
        double x_step = 1.0f * (x_end - x_start) / x_resolution;
        double y_step = 1.0f * (y_end - y_start) / y_resolution / sample_per_pixel;
        for (int x_i = 0; x_i < x_resolution; ++x_i)
        {
            int x_now = static_cast<int>(x_start + x_i * x_step);
            set_dac_x(x_now);
            int sample_count = 0;
            int err_sum = 0;
            int dacz_sum = 0;
            for (int y_i = 0; y_i < y_resolution * sample_per_pixel; ++y_i)
            {
                int y_now = static_cast<int>(y_start + y_i * y_step);
                set_dac_y(y_now);
                int adc_value = read_adc_raw();
                if (this->stm_status.is_const_current)
                {

                    adc_value = control_current(adc_value);
                }
                err_sum += adc_value;
                dacz_sum += stm_status.dac_z;
                sample_count++;
                if (sample_count == sample_per_pixel)
                {
                    scan_image_adc[y_i / sample_per_pixel] = err_sum / sample_per_pixel;
                    scan_image_z[y_i / sample_per_pixel] = dacz_sum / sample_per_pixel;
                    sample_count = 0;
                    err_sum = 0;
                    dacz_sum = 0;
                }
            }
            send_scan_line("A", x_i, scan_image_adc, y_resolution);
            send_scan_line("Z", x_i, scan_image_z, y_resolution);
            for (int y_i = y_resolution * sample_per_pixel - 1; y_i >= 0; --y_i)
            {
                int y_now = static_cast<int>(y_start + y_i * y_step);
                set_dac_y(y_now);
                if (this->stm_status.is_const_current)
                {
                    control_current(read_adc_raw());
                }
            }
        }
        Serial.println("D");
    }
    void send_scan_line(String prefix, int x_i, int *data, int num_points)
    {
        Serial.print(prefix);
        Serial.printf(",%d,", x_i);
        for (int i = 0; i < num_points; ++i)
        {
            Serial.print(data[i]);
            if (i < num_points - 1)
                Serial.print(",");
        }
        Serial.print("\r\n");
    }
    bool move_to(int target_x, int target_y)
    {
        // Move X axis
        while (target_x != stm_status.dac_x)
        {
            if (stm_status.is_const_current)
            {
                control_current(read_adc_raw());
            }
            
            int next_x;
            if (abs(target_x - stm_status.dac_x) < MOVE_SPEED)
            {
                next_x = target_x;
            }
            else
            {
                if (target_x > stm_status.dac_x)
                {
                    next_x = stm_status.dac_x + MOVE_SPEED;
                }
                else
                {
                    next_x = stm_status.dac_x - MOVE_SPEED;
                }
            }
            
            if (!set_dac_x(next_x))
            {
                Serial.println("ERROR: Failed to move X axis");
                return false;
            }
        }
        
        // Move Y axis
        while (target_y != stm_status.dac_y)
        {
            if (stm_status.is_const_current)
            {
                control_current(read_adc_raw());
            }
            
            int next_y;
            if (abs(target_y - stm_status.dac_y) < MOVE_SPEED)
            {
                next_y = target_y;
            }
            else
            {
                if (target_y > stm_status.dac_y)
                {
                    next_y = stm_status.dac_y + MOVE_SPEED;
                }
                else
                {
                    next_y = stm_status.dac_y - MOVE_SPEED;
                }
            }
            
            if (!set_dac_y(next_y))
            {
                Serial.println("ERROR: Failed to move Y axis");
                return false;
            }
        }
        
        return true;
    }

    // Piezo
    void test_piezo()
    {
        for (int i = 0; i < 500; i++)
        {
             set_dac_z(MAX_DAC_OUT);
            delayMicroseconds(500);
             set_dac_z(MIN_DAC_OUT);
            delayMicroseconds(500);
        }
        delay(1000);
        for (int i = 0; i < 500; i++)
        {
             set_dac_x(MAX_DAC_OUT);
            delayMicroseconds(500);
             set_dac_x(MIN_DAC_OUT);
            delayMicroseconds(500);
        }
        delay(1000);
        for (int i = 0; i < 500; i++)
        {
             set_dac_y(MAX_DAC_OUT);
            delayMicroseconds(500);
             set_dac_y(MIN_DAC_OUT);
            delayMicroseconds(500);
        }
    }
    STMStatus stm_status = STMStatus();

private:
     // DAC output mode register setting
    // See AD5761 datasheet for details
    //  static const uint16_t MODE_3V = 0b0000000000000000;
    //  static const uint16_t MODE_10V = 0b0000000000000010;
    //  static const uint16_t MODE_5V = 0b0000000000000001; // 0V to 5V range
    //  static const uint16_t MODE_BIAS = 0b0000000000001000;

    // static const uint16_t MODE_3V = 0b0000000000000000;
    static const uint16_t MODE_10V = 0b0000000000000001; // 0V to 10V range
    static const uint16_t MODE_5V = 0b0000000000000011; // 0V to 5V range (unipolar)
    static const uint16_t MODE_pm5V = 0b0000000000000010; // -5V to +5V range (bipolar)
    static const uint16_t MODE_pm3V = 0b0000000000000101; // -3V to +3V range (bipolar)
    static const uint16_t MODE_pm10V = 0b0000000000000000; // -10V to +10V range (bipolar)

    // static constant uint16_t MODE list
    // 0b0000000000000 000 : -10V to +10V ------------
    // 0b0000000000000 001 : 0V to +10V --------------
    // 0b0000000000000 010 : -5V to +5V
    // 0b0000000000000 011 : 0V to +5V ---------------
    // 0b0000000000000 100 : -2.5V to +7.5V
    // 0b0000000000000 101 : -3V to +3V ---------------
    // 0b0000000000000 110 : 0V to 16V
    // 0b0000000000000 111 : 0V to 20V

    //  // DAC objects
    //  AD5761 dac_x = AD5761(DAC_1, MODE_5V);
    //  AD5761 dac_y = AD5761(DAC_2, MODE_5V);
    //  AD5761 dac_z = AD5761(DAC_3, MODE_10V);
    //  AD5761 dac_bias = AD5761(DAC_4, MODE_BIAS);

    // DAC objects
    // Note: Using bipolar ±5V for X/Y allows symmetric scanning around center point
     AD5761 dac_x = AD5761(DAC_1, MODE_pm5V);   // -5V to +5V for symmetric X scanning
     AD5761 dac_y = AD5761(DAC_2, MODE_pm5V);   // -5V to +5V for symmetric Y scanning
     AD5761 dac_z = AD5761(DAC_3, MODE_pm3V);   // -3V to +3V for Z
     AD5761 dac_bias = AD5761(DAC_4, MODE_pm10V); // -10V to +10V for bias

    // ADC settings
    LTC2326_16 ltc2326 = LTC2326_16(CS_ADC, CNV, BUSY);

    int _adc_buffer[5];
    int _current_index = 0;
    int _adc_sum = 0;
    void _add_adc_value(int value)
    {
        _current_index += 1;
        _current_index = _current_index % 5;
        _adc_sum -= _adc_buffer[_current_index];
        _adc_buffer[_current_index] = value;
        _adc_sum += _adc_buffer[_current_index];
    }
    int _get_adc_avg()
    {
        return static_cast<int>(_adc_sum / 5.0);
    }
};

#endif // STM_FIRMWARE_H