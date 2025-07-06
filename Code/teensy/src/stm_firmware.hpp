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
#define STEPS_PER_REVOLUTION 2048

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
    uint32_t time_millis = 0; /**< Time in milliseconds */

    /**
     * @brief Converts the status to a character string.
     * @param buffer The buffer to store the string.
     */
    void to_char(char *buffer)
    {
        sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu", bias, dac_z, dac_x, dac_y, adc, steps, is_approaching, is_const_current, is_scanning, time_millis);
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
     * @brief Sets the Z DAC value.
     * @param value The value to set, from MIN_DAC_OUT to MAX_DAC_OUT.
     */
    void set_dac_z(int value)
    {
        dac_z.write(CMD_WR_UPDATE_DAC_REG, value);
        stm_status.dac_z = value;
        stm_status.time_millis = millis();
    }

    /**
     * @brief Sets the X DAC value.
     * @param value The value to set, from MIN_DAC_OUT to MAX_DAC_OUT.
     */
    void set_dac_x(int value)
    {
        dac_x.write(CMD_WR_UPDATE_DAC_REG, value);
        stm_status.dac_x = value;
        stm_status.time_millis = millis();
    }

    /**
     * @brief Sets the Y DAC value.
     * @param value The value to set, from MIN_DAC_OUT to MAX_DAC_OUT.
     */
    void set_dac_y(int value)
    {
        dac_y.write(CMD_WR_UPDATE_DAC_REG, value);
        stm_status.dac_y = value;
        stm_status.time_millis = millis();
    }

    /**
     * @brief Sets the bias DAC value.
     * @param value The value to set, from MIN_DAC_OUT to MAX_DAC_OUT.
     */
    void set_dac_bias(int value)
    {
        dac_bias.write(CMD_WR_UPDATE_DAC_REG, value);
        stm_status.bias = value;
        stm_status.time_millis = millis();
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

    int approach_z_value;
    bool z_sweep_in_progress;

    /**
     * @brief Starts the automated approach sequence.
     * @param target_adc The target ADC value to stop the approach.
     * @param max_motor_steps The maximum number of motor steps to take.
     * @param step_interval The number of steps to take in each iteration.
     */
    void start_approach(int target_adc, int max_motor_steps, int step_interval)
    {
        approach_config.max_steps = stepper_motor.get_total_steps() + max_motor_steps;
        approach_config.step_interval = step_interval;
        approach_config.target_dac = target_adc;
        stm_status.is_approaching = true;
        z_sweep_in_progress = false;
        approach_z_value = -23000;
    }

    /**
     * @brief Executes one step of the approach sequence.
     * @return True if the approach is complete, false otherwise.
     */
    bool approach()
    {
        if (stm_status.is_approaching)
        {
            if (z_sweep_in_progress)
            {
                approach_z_value += 100; // Increment Z DAC value
                if (approach_z_value <= 17000)
                {
                    set_dac_z(approach_z_value);
                    delayMicroseconds(100); // Short delay for DAC to settle
                    update();
                    if (read_adc() > approach_config.target_dac)
                    {
                        Serial.println("Approached!");
                        Serial.println(stm_status.adc);
                        stm_status.is_approaching = false;
                        z_sweep_in_progress = false;
                        stepper_motor.disable();
                        return true;
                    }
                }
                else
                {
                    // Z sweep finished, did not find target
                    z_sweep_in_progress = false;
                }
            }
            else
            {
                // Z sweep not in progress, so let's move the motor
                if (stepper_motor.get_total_steps() < approach_config.max_steps)
                {
                    move_motor(approach_config.step_interval);
                    delay(2); // delay for motor to settle
                    approach_z_value = -23000; // Reset Z DAC for new sweep
                    z_sweep_in_progress = true; // Start sweep
                }
                else
                {
                    // Reached max steps
                    stm_status.is_approaching = false;
                    return false;
                }
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
    // Specify the links and initial tuning parameters
    // Constant current mode
    bool is_const_current = false;
    int adc_set_value;
    double adc_set_value_log, adc_real_value_log, dac_z_control_value;

    // PID current_pid = PID(&adc_real_value_log_log, &dac_z_control_value, &adc_set_value_log_log, INIT_KP, INIT_KI, INIT_KD, DIRECT);
    double Kp = 0.0, Ki = 0.0, Kd = 0.0;
    double pTerm, iTerm;
    void turn_on_const_current(int target_adc)
    {
        this->adc_set_value = target_adc;
        this->adc_set_value_log = static_cast<double>(logTable[abs(target_adc)]);
        this->dac_z_control_value = static_cast<double>(stm_status.dac_z);
        pTerm = 0.0;
        iTerm = 0.0;
        this->stm_status.is_const_current = true;
    }
    int control_current(int adc_value)
    {
        this->adc_real_value_log = static_cast<double>(logTable[abs(adc_value)]);
        double error = this->adc_set_value_log - this->adc_real_value_log;
        pTerm = Kp * error;
        iTerm += Ki * error;
         
         // BUG FIX 5: Clamp integrator term to the full DAC output range
         iTerm = clamp_value(iTerm, MIN_DAC_OUT, MAX_DAC_OUT);
 
         // BUG FIX 5: Remove "+ 32768" offset, as we are now in Two's Complement mode.
         double z_double = pTerm + iTerm;
         
         // Clamp final output to the valid 16-bit signed range.
         int z = static_cast<int>(clamp_value(z_double, MIN_DAC_OUT, MAX_DAC_OUT));
 
        this->set_dac_z(z);
        return static_cast<int>(error);
    }
    void turn_off_const_current()
    {
        this->stm_status.is_const_current = false;
    }
    // Scan Control
    int scan_image_z[2048];
    int scan_image_adc[2048];

    void start_scan(int x_start, int x_end, int x_resolution, int y_start, int y_end, int y_resolution, int sample_per_pixel)
    {
        move_to(x_start, y_start);
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
    void move_to(int target_x, int target_y)
    {
        while (target_x != stm_status.dac_x)
        {
            if (stm_status.is_const_current)
            {
                control_current(read_adc_raw());
            }
            if (abs(target_x - stm_status.dac_x) < MOVE_SPEED)
            {
                set_dac_x(target_x);
            }
            else
            {
                if (target_x > stm_status.dac_x)
                {
                    set_dac_x(stm_status.dac_x + MOVE_SPEED);
                }
                else
                {
                    set_dac_x(stm_status.dac_x - MOVE_SPEED);
                }
            }
        }
        while (target_y != stm_status.dac_y)
        {
            if (stm_status.is_const_current)
            {
                control_current(read_adc_raw());
            }
            if (abs(target_y - stm_status.dac_y) < MOVE_SPEED)
            {
                set_dac_y(target_y);
            }
            else
            {
                if (target_y > stm_status.dac_y)
                {
                    set_dac_y(stm_status.dac_y + MOVE_SPEED);
                }
                else
                {
                    set_dac_y(stm_status.dac_y - MOVE_SPEED);
                }
            }
        }
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
     // --- BUG FIX 2 & 3: Correct DAC initialization modes ---
     // All bipolar ranges are now configured for Two's Complement data coding (B2C Bit = 1).
     // This ensures that the signed integer values used in the firmware are correctly interpreted by the DAC.
     // The RA[2:0] bits are set according to the desired voltage ranges from Table 12.
 
     // Output range: -5V to +5V (RA=010), Two's Complement (B2C=1)
     static const uint16_t MODE_X = 0b0000000001000010;
     // Output range: -5V to +5V (RA=010), Two's Complement (B2C=1)
     static const uint16_t MODE_Y = 0b0000000001000010;
     // Output range: -10V to +10V (RA=000), Two's Complement (B2C=1)
     static const uint16_t MODE_Z = 0b0000000001000000;
     // Output range: 0V to +3V (RA=110), Straight Binary (B2C=0)
     static const uint16_t MODE_BIAS = 0b0000000000000110;
 
    // DAC Settings
     AD5761 dac_x = AD5761(DAC_1, MODE_X);
     AD5761 dac_y = AD5761(DAC_2, MODE_Y);
     AD5761 dac_z = AD5761(DAC_3, MODE_Z);
     AD5761 dac_bias = AD5761(DAC_4, MODE_BIAS);

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