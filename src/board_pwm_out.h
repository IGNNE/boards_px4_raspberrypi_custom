/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <cstring> // for memcpy

#include <mutex>

#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform/pwm_out_base.h>


/**
 * @file Hacky RasPi driver to go with rc-aux-controller
 */

#define BOARD_PWM_OUT_IMPL PWMCustomAvr

namespace pwm_out
{

class PWMCustomAvr : public device::I2C, public PWMOutBase, public px4::ScheduledWorkItem
{
public:

    /**
     *  uses default values
     * @todo maybe change this?
     */
    PWMCustomAvr(int num_outputs);

    /**
     * Will set up to ::NUM_PWM_CHANNELS outputs to values of 0-4095,
     * overrides PWMOutBase method
     */
    int send_output_pwm(const uint16_t *pwm, int num_outputs) override;

    int init() override;

    /**
    * Last channel is the integrated ESC
    */
    constexpr static uint8_t NUM_PWM_CHANNELS = 4;

    /**
     * I2C bus to use
     * @todo: this should be configurable
     */
    constexpr static uint8_t I2C_BUS_NUM = 3;

private:
    /**
     * Default address for the avr mcu, no deeper reason, set your own if you want
     * This needs to be in the public part so the main function can access it
     */
    const static uint8_t I2C_DEFAULT_ADDRESS = 0x4;

    /**
     * First pwm channel
     */
    const static uint8_t FIRST_PWM_ADDR = 2;

    /**
     * Since the output is not written directly, save the received output values
     */
    uint16_t current_outputs[NUM_PWM_CHANNELS];

    /**
     * Mutex to synchronize access to the array
     */
    std::mutex current_output_mutex;

    /**
     * Number of outputs that will be used, 0-NUM_PWM_CHANNELS
     */
    uint8_t num_outputs_used = 0;

    /**
     * This is the method that will actually write the values over I2C!
     * It has to be done via a workqueue, to avoid blocking the bus.
     */
    void Run() override;

    /**
     * Set PWM value for a channel (0-(NUM_PWM_CHANNELS-1)).
     * value should be range of 0-4095, see ::Run() for why this is not used directly
     */
    int set_pwm_i2c(uint8_t channel, uint16_t value);




};

} // namespace linux_pwm_out
