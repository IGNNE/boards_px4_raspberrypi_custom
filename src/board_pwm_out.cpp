/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

#include "board_pwm_out.h"

using namespace pwm_out;


PWMCustomAvr::PWMCustomAvr(int num_outputs) :
	I2C(DRV_PWM_CUSTOM_AVR, MODULE_NAME, I2C_BUS_NUM, I2C_DEFAULT_ADDRESS, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::I2C3)
{
	if (num_outputs > NUM_PWM_CHANNELS) {
		PX4_WARN("Tried to init more channels than coprocessor has, tried: %i, available: %i", num_outputs, NUM_PWM_CHANNELS);
		num_outputs_used = NUM_PWM_CHANNELS;

	} else {
		num_outputs_used = num_outputs;
	}
}

int PWMCustomAvr::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	if (num_outputs > num_outputs_used) {
		PX4_DEBUG("Tried to set more channels than specified has");
		num_outputs = num_outputs_used;
	}

	// save input for the workitem to write
	current_output_mutex.lock();
	memcpy(current_outputs, pwm, num_outputs * sizeof(current_outputs[0]));
	current_output_mutex.unlock();

	return PX4_OK;
}

int PWMCustomAvr::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C::init failed: (%i)", ret);
	}


	uint8_t id_address = 0;
	uint8_t id_value = 0;

	// for some reason, my avr needs two separate tranfers
	// first write the "register"/key, then read the date
	ret = transfer(&id_address, 1, nullptr, 0);
	ret = transfer(nullptr, 0, &id_value, 1);

	if (ret != PX4_OK || id_value != I2C_DEFAULT_ADDRESS) {
		// TODO: change back to DEVICE_DEBUG?
		PX4_WARN("Controller does not respond, address returned %i, should be %i", id_value, I2C_DEFAULT_ADDRESS);
	}

	// 100 Hz interval should be enough, add a delay to avoid interfering with other drivers
	ScheduleOnInterval(10 * 1000, 200 * 1000);

	return ret;
}


int PWMCustomAvr::set_pwm_i2c(uint8_t channel, uint16_t value)
{
	if (value >= 4096) {
		PX4_WARN("invalid pwm value");
		return PX4_ERROR;
	}

	int ret = 0;

#ifdef MOTOR_PWM

	if (channel >= (NUM_PWM_CHANNELS - 1)) {
		// last channel is the ESC
		// the ESC takes values of 0-255, so scale our 1000-2000 down, also remove the 900 "save-position"
		// from the servo logic, 180 is the max value with the current power supply
		// TODO: do some proper rounding here?
		value = round((value - 900) / 1100.0 * 180);
		uint8_t buffer[2];
		buffer[0] = FIRST_PWM_ADDR + channel * 2; // first byte is the "commmand"
		buffer[1] = value; // one byte payload

		ret = transfer(buffer, 2, nullptr, 0);

	} else
#endif // MOTOR_PWM
	{
		// servo PWM, write two-byte value
		uint8_t buffer[2];
		// TODO: for now (25.03.21) the aux controller has a weird bug when writing more than one value,
		// so I'll just work around it by dividing it into two transactions
		uint8_t *value_8b_ptr = (uint8_t *) &value;
		buffer[0] = FIRST_PWM_ADDR + channel * 2; // first byte is the "commmand"
		buffer[1] = *(value_8b_ptr + 1); // first byte of payload, MSB
		ret = transfer(buffer, 2, nullptr, 0);
		buffer[0]++; // increase register pointer
		buffer[1] = *value_8b_ptr; // second byte of payload, LSB
		ret = transfer(buffer, 2, nullptr, 0);
	}

	if (OK != ret) {
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}


void PWMCustomAvr::Run()
{
	for (int i = 0; i < num_outputs_used; i++) {
		current_output_mutex.lock();
		set_pwm_i2c(i, current_outputs[i]);
		current_output_mutex.unlock();
	}
}



