/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

/**
 * @file teensy.cpp
 * @author NCKU UAV
 *
 * Driver for the I2C attached TEENSY
 */
#include "teensy.h"


TEENSY::TEENSY(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address/*, int battery_index*/) :
	I2C(DRV_MSG_DEVTYPE_TEENSY, MODULE_NAME, bus, address, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address),
	_sample_perf(perf_alloc(PC_ELAPSED, "teensy_read")),
	_comms_errors(perf_alloc(PC_COUNT, "teensy_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "teensy_collection_err"))/*,
	_measure_errors(perf_alloc(PC_COUNT, "teensy_measurement_err")),
	_battery(battery_index, this, INA226_SAMPLE_INTERVAL_US)*/
{

}

TEENSY::~TEENSY()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	// perf_free(_measure_errors);
}
/*
int TEENSY::read(uint8_t address, uint32_t &data)
{
	// PX4_INFO("Teensy Start Read\n");
	// read desired little-endian value via I2C
	uint32_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap32(received_bytes);
		// data = received_bytes;


	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		// PX4_INFO("Teensy read Error");
	}

	return ret;
}*/
int TEENSY::read(uint8_t address, uint16_t &data)
{
	// PX4_INFO("Teensy Start Read\n");
	// read desired little-endian value via I2C
	// union {
	// 	uint32_t reg;
	// 	uint8_t b[4]={};
	// }received_bytes;
	uint16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));
	// const int ret = transfer(&address, 1, (uint8_t *)&received_bytes.b[0], sizeof(received_bytes.b));

	if (ret == PX4_OK) {

		// data = swap32(received_bytes.reg);
		data = swap16(received_bytes);
		// data = received_bytes.reg;
		// data = received_bytes;
		// if(data != 0){
		// 	data = 999;
		// }


	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		// PX4_INFO("Teensy read Error");
		data=444;
	}

	return ret;
}

int TEENSY::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)}; // NCKU UAV: here should change in unit32_t
	return transfer(data, sizeof(data), nullptr, 0);
}



int
TEENSY::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}


	ret = PX4_OK;
	// PX4_INFO("Teensy init!");
	start();
	_teensy_ok = true;

	_initialized = ret == PX4_OK;
	return ret;
}

int
TEENSY::force_init()
{
	int ret = init();

	start();

	return ret;
}

int
TEENSY::probe()
{
	// int16_t value{0};

	// if (read(INA226_MFG_ID, value) != PX4_OK || value != INA226_MFG_ID_TI) {
	// 	PX4_DEBUG("probe mfgid %d", value);
	// 	return -1;
	// }

	// if (read(INA226_MFG_DIEID, value) != PX4_OK || value != INA226_MFG_DIE) {
	// 	PX4_DEBUG("probe die id %d", value);
	// 	return -1;
	// }

	return PX4_OK;
}

// int
// TEENSY::measure()
// {
// 	int ret = PX4_OK;


// 	return ret;
// }

int
TEENSY::collect()
{
	PX4_INFO("Teensy collect");
	perf_begin(_sample_perf);


	bool success{true};
	uint16_t AOA,AOS,AiL,AiR,HT,VT,RPM;
	success = success &&(read(TEENSY_REG_AOS,AOS) == PX4_OK);
	success = success &&(read(0x05,testData) == PX4_OK);
	success = success &&(read(TEENSY_REG_AOA,AOA) == PX4_OK);
	success = success &&(read(TEENSY_REG_AILERON_L,AiL) == PX4_OK);
	success = success &&(read(TEENSY_REG_AILERON_R,AiR) == PX4_OK);
	success = success &&(read(TEENSY_REG_HT,HT) == PX4_OK);
	success = success &&(read(TEENSY_REG_VT,VT) == PX4_OK);
	success = success &&(read(TEENSY_REG_RPM,RPM) == PX4_OK);
	if (!success) {
		PX4_DEBUG("error reading from TEENSY");
	}
	report.timestamp = hrt_absolute_time();
	report.output[0] = (float32) AOA/100;
	report.output[1] = (float32) AOS/100;
	report.output[2] = (float32) AiL/100;
	report.output[3] = (float32)AiR/100;
	report.output[4] =(float32)HT/100;
	report.output[5] = (float32)VT/100;
	report.output[6] = (float32) RPM;
	test.timestamp = hrt_absolute_time();
	test.val = (int32_t) testData;


	orb_publish(ORB_ID(Teensy), Teensy_pub, &report);
	orb_publish(ORB_ID(teensy_test),Teensy_pub_test,&test);

	perf_end(_sample_perf);

	if (success) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}

}

void
TEENSY::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	 _collect_phase = false;

	// _measure_interval = TEENSY_SAMPLE_INTERVAL_US;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void
TEENSY::RunImpl()
{
	PX4_INFO("Teensy Run!");
	if (_initialized) {
		if(_collect_phase){
		  	if (collect() != PX4_OK) {
				perf_count(_collection_errors);
				start();
				return;
			}
		}
		_collect_phase=true;
		ScheduleDelayed(TEENSY_SAMPLE_INTERVAL_US);

	} else {
		if (init() != PX4_OK) {
			ScheduleDelayed(TEENSY_INIT_RETRY_INTERVAL_US);
		}
	}
}

void
TEENSY::print_status()
{
	PX4_INFO("Teensy status");
	I2CSPIDriverBase::print_status();

	if (_initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		printf("poll interval:  %u \n", _measure_interval);

	} else {
		PX4_INFO("Device not initialized.");//,
		// 	 TEENSY_INIT_RETRY_INTERVAL_US / 1000);
	}
}
