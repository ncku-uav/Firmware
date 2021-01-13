
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "teensy.h"

I2CSPIDriverBase *TEENSY::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	TEENSY *instance = new TEENSY(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address/*,
				      cli.custom2*/);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (cli.custom1 == 1) {
		if (instance->force_init() != PX4_OK) {
			PX4_INFO("Failed to init TEENSY on bus %d, but will try again periodically.", iterator.bus());
		}

	} else if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

void
TEENSY::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for TEENSY

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("teensy", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0xac);
	PRINT_MODULE_USAGE_PARAM_FLAG('k', "if initialization (probing) fails, keep retrying periodically", true);
	// PRINT_MODULE_USAGE_PARAM_INT('t', 1, 1, 2, "battery index for calibration values (1 or 2)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int
teensy_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = TEENSY;
	BusCLIArguments cli{true, false};
	cli.i2c_address = TEENSY_BASEADDR;
	cli.default_i2c_frequency = 100000;
	cli.custom2 = 1;

	while ((ch = cli.getopt(argc, argv, "kt:")) != EOF) {
		switch (ch) {
		case 'k': // keep retrying
			cli.custom1 = 1;
			break;

		// case 't': // battery index
		// 	cli.custom2 = (int)strtol(cli.optarg(), NULL, 0);
		// 	break;
		}
	}

	const char *verb = cli.optarg();
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MSG_DEVTYPE_TEENSY);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
