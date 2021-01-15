#pragma once


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
// #include <uORB/topics/orb_test.h>
#include <px4_platform_common/module_params.h>

#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

/* Configuration Constants */
#define TEENSY_BASEADDR 	                    0xac /*NCKU UAV */  /* 7-bit address. 8-bit address is 0x1 */
// If initialization is forced (with the -f flag on the command line), but it fails, the drive will try again to
// connect to the TEENSY every this many microseconds
#define TEENSY_INIT_RETRY_INTERVAL_US			500000 //NCKU UAV

/* TEENSY Registers addresses*/

#define TEENSY_REG_AOS (0x07)
#define TEENSY_REG_AILERON_L (0x09)
#define TEENSY_REG_AILERON_R (0x0b)
#define TEENSY_REG_VT (0x0d)
#define TEENSY_REG_HT (0x0f)
#define TEENSY_REG_RPM (0x11)
#define TEENSY_REG_AOA (0x13)


#define TEENSY_SAMPLE_FREQUENCY_HZ            50	//NCKU UAV
#define TEENSY_SAMPLE_INTERVAL_US             (1_s / TEENSY_SAMPLE_FREQUENCY_HZ)

#define swap16(w)                       __builtin_bswap16((w)) //swap 16bites message

class TEENSY : public device::I2C, public ModuleParams, public I2CSPIDriver<TEENSY>
{
public:
	TEENSY(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address);
	virtual ~TEENSY();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance); //For I2C
	static void print_usage();

	void	RunImpl(); // main loop

	int 		  init() override; //initialize

	/**
	 * Tries to call the init() function. If it fails, then it will schedule to retry again in
	 * TEENSY_INIT_RETRY_INTERVAL_US microseconds. It will keep retrying at this interval until initialization succeeds.
	 *
	 * @return PX4_OK if initialization succeeded on the first try. Negative value otherwise.
	 */
	int force_init();  //fore initialize

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				      print_status() override;
	struct actuator_outputs_s report;
	// struct orb_test_s test ;

protected:
	int	  		probe() override;

private:
	/*for teensy status*/
	bool			        _teensy_ok{false};
	unsigned                        _measure_interval{0};
	bool			        _collect_phase{false};
	bool 					_initialized{false};

	/*event counters*/
	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t 		_collection_errors;



	/*I2C IO main function*/

	int read(uint8_t address, uint16_t &data);
	int write(uint8_t address, uint16_t data);

	/*uORB message*/
	orb_advert_t Teensy_pub = orb_advertise(ORB_ID(Teensy), &report);
	// orb_advert_t Teensy_pub_test = orb_advertise(ORB_ID(teensy_test),&test);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				      start();

	int					     collect();

};
