/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file auxsensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <board_config.h>

#include <px4_config.h>
#include <px4_module.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_px4flow.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include <conversion/rotation.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/aux_sensor_preflight.h>

#include <DevMgr.hpp>

#include "voted_aux_sensors_update.h"

using namespace DriverFramework;
using namespace auxSensors;



/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG		5.0f
#define STICK_ON_OFF_LIMIT		0.75f

/**
 * Sensor app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aux_sensors_main(int argc, char *argv[]);

class auxsensors : public ModuleBase<auxsensors>
{
public:
        auxsensors(bool hil_enabled);

        ~auxsensors() {}


	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
        static auxsensors *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	const bool	_hil_enabled;			/**< if true, HIL is active */
	bool		_armed{false};				/**< arming status of the vehicle */

	int		_vcontrol_mode_sub{-1};		/**< vehicle control mode subscription */

	orb_advert_t	_sensor_pub{nullptr};			/**< combined sensor data topic */


	orb_advert_t	_sensor_preflight{nullptr};		/**< sensor preflight topic */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

        Parameters		_parameters{};			/**< local copies of interesting parameters */
        ParameterHandles	_parameter_handles{};		/**< handles for interesting parameters */

        VotedSensorsUpdate _voted_sensors_update;


	/**
	 * Check for changes in vehicle control mode.
	 */
        void vehicle_control_mode_poll();

        /**
         * Update our local parameter cache.
         */
        int parameters_update();

};


auxsensors::auxsensors(bool hil_enabled) :
        _hil_enabled(hil_enabled),
        _loop_perf(perf_alloc(PC_ELAPSED, "auxsensors")),
        _voted_sensors_update(_parameters, hil_enabled)
{
    initialize_parameter_handles(_parameter_handles);
}

int
auxsensors::parameters_update()
{
	if (_armed) {
		return 0;
	}

        /* read the parameter values into _parameters */
        int ret = update_parameters(_parameter_handles, _parameters);

        if (ret) {
                return ret;
        }

        _voted_sensors_update.parameters_update();

        return ret;
}

void
auxsensors::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;

	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);
		_armed = vcontrol_mode.flag_armed;
	}
}

void
auxsensors::run()
{

        struct aux_sensor_combined_s raw = {};

        struct aux_sensor_preflight_s preflt = {};

        _voted_sensors_update.init(raw);

	/*
	 * do subscriptions
	 */

	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	/* get a set of initial values */
        _voted_sensors_update.sensors_poll(raw);

	/* advertise the sensor_combined topic and make the initial publication */
        _sensor_pub = orb_advertise(ORB_ID(aux_sensor_combined), &raw);

	/* advertise the sensor_preflight topic and make the initial publication */
        //preflt.accel_inconsistency_m_s_s = 0.0f;

        //preflt.gyro_inconsistency_rad_s = 0.0f;

        //preflt.mag_inconsistency_ga = 0.0f;

        _sensor_preflight = orb_advertise(ORB_ID(aux_sensor_preflight), &preflt);

	/* wakeup source */
	px4_pollfd_struct_t poll_fds = {};

	poll_fds.events = POLLIN;

        uint64_t last_config_update = hrt_absolute_time();

	while (!should_exit()) {

		/* use the best-voted gyro to pace output */
                poll_fds.fd = _voted_sensors_update.best_gyro_fd();

		/* wait for up to 50ms for data (Note that this implies, we can have a fail-over time of 50ms,
		 * if a gyro fails) */
		int pret = px4_poll(&poll_fds, 1, 50);

		/* if pret == 0 it timed out - periodic check for should_exit(), etc. */

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			/* if the polling operation failed because no gyro sensor is available yet,
			 * then attempt to subscribe once again
			 */
                        if (_voted_sensors_update.num_gyros() == 0) {
                                _voted_sensors_update.initialize_sensors();
			}

			usleep(1000);

			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

		/* the timestamp of the raw struct is updated by the gyro_poll() method (this makes the gyro
                 * a mandtory sensor) */
                _voted_sensors_update.sensors_poll(raw);

		if (raw.timestamp > 0) {

                        _voted_sensors_update.set_relative_timestamps(raw);

                        orb_publish(ORB_ID(aux_sensor_combined), _sensor_pub, &raw);

                        _voted_sensors_update.check_failover();

			/* If the the vehicle is disarmed calculate the length of the maximum difference between
			 * IMU units as a consistency metric and publish to the sensor preflight topic
                        */
			if (!_armed) {
                                _voted_sensors_update.calc_accel_inconsistency(preflt);
                                _voted_sensors_update.calc_gyro_inconsistency(preflt);
                                _voted_sensors_update.calc_mag_inconsistency(preflt);
                                orb_publish(ORB_ID(aux_sensor_preflight), _sensor_preflight, &preflt);

                        }
		}

                /* keep adding auxsensors as long as we are not armed,
                */
                if (!_armed && hrt_elapsed_time(&last_config_update) > 500 * 1000)
                {
                        //_voted_sensors_update.initialize_sensors();
			last_config_update = hrt_absolute_time();
                }

                perf_end(_loop_perf);
	}

	orb_unsubscribe(_vcontrol_mode_sub);
	orb_unadvertise(_sensor_pub);
        _voted_sensors_update.deinit();
}

int auxsensors::task_spawn(int argc, char *argv[])
{
	/* start the task */
        _task_id = px4_task_spawn_cmd("auxsensors",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_SENSOR_HUB,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int auxsensors::print_status()
{
        _voted_sensors_update.print_status();

	return 0;
}

int auxsensors::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int auxsensors::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The auxsensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:
- Read the output from the sensor drivers (`sensor_gyro`, etc.).
  If there are multiple of the same type, do voting and failover handling.
  Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the
  topics is `sensor_combined`, used by many parts of the system.
- Do RC channel mapping: read the raw input channels (`input_rc`), then apply the calibration, map the RC channels
  to the configured channels & mode switches, low-pass filter, and then publish as `rc_channels` and
  `manual_control_setpoint`.
- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
  on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the
  sensor drivers must already be running when `auxsensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("auxsensors", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

auxsensors *auxsensors::instantiate(int argc, char *argv[])
{
	bool hil_enabled = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "h", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'h':
			hil_enabled = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

        return new auxsensors(hil_enabled);;
}

int aux_sensors_main(int argc, char *argv[])
{
        return auxsensors::main(argc, argv);
}
