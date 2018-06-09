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
 * @file attitude_filter.cpp
 *
 * @author 

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

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include "vertical_frame_attitude/six_axis_comp_filter.h"

#include <uORB/uORB.h>
#include <uORB/topics/aux_sensor_combined.h>
#include <uORB/topics/vertical_frame_attitude.h>

/**Defines **/
#define COMPL_FILT_KP_MAX                   (0.015f)
#define COMPL_FILT_KP_MIN                   (0.000f)
#define COMPL_FILT_KI_MAX                   (0.002f)
#define COMPL_FILT_KI_MIN                   (0.0005f)
#define COMPL_FILT_MAX_ANG_FOR_BIAS         (0.1f)
#define COMPL_FILT_BIAS_MAX                 (0.05f)


using namespace attitudeFilter;

extern "C" __EXPORT int vertical_frame_attitude_main(int argc, char *argv[]);

class AttitudeFilter : public ModuleBase<AttitudeFilter>
{
public:
	AttitudeFilter(bool hil_enabled);

	~AttitudeFilter() {}

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static AttitudeFilter *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

        //update new values in attitude class
        void updateFilter(aux_sensor_combined_s *raw, vertical_frame_attitude_s *attitude);

private:
	const bool	_hil_enabled;			/**< if true, HIL is active */
	perf_counter_t	_loop_perf;			/**< loop performance counter */
	int _aux_sensor_data{-1};
	orb_advert_t _attitude_pub{nullptr};
        struct vertical_frame_attitude_s att = {};
        CompSixAxis _comp_filter;

};

AttitudeFilter::AttitudeFilter(bool hil_enabled) :
	_hil_enabled(hil_enabled),
        _loop_perf(perf_alloc(PC_ELAPSED, "AttitudeFilter")),
        _comp_filter()
{};

void AttitudeFilter::run()
{
	struct aux_sensor_combined_s raw = {};	
	/*
	 * do subscriptions
	 */	
	_aux_sensor_data = orb_subscribe(ORB_ID(aux_sensor_combined));

	/* wakeup source */
        px4_pollfd_struct_t poll_fds = {};

        while(!should_exit())
        {

        poll_fds.fd = _aux_sensor_data;
        poll_fds.events = POLLIN;

        /* wait 500 ms for new data*/
        int pret = px4_poll(&poll_fds, 1, 500);

	if (pret < 0)
	{
            PX4_WARN("aux_imu_data = NOT AVAILABLE!!!\n");
        } else if (pret == 0)
                {
                    // Poll timeout or no new data, do nothing
                    continue;
                }


	perf_begin(_loop_perf);

        orb_copy(ORB_ID(aux_sensor_combined), _aux_sensor_data, &raw);

        //update attitude angles
        updateFilter(&raw, &att);

        /*publish attitude data*/
        if (_attitude_pub == nullptr) {
                _attitude_pub = orb_advertise(ORB_ID(vertical_frame_attitude), &att);

        } else {
                orb_publish(ORB_ID(vertical_frame_attitude), _attitude_pub, &att);
        }

	perf_end(_loop_perf);
	}	

        orb_unsubscribe(_aux_sensor_data);
        orb_unadvertise(_attitude_pub);

}

void AttitudeFilter::updateFilter(aux_sensor_combined_s *raw, vertical_frame_attitude_s *attitude)
{
    //update dt
    _comp_filter.CompDeltaTUpdate(raw->gyro_integral_dt/1e6f, GYRO_DRIFT_COEF);
    //update acc, gyro data in complementary filter class
    _comp_filter.CompAccelUpdate(raw->accelerometer_m_s2[0], raw->accelerometer_m_s2[1], raw->accelerometer_m_s2[2]);
    _comp_filter.CompGyroUpdate(raw->gyro_rad[0], raw->gyro_rad[1], raw->gyro_rad[2]);
    //start computing angles
    _comp_filter.CompStart();
    //update values
    _comp_filter.CompUpdate();
    //get values
    _comp_filter.CompAnglesGet(&attitude->roll, &attitude->pitch);
}

int AttitudeFilter::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("AttitudeFilter",
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

int AttitudeFilter::print_status()
{
	//add some information for status
	return 0;
}

int AttitudeFilter::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AttitudeFilter::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The sensors module is central to the whole system. It takes low-level output from drivers, turns
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
  sensor drivers must already be running when `sensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensors", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

AttitudeFilter *AttitudeFilter::instantiate(int argc, char *argv[])
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

	return new AttitudeFilter(hil_enabled);;
}

int vertical_frame_attitude_main(int argc, char *argv[])
{
	return AttitudeFilter::main(argc, argv);
}
