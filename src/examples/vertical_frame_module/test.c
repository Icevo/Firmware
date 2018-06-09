/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vertical_frame_attitude.h>
#include <uORB/topics/aux_sensor_combined.h>
#include <uORB/topics/aux_sensor_accel.h>
#include <uORB/topics/aux_sensor_gyro.h>
#include <uORB/topics/aux_sensor_mag.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int test_main(int argc, char *argv[]);

int test_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
	//int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	//int sensor_sub_fd = orb_subscribe(ORB_ID(vertical_frame_attitude));
        //int sensor_sub_fd = orb_subscribe(ORB_ID(aux_sensor_combined));
        //int sensor_sub_fd1 = orb_subscribe(ORB_ID(aux_sensor_gyro));
        //int sensor_sub_fd2 = orb_subscribe(ORB_ID(aux_sensor_mag));
	
	//collect interval
	orb_set_interval(sensor_sub_fd, 1000);

         /* obtained data for the first file descriptor */
         struct actuator_controls_s raw;
	 //struct manual_control_setpoint_s raw;
	 //struct vehicle_attitude_setpoint_s raw;
	 //struct vertical_frame_attitude_s raw;
         //struct aux_sensor_combined_s raw;
         //struct aux_sensor_gyro_s gyro_raw;
         //struct aux_sensor_mag_s mag_raw;

	/* limit the update rate to 5 Hz */
	//orb_set_interval(sensor_sub_fd, 200);
       // orb_set_interval(sensor_sub_fd1,200);
       // orb_set_interval(sensor_sub_fd2,200);

	/* one could wait for multiple topics with this technique, just using one here */
        px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
                //{ .fd = sensor_sub_fd1,  .events = POLLIN },
               // { .fd = sensor_sub_fd2,  .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

        while(1)
        {

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
                int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {

				/* copy sensors raw data into local buffer */
                                orb_copy(ORB_ID(actuator_controls_0), sensor_sub_fd, &raw);
				//orb_copy(ORB_ID(manual_control_setpoint), sensor_sub_fd, &raw);
				//orb_copy(ORB_ID(vehicle_attitude_setpoint), sensor_sub_fd, &raw);
				//orb_copy(ORB_ID(vertical_frame_attitude), sensor_sub_fd, &raw);
                                //orb_copy(ORB_ID(aux_sensor_combined), sensor_sub_fd, &raw);
                                //orb_copy(ORB_ID(aux_sensor_gyro), sensor_sub_fd1, &gyro_raw);
                                //orb_copy(ORB_ID(aux_sensor_mag), sensor_sub_fd2, &mag_raw);


			        PX4_INFO("actuator_ctrl:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)raw.control[0],
                                         (double)raw.control[1],
                                         (double)raw.control[2]);
                                /*PX4_INFO("Gyro_rad:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)raw.gyro_rad[0],
                                         (double)raw.gyro_rad[1],
                                         (double)raw.gyro_rad[2]);
                                PX4_INFO("Acc_mps2:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)raw.accelerometer_m_s2[0],
                                         (double)raw.accelerometer_m_s2[1],
                                         (double)raw.accelerometer_m_s2[2]);
                                PX4_INFO("Mag_ga:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)raw.magnetometer_ga[0],
                                         (double)raw.magnetometer_ga[1],
                                         (double)raw.magnetometer_ga[2]);

                                PX4_INFO("\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.roll,
                                         (double)raw.pitch,
                                         (double)raw.yaw);

                                PX4_INFO("\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.x,
                                         (double)raw.y,
                                         (double)raw.z);*/
 
			}

		}
        }

	PX4_INFO("exiting");

	return 0;
}
