/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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




#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer/mixer.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vertical_frame_attitude.h>
#include <uORB/topics/aux_sensor_gyro.h>
#include <uORB/topics/parameter_update.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aux_angle_control_main(int argc, char *argv[]);

#define GUARD_MIN(high, low)                    ((high + low)/2 - (high - low)*0.05)
#define GUARD_MAX(high, low)                    ((high + low)/2 + (high - low)*0.05)
#define K_RC_ANG(maxAng, maxRC, maxGuard)       (maxAng/(maxRC - maxGuard))
#define MAX_ANGLE_PITCH_V                       (80 * M_PI / 180)

#define RC_PITCH_MAX_V                          (0.8)
#define RC_PITCH_MIN_V                          (-0.4)

class AuxAngleControl
{
public:
        /**
         * Constructor
         */
        AuxAngleControl();

        /**
         * Destructor, also kills the main task
         */
        ~AuxAngleControl();

        /**
         * Start the multicopter attitude control task.
         *
         * @return		OK on success.
         */
        int		start();

private:

        bool	_task_should_exit;		/**< if true, task_main() should exit */
        int		_control_task;			/**< task handle */

        int             _vertical_frame_gyro_sub;
        int             _vertical_frame_att_sub;
        int		_v_att_sub;		/**< vehicle attitude subscription */
        int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
        int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
        int		_v_control_mode_sub;	/**< vehicle control mode subscription */
        int		_params_sub;			/**< parameter updates subscription */
        int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
        int		_vehicle_status_sub;	/**< vehicle status subscription */
        int		_motor_limits_sub;		/**< motor limits subscription */

        MultirotorMixer::saturation_status _saturation_status{};

        orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
        orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
        orb_advert_t	_controller_status_pub;	/**< controller status publication */

        orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
        orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

        bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

        struct aux_sensor_gyro_s                _aux_sensor_gyro_data;
        struct vertical_frame_attitude_s        _vertical_frame_att;
        struct vehicle_attitude_s		_v_att;			/**< vehicle attitude */
        struct vehicle_attitude_setpoint_s	_v_att_sp;		/**< vehicle attitude setpoint */
        struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
        struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
        struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
        struct actuator_controls_s		_actuators;		/**< actuator controls */
        struct vehicle_status_s			_vehicle_status;	/**< vehicle status */

        perf_counter_t	_loop_perf;			/**< loop performance counter */
        perf_counter_t	_controller_latency_perf;

        math::LowPassFilter2p _lp_filter_rate;
        math::LowPassFilter2p _lp_filter_d;                          /**< low-pass filter for D-term*/
        static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
        float _loop_update_rate_hz;                                  /**< current rate-controller loop update rate in [Hz] */

        float _vert_frame_sens;
        float _vert_frame_sp;
        float _vert_frame_p_err;
        float _vert_frame_rate_sens;
        float _vert_frame_rate_sp;
        float _vert_frame_rate_err_integ;
        float _vert_frame_rate_err_prev;
        float _vert_frame_rate_prop;
        float _vert_frame_rate_integ;
        float _vert_frame_rate_d_filtered;
        float _vert_frame_pid_out;


        float _rate_prev;		/**< angular rates on previous step */
        float _rate_prev_filtered;	/**< angular rates on previous step (low-pass filtered) */
        float _rate_sp;		/**< angular rates setpoint */
        float _rate_int;		/**< angular rates integral error */

        float _d_prev;
        float _d_prev_filtered;

        struct {
                param_t vert_frame_p;
                param_t vert_frame_rate_p;
                param_t vert_frame_rate_i;
                param_t vert_frame_rate_integ_lim;
                param_t vert_frame_rate_d;
                param_t vert_frame_rate_ff;
                param_t d_term_cutoff_freq;

        }		_params_handles;		/**< handles for interesting parameters */

        struct {
                float vert_frame_att_p;					/**< P gain for angular error */
                float vert_frame_rate_p;				/**< P gain for angular rate error */
                float vert_frame_rate_i;				/**< I gain for angular rate error */
                float vert_frame_rate_int_lim;			/**< integrator state limit for rate loop */
                float vert_frame_rate_d;	        /**< D gain for angular rate error */
                float vert_frame_rate_ff;			/**< Feedforward gain for desired rates */
                float d_term_cutoff_freq;			/**< Cutoff frequency for the D-term filter */

        }		_params;

        /**
         * Update our local parameter cache.
         */
        void			parameters_update();

        /**
         * Check for parameter update and handle it.
         */
        void            aux_gyro_data_poll();
        void		vehicle_attitude_poll();
        void		vehicle_control_mode_poll();
        void		vehicle_manual_poll();
        void		vehicle_motor_limits_poll();
        void		vehicle_status_poll();

        /**
         * Attitude controller.
         */
        void		control_attitude(float dt);

        /**
         * Shim for calling task_main from task_create.
         */
        static void	task_main_trampoline(int argc, char *argv[]);

        /**
         * Main attitude control task.
         */
        void		task_main();

        /**
         * Convert rc value to radians
         */

        void            val_rc_to_rad();
};

namespace aux_angle_att_control
{

AuxAngleControl	*g_control;
}

AuxAngleControl::AuxAngleControl() :

        _task_should_exit(false),
        _control_task(-1),

        /* subscriptions */
        _vertical_frame_gyro_sub(-1),
        _vertical_frame_att_sub(-1),
        _v_att_sub(-1),
        _v_control_mode_sub(-1),
        _params_sub(-1),
        _manual_control_sp_sub(-1),
        _vehicle_status_sub(-1),
        _motor_limits_sub(-1),

        /* publications */
        _v_rates_sp_pub(nullptr),
        _actuators_0_pub(nullptr),
        _actuators_id(nullptr),

        _actuators_0_circuit_breaker_enabled(false),

        _vertical_frame_att{},
        _v_att{},
        _manual_control_sp{},
        _v_control_mode{},
        _actuators{},
        _vehicle_status{},

        /* performance counters */
        _loop_perf(perf_alloc(PC_ELAPSED, "vert_frame_att_control")),
        _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),

        _lp_filter_rate(initial_update_rate_hz, 50.f),
        _lp_filter_d(initial_update_rate_hz, 50.f),// will be initialized correctly when params are loaded

_loop_update_rate_hz(initial_update_rate_hz)
{

        _vehicle_status.is_rotary_wing = true;

        _params_handles.vert_frame_p			= 	param_find("MC_ROLL_P");
        _params_handles.vert_frame_rate_p		= 	param_find("MC_ROLLRATE_P");
        _params_handles.vert_frame_rate_i		= 	param_find("MC_ROLLRATE_I");
        _params_handles.vert_frame_rate_integ_lim	= 	param_find("MC_RR_INT_LIM");
        _params_handles.vert_frame_rate_d		= 	param_find("MC_ROLLRATE_D");
        _params_handles.vert_frame_rate_ff		= 	param_find("MC_ROLLRATE_FF");
        _params_handles.d_term_cutoff_freq              = 	param_find("MC_DTERM_CUTOFF");

        /* fetch initial parameter values */
        parameters_update();
}

AuxAngleControl::~AuxAngleControl()
{
        if (_control_task != -1) {
                /* task wakes up every 100ms or so at the longest */
                _task_should_exit = true;

                /* wait for a second for the task to quit at our request */
                unsigned i = 0;

                do {
                        /* wait 20ms */
                        usleep(20000);

                        /* if we have given up, kill it */
                        if (++i > 50) {
                                px4_task_delete(_control_task);
                                break;
                        }
                } while (_control_task != -1);
        }

       aux_angle_att_control::g_control = nullptr;
}

void
AuxAngleControl::parameters_update()
{
        float v;

        /* aux_angle gains */
        param_get(_params_handles.vert_frame_p, &v);
        _params.vert_frame_att_p = v;
        param_get(_params_handles.vert_frame_rate_p, &v);
        _params.vert_frame_rate_p = v;
        param_get(_params_handles.vert_frame_rate_i, &v);
        _params.vert_frame_rate_i = v;
        param_get(_params_handles.vert_frame_rate_integ_lim, &v);
        _params.vert_frame_rate_int_lim = v;
        param_get(_params_handles.vert_frame_rate_d, &v);
        _params.vert_frame_rate_d = v;
        param_get(_params_handles.vert_frame_rate_ff, &v);
        _params.vert_frame_rate_ff = v;

        param_get(_params_handles.d_term_cutoff_freq, &_params.d_term_cutoff_freq);

        if (fabsf(_lp_filter_rate.get_cutoff_freq() - _params.d_term_cutoff_freq) > 0.01f) {
                _lp_filter_rate.set_cutoff_frequency(_loop_update_rate_hz, _params.d_term_cutoff_freq);
                _lp_filter_d.set_cutoff_frequency(_loop_update_rate_hz, _params.d_term_cutoff_freq);
                _lp_filter_rate.reset(_rate_prev);
                _lp_filter_d.reset(_d_prev);
        }

        _actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);
}

void
AuxAngleControl::aux_gyro_data_poll()
{
        bool updated;

        orb_check(_vertical_frame_gyro_sub, &updated);

        if(updated) {
            orb_copy(ORB_ID(aux_sensor_gyro), _vertical_frame_gyro_sub, &_aux_sensor_gyro_data);
        }
}

void
AuxAngleControl::vehicle_control_mode_poll()
{
        bool updated;

        /* Check if vehicle control mode has changed */
        orb_check(_v_control_mode_sub, &updated);

        if (updated) {
                orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
        }
}

void
AuxAngleControl::vehicle_manual_poll()
{
        bool updated;

        /* get pilots inputs */
        orb_check(_manual_control_sp_sub, &updated);

        if (updated) {
                orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
        }
}

void
AuxAngleControl::vehicle_status_poll()
{
        /* check if there is new status information */
        bool vehicle_status_updated;
        orb_check(_vehicle_status_sub, &vehicle_status_updated);

        if (vehicle_status_updated)
        {
                orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

                /* set correct uORB ID */
                if (_rates_sp_id == nullptr)
                {

                     _rates_sp_id = ORB_ID(vehicle_rates_setpoint);
                     _actuators_id = ORB_ID(actuator_controls_0);
                 }
          }
}

void
AuxAngleControl::vehicle_motor_limits_poll()
{
        /* check if there is a new message */
        bool updated;
        orb_check(_motor_limits_sub, &updated);

        if (updated) {
                multirotor_motor_limits_s motor_limits = {};
                orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &motor_limits);

                _saturation_status.value = motor_limits.saturation_status;
        }
}

void
AuxAngleControl::vehicle_attitude_poll()
{
        /* check if there is a new message */
        bool updated;
        orb_check(_v_att_sub, &updated);

        if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
        }
}

void
AuxAngleControl::val_rc_to_rad()
{
    float angle;
    float coef;
    float guard_max = GUARD_MAX(RC_PITCH_MAX_V, RC_PITCH_MIN_V);
    float guard_min = GUARD_MIN(RC_PITCH_MAX_V, RC_PITCH_MIN_V);

    coef = MAX_ANGLE_PITCH_V/(RC_PITCH_MAX_V - (double)guard_max);

    if(_manual_control_sp.aux1 > guard_max)
    {
        angle = coef * (_manual_control_sp.aux1 - guard_max);
    }
    else if(_manual_control_sp.aux1 < guard_min)
    {
         angle = coef * (_manual_control_sp.aux1 - guard_min);
    }
    else
    {
        angle = 0;
    }

    _vert_frame_sp = angle;
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
AuxAngleControl::control_attitude(float dt)
{
        vehicle_attitude_poll();
        vehicle_manual_poll();

        /* Get main frame pitch */
        float main_frame_pitch = atan2f((2*_v_att.q[1]*_v_att.q[0] - 2*_v_att.q[2]*_v_att.q[3]), (1 - 2*_v_att.q[1]*_v_att.q[1] - 2*_v_att.q[3]*_v_att.q[3]));

        /* Get error angle between vert_frame and main_frame */
        _vert_frame_sens = _vertical_frame_att.pitch - main_frame_pitch;

        /* compute vertical frame setpoint from rc pitch stick */
        val_rc_to_rad();

        /* Compute pid for angle error*/
        _vert_frame_p_err = _vert_frame_sp - _vert_frame_sens;

        _vert_frame_rate_sp = _params.vert_frame_att_p * _vert_frame_p_err;

        /* Speed control PID side */
        float vert_frame_rate = _aux_sensor_gyro_data.y ;//TO DO :- _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];

        //vert_frame_rate -= _sensor_bias.gyro_x_bias; TO DO: compute aux sensor bias

        /* apply low-pass filtering to the rates for D-term */
        float rate_filtered = _lp_filter_rate.apply(vert_frame_rate);
        _rate_prev = vert_frame_rate;
        _rate_prev_filtered = rate_filtered;

        _vert_frame_rate_sens = rate_filtered;

        /*  Perform pid computing    */
        float err = _vert_frame_rate_sp - _vert_frame_rate_sens;
        float abs_err = fabs(err);

        if((abs_err > _params.vert_frame_rate_int_lim) && ((int)_params.vert_frame_rate_i !=0))
        {
            _vert_frame_rate_err_integ = _vert_frame_rate_err_integ + (err * dt);
        }
        if((int)_vert_frame_rate_err_prev == 0)
        {
            _vert_frame_rate_err_prev = err;
        }

        _vert_frame_rate_err_integ = math::constrain(_vert_frame_rate_err_integ, -_params.vert_frame_rate_int_lim, _params.vert_frame_rate_int_lim);
        _vert_frame_rate_prop = _params.vert_frame_rate_p * err;
        _vert_frame_rate_integ = _params.vert_frame_rate_i * _vert_frame_rate_err_integ;
         float deriv = _params.vert_frame_rate_d * (err - _vert_frame_rate_err_prev) / dt;

         /*  Lpf filter for d term*/
         _vert_frame_rate_d_filtered = _lp_filter_d.apply(deriv);
         _d_prev = deriv;
         _d_prev_filtered = _vert_frame_rate_d_filtered;

         _vert_frame_pid_out = _vert_frame_rate_prop + _vert_frame_rate_integ + _vert_frame_rate_d_filtered;
}


void
AuxAngleControl::task_main_trampoline(int argc, char *argv[])
{
        aux_angle_att_control::g_control->task_main();
}

void
AuxAngleControl::task_main()
{

        /*
         * do subscriptions
         */
        _vertical_frame_gyro_sub = orb_subscribe(ORB_ID(aux_sensor_gyro));
        _vertical_frame_att_sub = orb_subscribe(ORB_ID(vertical_frame_attitude));
        _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
        _params_sub = orb_subscribe(ORB_ID(parameter_update));
        _manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
        _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
        _motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));

        /* initialize parameters cache */
        parameters_update();

        /* wakeup source: gyro data from sensor selected by the sensor app */
        px4_pollfd_struct_t poll_fds = {};
        poll_fds.events = POLLIN;

        const hrt_abstime task_start = hrt_absolute_time();
        hrt_abstime last_run = task_start;
        float dt_accumulator = 0.f;
        int loop_counter = 0;

        while (!_task_should_exit) {

                poll_fds.fd = _vertical_frame_att_sub;

                /* wait for up to 100ms for data */
                int pret = px4_poll(&poll_fds, 1, 100);

                /* timed out - periodic check for _task_should_exit */
                if (pret == 0) {
                        continue;
                }

                /* this is undesirable but not much we can do - might want to flag unhappy status */
                if (pret < 0) {
                        warn("mc att ctrl: poll error %d, %d", pret, errno);
                        /* sleep a bit before next try */
                        usleep(100000);
                        continue;
                }

                perf_begin(_loop_perf);

                /* run controller on gyro changes */
                if (poll_fds.revents & POLLIN) {
                        const hrt_abstime now = hrt_absolute_time();
                        float dt = (now - last_run) / 1e6f;
                        last_run = now;

                        /* guard against too small (< 2ms) and too large (> 20ms) dt's */
                        if (dt < 0.002f) {
                                dt = 0.002f;

                        } else if (dt > 0.02f) {
                                dt = 0.02f;
                        }

                        /* vertical frame attitude */
                        orb_copy(ORB_ID(vertical_frame_attitude), _vertical_frame_att_sub, &_vertical_frame_att);

                        /* check for updates in other topics */
                        vehicle_control_mode_poll();
                        vehicle_manual_poll();
                        vehicle_status_poll();
                        vehicle_motor_limits_poll();
                        vehicle_attitude_poll();
                        aux_gyro_data_poll();


                        control_attitude(dt);


                        /* publish actuator controls */
                        _actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
                        _actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
                        _actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
                        _actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
                        _actuators.control[7] = _v_att_sp.landing_gear;
                        _actuators.timestamp = hrt_absolute_time();
                        _actuators.timestamp_sample = _sensor_gyro.timestamp;


                         if (!_actuators_0_circuit_breaker_enabled) {
                             if (_actuators_0_pub != nullptr) {

                                 orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
                                 perf_end(_controller_latency_perf);

                                        } else if (_actuators_id) {
                                                _actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
                                        }

                          }

                          /* publish controller status */
                          rate_ctrl_status_s rate_ctrl_status;
                          rate_ctrl_status.timestamp = hrt_absolute_time();
                          rate_ctrl_status.rollspeed = _rates_prev(0);
                          rate_ctrl_status.pitchspeed = _rates_prev(1);
                          rate_ctrl_status.yawspeed = _rates_prev(2);
                          rate_ctrl_status.rollspeed_integ = _rates_int(0);
                          rate_ctrl_status.pitchspeed_integ = _rates_int(1);
                          rate_ctrl_status.yawspeed_integ = _rates_int(2);

                          int instance;
                          orb_publish_auto(ORB_ID(rate_ctrl_status), &_controller_status_pub, &rate_ctrl_status, &instance, ORB_PRIO_DEFAULT);
                        }


                        /* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
                        if (!_v_control_mode.flag_armed || (now - task_start) < 3300000) {
                                dt_accumulator += dt;
                                ++loop_counter;

                                if (dt_accumulator > 1.f) {
                                        const float loop_update_rate = (float)loop_counter / dt_accumulator;
                                        _loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
                                        dt_accumulator = 0;
                                        loop_counter = 0;
                                        _lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _params.d_term_cutoff_freq);
                                        _lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _params.d_term_cutoff_freq);
                                        _lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _params.d_term_cutoff_freq);
                                }
                        }

                }

                perf_end(_loop_perf);
        }

        _control_task = -1;
}

int
MulticopterAttitudeControl::start()
{
        ASSERT(_control_task == -1);

        /* start the task */
        _control_task = px4_task_spawn_cmd("mc_att_control",
                                           SCHED_DEFAULT,
                                           SCHED_PRIORITY_ATTITUDE_CONTROL,
                                           1700,
                                           (px4_main_t)&MulticopterAttitudeControl::task_main_trampoline,
                                           nullptr);

        if (_control_task < 0) {
                warn("task start failed");
                return -errno;
        }

        return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
        if (argc < 2) {
                warnx("usage: mc_att_control {start|stop|status}");
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (mc_att_control::g_control != nullptr) {
                        warnx("already running");
                        return 1;
                }

                mc_att_control::g_control = new MulticopterAttitudeControl;

                if (mc_att_control::g_control == nullptr) {
                        warnx("alloc failed");
                        return 1;
                }

                if (OK != mc_att_control::g_control->start()) {
                        delete mc_att_control::g_control;
                        mc_att_control::g_control = nullptr;
                        warnx("start failed");
                        return 1;
                }

                return 0;
        }

        if (!strcmp(argv[1], "stop")) {
                if (mc_att_control::g_control == nullptr) {
                        warnx("not running");
                        return 1;
                }

                delete mc_att_control::g_control;
                mc_att_control::g_control = nullptr;
                return 0;
        }

        if (!strcmp(argv[1], "status")) {
                if (mc_att_control::g_control) {
                        warnx("running");
                        return 0;

                } else {
                        warnx("not running");
                        return 1;
                }
        }

        warnx("unrecognized command");
        return 1;
}
