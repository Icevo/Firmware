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
 * Board rotation
 *
 * This parameter defines the rotation of the FMU board relative to the platform.
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Roll 270°, Yaw 270°
 * @value 27 Roll 180°, Pitch 270°
 * @value 28 Pitch 90°, Yaw 180
 * @value 29 Pitch 90°, Roll 90°
 * @value 30 Yaw 293°, Pitch 68°, Roll 90° (Solo)
 * @value 31 Pitch 90°, Roll 270°
 * @value 32 Pitch 9°, Yaw 180°
 * @value 33 Pitch 45°
 * @value 34 Pitch 315°
 *
 * @reboot_required true
 *
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_AUX_BRD_ROT, 0);


/**
 * Board rotation Y (Pitch) offset
 *
 * This parameter defines a rotational offset in degrees around the Y (Pitch) axis. It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @unit deg
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SNS_AUX_BRD_Y_OF, 0.0f);

/**
 * Board rotation X (Roll) offset
 *
 * This parameter defines a rotational offset in degrees around the X (Roll) axis It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @unit deg
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SNS_AUX_BRD_X_OF, 0.0f);

/**
 * Board rotation Z (YAW) offset
 *
 * This parameter defines a rotational offset in degrees around the Z (Yaw) axis. It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @unit deg
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SNS_AUX_BRD_Z_OF, 0.0f);

/**
 * Thermal control of sensor temperature
 *
 * @value -1 Thermal control unavailable
 * @value 0 Thermal control off
 * @group Sensors
 */
PARAM_DEFINE_INT32(SNS_AUX_EN_TRML, -1);

/**
* Driver level cutoff frequency for gyro
*
* The cutoff frequency for the 2nd order butterworth filter on the gyro driver. This features
* is currently supported by the mpu6000 and mpu9250. This only affects the signal sent to the
* controllers, not the estimators. 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_AUX_GYRO_CTF, 30.0f);

/**
* Driver level cutoff frequency for accel
*
* The cutoff frequency for the 2nd order butterworth filter on the accel driver. This features
* is currently supported by the mpu6000 and mpu9250. This only affects the signal sent to the
* controllers, not the estimators. 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_AUX_ACEL_CTF, 30.0f);
