/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file parameters.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "aux_parameters.h"

namespace auxSensors
{

void initialize_parameter_handles(ParameterHandles &parameter_handles)
{
    /* rotations */
    parameter_handles.board_rotation = param_find("SENS_AUX_BOARD_ROT");

    /* rotation offsets */
    parameter_handles.board_offset[0] = param_find("SENS_AUX_BOARD_X_OFF");
    parameter_handles.board_offset[1] = param_find("SENS_AUX_BOARD_Y_OFF");
    parameter_handles.board_offset[2] = param_find("SENS_AUX_BOARD_Z_OFF");

}

int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters)
{
    int ret = PX4_OK;

    param_get(parameter_handles.board_rotation, &(parameters.board_rotation));
    param_get(parameter_handles.board_offset[0], &(parameters.board_offset[0]));
    param_get(parameter_handles.board_offset[1], &(parameters.board_offset[1]));
    param_get(parameter_handles.board_offset[2], &(parameters.board_offset[2]));

    return ret;
}

} /* namespace sensors */
