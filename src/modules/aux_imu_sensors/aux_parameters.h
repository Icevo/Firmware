
#pragma once

/**
 * @file parameters.h
 *
 * defines the list of parameters that are used within the sensors module
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */
#include <px4_config.h>

#include <systemlib/param/param.h>
#include <mathlib/mathlib.h>


namespace auxSensors
{

    struct Parameters{
        
        int32_t board_rotation;

	float board_offset[3];
        
    };
    
    struct ParameterHandles{
        
        param_t board_rotation;

	param_t board_offset[3];
        
    };

/**
 * initialize ParameterHandles struct
 */
void initialize_parameter_handles(ParameterHandles &parameter_handles);


/**
 * Read out the parameters using the handles into the parameters struct.
 * @return 0 on success, <0 on error
 */
int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters);
}
