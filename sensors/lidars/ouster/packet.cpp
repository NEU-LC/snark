// Copyright (c) 2019 The University of Sydney

#include "packet.h"

namespace ouster { namespace OS1 {

beam_angle_lut_entry beam_angle_lut[ ouster::OS1::pixels_per_column ];

void init_beam_angle_lut( const beam_intrinsics_t& beam_intrinsics )
{
    for( unsigned int i = 0; i < ouster::OS1::pixels_per_column; i++ )
    {
        beam_angle_lut[i] = { beam_intrinsics.beam_altitude_angles[i] * 2 * M_PI / 360.0
                            , beam_intrinsics.beam_azimuth_angles[i] * 2 * M_PI / 360.0 };
    }
}

} } // namespace ouster { namespace OS1 {
