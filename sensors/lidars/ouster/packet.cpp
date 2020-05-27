// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020 Mission Systems Pty Ltd

#include "packet.h"
#include <iostream>

namespace snark { namespace ouster { namespace OS1 {

beam_angle_lut_t get_beam_angle_lut( const beam_intrinsics_t& beam_intrinsics )
{
    beam_angle_lut_t beam_angle_lut;

    // not set - that's okay but not ideal, return lut with no corrections
    if( beam_intrinsics.beam_altitude_angles.size() == 0 && beam_intrinsics.beam_azimuth_angles.size() == 0 ) { return beam_angle_lut; }

    // incorrectly set - flag the error then return lut with no corrections
    if( beam_intrinsics.beam_altitude_angles.size() != beam_angle_lut.size() ||
        beam_intrinsics.beam_azimuth_angles.size() != beam_angle_lut.size() )
    {
        std::cerr << "ouster: expected " << beam_angle_lut.size() << " intrinsic beam angles, got "
                  << beam_intrinsics.beam_altitude_angles.size() << " altitude angles and "
                  << beam_intrinsics.beam_azimuth_angles.size() << " azimuth angles" << std::endl;
        return beam_angle_lut;
    }

    // correctly set - set lut
    for( unsigned int i = 0; i < ouster::OS1::pixels_per_column; i++ )
    {
        beam_angle_lut[i] = beam_angle_lut_entry( beam_intrinsics.beam_altitude_angles[i] * 2 * M_PI / 360.0
                                                , beam_intrinsics.beam_azimuth_angles[i] * 2 * M_PI / 360.0 );
    }
    return beam_angle_lut;
}

} } } // namespace snark { namespace ouster { namespace OS1 {
