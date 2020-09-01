// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2020 Mission Systems Pty Ltd
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright owner nor the names of the contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "types.h"
#include <comma/visiting/traits.h>

namespace comma { namespace visiting {

template <> struct traits< snark::echodyne::status_data_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::echodyne::status_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "state", t.state );
        v.apply( "search_fps", t.search_fps );
        v.apply( "platform_height", t.platform_height );
        v.apply( "orientation", t.orientation );
        v.apply( "velocity", t.velocity );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::echodyne::status_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "state", t.state );
        v.apply( "search_fps", t.search_fps );
        v.apply( "platform_height", t.platform_height );
        v.apply( "orientation", t.orientation );
        v.apply( "velocity", t.velocity );
    }
};

template <> struct traits< snark::echodyne::rvmap_data_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::echodyne::rvmap_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "beam_az", t.beam_az );
        v.apply( "beam_el", t.beam_el );
        v.apply( "dR", t.dR );
        v.apply( "num_ranges", t.num_ranges );
        v.apply( "dV", t.dV );
        v.apply( "num_velocities", t.num_velocities );
        v.apply( "orientation", t.orientation );
        v.apply( "search_fps", t.search_fps );
        v.apply( "zero_range_bin", t.zero_range_bin );
        v.apply( "zero_doppler_bin", t.zero_doppler_bin );
        v.apply( "platform_height", t.platform_height );
        v.apply( "velocity", t.velocity );
        v.apply( "status", t.status );
        v.apply( "data", t.data );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::echodyne::rvmap_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "beam_az", t.beam_az );
        v.apply( "beam_el", t.beam_el );
        v.apply( "dR", t.dR );
        v.apply( "num_ranges", t.num_ranges );
        v.apply( "dV", t.dV );
        v.apply( "num_velocities", t.num_velocities );
        v.apply( "orientation", t.orientation );
        v.apply( "search_fps", t.search_fps );
        v.apply( "zero_range_bin", t.zero_range_bin );
        v.apply( "zero_doppler_bin", t.zero_doppler_bin );
        v.apply( "platform_height", t.platform_height );
        v.apply( "velocity", t.velocity );
        v.apply( "status", t.status );
        v.apply( "data", t.data );
    }
};

template <> struct traits< snark::echodyne::detection_data_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::echodyne::detection_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "power", t.power );
        v.apply( "snr", t.snr );
        v.apply( "rbe", t.rbe );
        v.apply( "vradial", t.vradial );
        v.apply( "r_interp", t.r_interp );
        v.apply( "detection_id", t.detection_id );
        v.apply( "rcs", t.rcs );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::echodyne::detection_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "power", t.power );
        v.apply( "snr", t.snr );
        v.apply( "rbe", t.rbe );
        v.apply( "vradial", t.vradial );
        v.apply( "r_interp", t.r_interp );
        v.apply( "detection_id", t.detection_id );
        v.apply( "rcs", t.rcs );
    }
};

template <> struct traits< snark::echodyne::track_data_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::echodyne::track_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "id", t.id );
        v.apply( "state", t.state );
        v.apply( "rbe", t.rbe );
        v.apply( "position", t.position );
        v.apply( "velocity", t.velocity );
        v.apply( "toca", t.toca );
        v.apply( "doca", t.doca );
        v.apply( "lifetime", t.lifetime );
        v.apply( "last_update_time", t.last_update_time );
        v.apply( "last_associated_data_time", t.last_associated_data_time );
        v.apply( "acquired_time", t.acquired_time );
        v.apply( "confidence", t.confidence );
        v.apply( "num_assoc_measurements", t.num_assoc_measurements );
        v.apply( "rcs", t.rcs );
        v.apply( "probability_other", t.probability_other );
        v.apply( "probability_uav", t.probability_uav );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::echodyne::track_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "id", t.id );
        v.apply( "state", t.state );
        v.apply( "rbe", t.rbe );
        v.apply( "position", t.position );
        v.apply( "velocity", t.velocity );
        v.apply( "toca", t.toca );
        v.apply( "doca", t.doca );
        v.apply( "lifetime", t.lifetime );
        v.apply( "last_update_time", t.last_update_time );
        v.apply( "last_associated_data_time", t.last_associated_data_time );
        v.apply( "acquired_time", t.acquired_time );
        v.apply( "confidence", t.confidence );
        v.apply( "num_assoc_measurements", t.num_assoc_measurements );
        v.apply( "rcs", t.rcs );
        v.apply( "probability_other", t.probability_other );
        v.apply( "probability_uav", t.probability_uav );
    }
};

template <> struct traits< snark::echodyne::meas_data_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::echodyne::meas_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "id", t.id );
        v.apply( "type", t.type );
        v.apply( "hyper_cube_reject", t.hyper_cube_reject );
        v.apply( "rbe", t.rbe );
        v.apply( "rcs", t.rcs );
        v.apply( "vradial", t.vradial );
        v.apply( "num_detections", t.num_detections );
        v.apply( "detection_ids", t.detection_ids );
        v.apply( "probability_other", t.probability_other );
        v.apply( "probability_uav", t.probability_uav );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::echodyne::meas_data_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "id", t.id );
        v.apply( "type", t.type );
        v.apply( "hyper_cube_reject", t.hyper_cube_reject );
        v.apply( "rbe", t.rbe );
        v.apply( "rcs", t.rcs );
        v.apply( "vradial", t.vradial );
        v.apply( "num_detections", t.num_detections );
        v.apply( "detection_ids", t.detection_ids );
        v.apply( "probability_other", t.probability_other );
        v.apply( "probability_uav", t.probability_uav );
    }
};

} } // namespace comma { namespace visiting {
