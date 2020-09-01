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

#include "../../../math/range_bearing_elevation.h"
#include <comma/base/types.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Eigen/Core>
#include <mesak_status_packet.h>
#include <mesak_rvmap_packet.h>
#include <mesak_detection_packet.h>
#include <mesak_track_packet.h>
#include <mesak_measurement_packet.h>

namespace snark { namespace echodyne {

boost::posix_time::ptime from_echodyne_time( uint32_t days, uint32_t ms );

// User manual section 7.3
struct status_data_t
{
    boost::posix_time::ptime t;
    uint32_t state;
    float search_fps;
    float platform_height;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f velocity;           // platform velocity over ground (m/s)
    uint32_t time_channel_state;
    uint32_t eth_speed;

    status_data_t() {}
    status_data_t( const sstat_data* data );
};

// User manual section 7.4
struct rvmap_data_t
{
    boost::posix_time::ptime t;
    float beam_az;
    float beam_el;
    float dR;
    float num_ranges;
    float dV;
    float num_velocities;
    Eigen::Quaternionf orientation;
    float search_fps;
    uint32_t zero_range_bin;
    uint32_t zero_doppler_bin;
    float platform_height;
    Eigen::Vector3f velocity;           // platform velocity over ground (m/s)
    uint8_t status;
    std::array< uint32_t, MAX_RVMAP_SIZE > data;

    rvmap_data_t() {}
    rvmap_data_t( const rvmap_header* header, const uint32_t* data );
};

// User manual section 7.5
struct detection_data_t
{
    boost::posix_time::ptime t;
    float power;                        // dB
    float snr;                          // dB
    range_bearing_elevation rbe;
    float vradial;                      // m/s
    float r_interp;                     // m
    uint32_t detection_id;
    float rcs;                          // radar cross section (dBsm)

    detection_data_t() {}
    detection_data_t( const detec_header* header, const detec_data& data );
};

// User manual sections 6.5.2 and 7.7
struct track_data_t
{
    boost::posix_time::ptime t;
    uint32_t id;
    uint32_t state;
    range_bearing_elevation rbe;
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    boost::posix_time::ptime toca;      // time of closest approach
    float doca;                         // distance of closest approach
    float lifetime;
    boost::posix_time::ptime last_update_time;
    boost::posix_time::ptime last_associated_data_time;
    boost::posix_time::ptime acquired_time;
    float confidence;
    uint32_t num_assoc_measurements;
    float rcs;
    float probability_other;
    float probability_uav;

    track_data_t() {}
    track_data_t( const track_header* header, const track_data& data );
};

// User manual section 7.6
struct meas_data_t
{
    boost::posix_time::ptime t;
    uint32_t id;
    uint32_t type;
    uint32_t hyper_cube_reject;
    range_bearing_elevation rbe;
    float rcs;
    float vradial;
    uint32_t num_detections;
    std::array< uint32_t, MAXNUMDETECTIONSPERMEASUREMENT > detection_ids;
    float probability_other;
    float probability_uav;

    meas_data_t() {}
    meas_data_t( const meas_header* header, const meas_data& data );
};

} } // namespace snark { namespace echodyne {
