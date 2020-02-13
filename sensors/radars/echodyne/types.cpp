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
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
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

#include "types.h"

namespace snark { namespace echodyne {

// Assumes system clock has been configured to be time since unix epoch
boost::posix_time::ptime from_echodyne_time( uint32_t days, uint32_t ms )
{
    boost::posix_time::ptime t = boost::posix_time::from_time_t( days * 86400 );
    t += boost::posix_time::milliseconds( ms );
    return t;
}

status_data_t::status_data_t( const sstat_data* data )
    : t( from_echodyne_time( data->sys_time_days, data->sys_time_ms ))
    , state( data->sys_state )
    , search_fps( data->search_fps )
    , platform_height( data->platform_height )
    , orientation( data->quat_x, data->quat_y, data->quat_z, data->quat_w )
    , velocity( data->platform_velocity_x, data->platform_velocity_y, data->platform_velocity_z )
    , time_channel_state( data->time_channel_state )
    , eth_speed( data->eth_link_rate )
{}

rvmap_data_t::rvmap_data_t( const rvmap_header* header, const uint32_t* data_ )
    : t( from_echodyne_time( header->trig_day, header->trig_ms ))
    , beam_az( header->beam_az )
    , beam_el( header->beam_el )
    , dR( header->dR )
    , num_ranges( header->nRanges )
    , dV( header->dV )
    , num_velocities( header->nVelocities )
    , orientation( header->quat_x, header->quat_y, header->quat_z, header->quat_w )
    , search_fps( header->search_fps )
    , zero_range_bin( header->zero_range_bin )
    , zero_doppler_bin( header->zero_doppler_bin )
    , platform_height( header->platform_height )
    , velocity( header->platform_velocity_x, header->platform_velocity_y, header->platform_velocity_z )
    , status( header->status )
{
    // this is pretty inefficient (we shouldn't need to copy) but is convenient for symmetry with the other types
    for( unsigned int i = 0; i < MAX_RVMAP_SIZE; i++ ) { data[i] = *( data_ + i ); }
}

detection_data_t::detection_data_t( const detec_header* header, const detec_data& data )
    // field name is detec_time_sec but it's actually milliseconds
    : t( from_echodyne_time( header->detec_time_days, header->detec_time_sec ))
    , power( data.power )
    , snr( data.snr )
    , position( data.r, data.az, data.el )
    , vradial( data.vradial )
    , r_interp( data.r_interp )
    , detection_id( data.detection_ID )
    , rcs( data.rcs )
{}

track_data_t::track_data_t( const track_header* header, const track_data& data )
    : t( from_echodyne_time( header->sys_time_days, header->sys_time_ms ))
    , id( data.ID )
    , state( data.state )
    , rbe( data.rest, data.azest, data.elest )
    , position( data.xest, data.yest, data.zest )
    , velocity( data.velxest, data.velyest, data.velzest )
    , toca( from_echodyne_time( data.TOCA_days, data.TOCA_ms ))
    , doca( data.DOCA )
    , lifetime( data.lifetime )
    , last_update_time( from_echodyne_time( data.lastUpdateTime_days, data.lastUpdateTime_ms ))
    , last_associated_data_time( from_echodyne_time( data.lastAssociatedDataTime_days, data.lastAssociatedDataTime_ms ))
    , acquired_time( from_echodyne_time( data.acquiredTime_days, data.acquiredTime_ms ))
    , confidence( data.estConfidence )
    , num_assoc_measurements( data.numAssocMeasurements )
    , rcs( data.estRCS )
    , probability_other( data.probabilityOther )
    , probability_uav( data.probabilityUAV )
{}

meas_data_t::meas_data_t( const meas_header* header, const meas_data& data )
    : t( from_echodyne_time( header->systemTime_days, header->systemTime_ms ))
    , id( data.id )
    , type( data.type )
    , hyper_cube_reject( data.b_hyperCubeReject )
    , rbe( data.r, data.az, data.el )
    , rcs( data.rcsEst )
    , vradial( data.vradial )
    , num_detections( data.nDetections )
    , probability_other( data.prob_other )
    , probability_uav( data.prob_uav )
{
    for( unsigned int i = 0; i < num_detections; i++ ) { detection_ids[i] = data.detIds[i]; }
}

} } // namespace snark { namespace echodyne {
