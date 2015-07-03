// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
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

/// @author vsevolod vlaskine

#ifndef SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKETS_GSOF_H_
#define SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKETS_GSOF_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <comma/base/exception.h>
#include <comma/packed/big_endian.h>
#include <comma/packed/bits.h>
#include <comma/packed/byte.h>
#include <comma/packed/struct.h>
#include "../packet.h"

namespace snark { namespace trimble { namespace bd9xx { namespace packets { namespace gsof {

struct transmission
{
    struct header: public comma::packed::packed_struct< header, 3 >
    {
        comma::packed::uint8 transmission_number;
        comma::packed::uint8 page_index;
        comma::packed::uint8 max_page_index;
    };
};

struct header : public comma::packed::packed_struct< header, 2 >
{
    comma::packed::uint8 type;
    comma::packed::uint8 length;
    
    header( unsigned char t = 0, unsigned char l = 0 ) { type = t; length = l; }
};

template < typename T >
struct record : public comma::packed::packed_struct< record< T >, header::size + T::data::size >
{
    gsof::header header;
    typename T::data body;
    
    record() : header( T::type, T::data::size ) {}
};

struct gps_time : public comma::packed::packed_struct< gps_time, 6 >
{
    comma::packed::big_endian_int32 milliseconds; // milliseconds of gps week
    comma::packed::big_endian_int16 week_number;
    
    boost::posix_time::ptime as_time() const; // todo
};

struct coordinates : public comma::packed::packed_struct< coordinates, 16 >
{
    comma::packed::big_endian_double latitude;
    comma::packed::big_endian_double longitude;
};

struct position : public comma::packed::packed_struct< position, 24 >
{
    gsof::coordinates coordinates;
    comma::packed::big_endian_double height;
};

struct quality
{
    enum values { fix_not_available_or_invalid = 0
                , autonomous_gps_fix = 1
                , differenction_sbas_or_omnistart_vbs = 2
                , rtk_fixed = 3
                , omnistart_xp_hp_centerpoint_rtx_float_rtk_location_rtk = 5 };
};

namespace records {

struct altitude
{
    enum { type = 0x1b };
    
    struct flags
    {
        unsigned char calibrated: 1,
                      pitch_valid: 1,
                      yaw_valid: 1,
                      roll_valid: 1,
                      scalar_valid: 1,
                      cobra_diagnotic_valid: 1,
                      cobra_slave_static: 1,
                      cobra_error_stats_valid: 1;
    };
    
    struct calculation_flags
    {
        unsigned char todo; // documentation unclear: http://www.trimble.com/OEM_ReceiverHelp/v4.91/en/default.html#GSOFmessages_Flags.html#Attitude%20calculation%20flags
    };
    
    struct data : public comma::packed::packed_struct< data, 70 >
    {
        comma::packed::big_endian_int32 milliseconds; // milliseconds of gps week
        comma::packed::bits< altitude::flags > flags;
        comma::packed::uint8 number_of_satellites_used;
        comma::packed::bits< altitude::calculation_flags > calculation_flags;
        comma::packed::byte reserved;
        comma::packed::big_endian_double pitch;
        comma::packed::big_endian_double yaw;
        comma::packed::big_endian_double roll;
        comma::packed::big_endian_double master_slave_range;
        comma::packed::big_endian_int16 position_dilution_of_precision;
        comma::packed::big_endian_float32 pitch_variance;
        comma::packed::big_endian_float32 yaw_variance;
        comma::packed::big_endian_float32 roll_variance;
        comma::packed::big_endian_float32 master_slave_range_variance;
    };
};

struct base_position_and_quality
{
    enum { type = 41 }; // should it be 27? documentation says 41: http://www.trimble.com/OEM_ReceiverHelp/v4.91/en/default.html#GSOFmessages_BasePositionQualityIndicator.html
    
    struct data : public comma::packed::packed_struct< data, 31 >
    {
        gsof::gps_time gps_time;
        gsof::position position;
        comma::packed::byte quality_indicator;
    };
};

struct current_time_utc
{
    enum { type = 0x10 };
    
    struct flags
    {
        unsigned char time_information_valid: 1,
                      utc_offset_valid: 1;
    };
    
    struct data : public comma::packed::packed_struct< data, 9 >
    {
        gsof::gps_time gps_time;
        comma::packed::big_endian_int16 utc_offset; // seconds
        comma::packed::bits< current_time_utc::flags > flags;
    };
};

struct detail_all_satellites; // todo

struct detail_satellite_info; // todo

struct position // llh
{
    enum { type = 0x02 };
    
    struct data : public comma::packed::packed_struct< data, 24 >
    {
        gsof::position position;
    };
};

struct position_sigma
{
    enum { type = 0x0c };
    
    struct data : public comma::packed::packed_struct< data, 38 >
    {
        comma::packed::big_endian_float32 position_rms;
        comma::packed::big_endian_float32 sigma_east; // meters
        comma::packed::big_endian_float32 sigma_north; // meters
        comma::packed::big_endian_float32 covariance_east_north;
        comma::packed::big_endian_float32 sigma_up; // meters
        comma::packed::big_endian_float32 semi_major_axis; // meters
        comma::packed::big_endian_float32 semi_minor_axis; // meters
        comma::packed::big_endian_float32 orientation;
        comma::packed::big_endian_float32 unit_variance;
        comma::packed::big_endian_uint16 number_of_epochs; // meters
    };
};

struct position_time
{
    enum { type = 0x01 };
    
    struct flags1
    {
        unsigned char is_new_position: 1,
                      clock_fix_calculated_for_this_position: 1,
                      horizontal_coordinates_calculated_for_this_position: 1,
                      height_calculated_for_this_position: 1,
                      reserved_0: 1,
                      least_squares_position: 1,
                      reserved_1: 1,
                      used_filtered_l1_pseudoranges: 1;
                      
    };
    
    struct flags2
    {
        unsigned char is_differential_solution: 1,
                      differential_position_method: 2,
                      is_omnistar_differential_solution: 1,
                      position_determined_with_static_as_constraint: 1,
                      position_is_network_rtk_solution: 1,
                      position_is_location_rtk: 1,
                      position_is_beacon_dgps: 1;
    };
    
    struct data : public comma::packed::packed_struct< data, 10 >
    {
        gsof::gps_time gps_time;
        comma::packed::uint8 number_of_satellites_used;
        comma::packed::bits< position_time::flags1 > flags1;
        comma::packed::bits< position_time::flags2 > flags2;
        comma::packed::uint8 initialized_number; // increments with each initialization
    };
};

struct received_base_info; // todo

struct position_type_info; // todo

struct velocity
{
    enum { type = 0x08 };
    
    struct flags
    {
        unsigned char valid: 1,
                      computed_from_doppler: 1;
    };
    
    struct data : public comma::packed::packed_struct< data, 17 >
    {
        comma::packed::bits< velocity::flags > flags;
        comma::packed::big_endian_float32 speed;
        comma::packed::big_endian_float32 heading;
        comma::packed::big_endian_float32 vertical_velocity;
        comma::packed::big_endian_float32 local_heading;
    };
};

struct ecef_position
{
    enum { type = 0x03 };
    
    struct data : public comma::packed::packed_struct< data, 24 >
    {
        comma::packed::big_endian_double x;
        comma::packed::big_endian_double y;
        comma::packed::big_endian_double z;
    };
};

} // namespace records {
    
} } } } } // namespace snark { namespace trimble { namespace bd9xx { namespace packets { namespace gsof {

#endif // SNARK_NAVIGATION_TRIMBLE_BD9XX_PACKETS_GSOF_H_
