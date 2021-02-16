// Copyright (c) 2021 Mission Systems Pty Ltd

#pragma once

#include <inno_lidar_api.h>
#include <comma/base/types.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>

namespace snark { namespace innovusion {

// representation of inno_frame
struct frame_t
{
    comma::uint64 idx;
    comma::uint16 sub_idx;
    comma::uint16 sub_seq;
    double ts_us_start;
    double ts_us_end;
    comma::uint32 points_number;
    unsigned char conf_level;
    unsigned char timestamp_sync;

    frame_t();
    frame_t( const inno_frame* frame );
};

// representation of inno_point
struct point_t
{
    float x;
    float y;
    float z;
    float radius;
    comma::uint16 ts_100us;
    comma::uint16 value;                // reflectance or intensity: 1-255
    comma::uint16 flags;
    comma::uint16 channel;              // 0 upper, 1 lower
    comma::uint16 scan_id;
    comma::uint16 scan_idx;

    point_t();
    point_t( const inno_point* point );
};

// subset of the point and frame data
struct output_data_t
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    float x;
    float y;
    float z;
    float radius;
    comma::uint16 value;                // reflectance or intensity

    output_data_t();
    output_data_t( const inno_frame* frame, unsigned int index );
};

// all of the point and frame data
struct output_data_full_t
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    frame_t frame;
    point_t point;

    output_data_full_t();
    output_data_full_t( const inno_frame* frame, unsigned int index );
};

std::string alarm_type_to_string( inno_alarm alarm_type );
std::string alarm_code_to_string( inno_alarm_code alarm_code );
std::string timestamp_sync_to_string( inno_timestamp_sync timestamp_sync );

} } // namespace snark { namespace innovusion {
