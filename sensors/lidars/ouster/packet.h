// Copyright (c) 2019 The University of Sydney

#pragma once

#include "config.h"
#include <comma/base/types.h>
#include <array>

namespace ouster { namespace OS1 {

const comma::uint32 packet_status_good = 0xffffffff;
const comma::uint32 packet_status_bad = 0;

const std::size_t pixels_per_column = 64; // TODO: determine from config, to support 16 beam lidar
const unsigned int encoder_ticks_per_rev = 90112;

// it would be nice to use comma::packed for this structure but it doesn't play
// well with reading via input_stream
struct data_block_t
{
    comma::uint32 range;
    comma::uint16 reflectivity;
    comma::uint16 signal;
    comma::uint16 noise;
    comma::uint16 unused;

    data_block_t() {}
};

struct azimuth_block_t
{
    comma::uint64 timestamp;
    comma::uint32 measurement_id;
    comma::uint32 encoder_count;
    std::array< data_block_t, pixels_per_column > data_blocks;
    comma::uint32 packet_status;

    azimuth_block_t() {}
};

struct imu_block_t
{
    comma::uint64 start_read_time;
    comma::uint64 acceleration_read_time;
    comma::uint64 gyro_read_time;
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float angular_acceleration_x;
    float angular_acceleration_y;
    float angular_acceleration_z;

    imu_block_t() {}
};

struct beam_angle_lut_entry
{
    double altitude;
    double azimuth;
};

extern beam_angle_lut_entry beam_angle_lut[ ouster::OS1::pixels_per_column ];
void init_beam_angle_lut( const beam_intrinsics_t& beam_intrinsics );

} } // namespace ouster { namespace OS1 {
