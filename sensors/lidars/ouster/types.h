// Copyright (c) 2019 The University of Sydney

#pragma once

#include "packet.h"
#include <snark/math/roll_pitch_yaw.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <Eigen/Geometry>

namespace ouster {

struct transform_t
{
    Eigen::Translation3d translation;
    snark::roll_pitch_yaw rotation;

    transform_t( const Eigen::Vector3d& translation_, const snark::roll_pitch_yaw& rotation_ )
        : translation( translation_ )
        , rotation( rotation_ )
    {}

    transform_t( std::vector< double >& transform_vector );

    std::vector< double > frame() const;
};

struct output_azimuth_block_t
{
    boost::posix_time::ptime t;
    comma::uint32 measurement_id;
    comma::uint32 encoder_count;
    comma::uint32 block_id;

    output_azimuth_block_t() {}

    output_azimuth_block_t( const ouster::OS1::azimuth_block_t& azimuth_block
                          , comma::uint32 block_id );
};

struct output_data_block_t
{
    comma::uint16 channel;
    double range;
    double bearing;
    double elevation;
    comma::uint16 signal;
    comma::uint16 reflectivity;
    comma::uint16 ambient;

    output_data_block_t() {}

    output_data_block_t( double azimuth_encoder_angle
                       , const ouster::OS1::data_block_t& data_block
                       , comma::uint16 channel );
};

struct output_lidar_t
{
    output_azimuth_block_t azimuth_block;
    output_data_block_t data_block;

    output_lidar_t() {}

    output_lidar_t( const output_azimuth_block_t& a, const output_data_block_t& d )
        : azimuth_block( a )
        , data_block( d )
    {}
};

template < typename T >
struct timed_xyz_t
{
    boost::posix_time::ptime time;
    T x;
    T y;
    T z;

    timed_xyz_t() {}

    timed_xyz_t( boost::posix_time::ptime time, T x, T y, T z )
        : time( time ), x( x ), y( y ), z( z ) {}
};

struct output_imu_t
{
    boost::posix_time::ptime start_time;
    timed_xyz_t< double > acceleration;
    timed_xyz_t< double > angular_acceleration;

    output_imu_t() {}

    output_imu_t( const ouster::OS1::imu_block_t& imu_block );
};

} // namespace ouster {
