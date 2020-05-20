// Copyright (c) 2019 The University of Sydney

#pragma once

#include "config.h"
#include "packet.h"
#include "types.h"
#include <comma/visiting/traits.h>

namespace comma { namespace visiting {

template <> struct traits< ouster::OS1::parameters_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::OS1::parameters_t& t, Visitor& v )
    {
        v.apply( "auto_start_flag", t.auto_start_flag );
        v.apply( "tcp_port", t.tcp_port );
        v.apply( "udp_ip", t.udp_ip );
        v.apply( "udp_port_lidar", t.udp_port_lidar );
        v.apply( "udp_port_imu", t.udp_port_imu );
        v.apply( "timestamp_mode", t.timestamp_mode );
        v.apply( "pps_out_mode", t.pps_out_mode );
        v.apply( "pps_out_polarity", t.pps_out_polarity );
        v.apply( "pps_rate", t.pps_rate );
        v.apply( "pps_angle", t.pps_angle );
        v.apply( "pps_pulse_width", t.pps_pulse_width );
        v.apply( "pps_in_polarity", t.pps_in_polarity );
        v.apply( "lidar_mode", t.lidar_mode );
        v.apply( "pulse_mode", t.pulse_mode );
        v.apply( "window_rejection_enable", t.window_rejection_enable );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::OS1::parameters_t& t, Visitor& v )
    {
        v.apply( "auto_start_flag", t.auto_start_flag );
        v.apply( "tcp_port", t.tcp_port );
        v.apply( "udp_ip", t.udp_ip );
        v.apply( "udp_port_lidar", t.udp_port_lidar );
        v.apply( "udp_port_imu", t.udp_port_imu );
        v.apply( "timestamp_mode", t.timestamp_mode );
        v.apply( "pps_out_mode", t.pps_out_mode );
        v.apply( "pps_out_polarity", t.pps_out_polarity );
        v.apply( "pps_rate", t.pps_rate );
        v.apply( "pps_angle", t.pps_angle );
        v.apply( "pps_pulse_width", t.pps_pulse_width );
        v.apply( "pps_in_polarity", t.pps_in_polarity );
        v.apply( "lidar_mode", t.lidar_mode );
        v.apply( "pulse_mode", t.pulse_mode );
        v.apply( "window_rejection_enable", t.window_rejection_enable );
    }
};

template <> struct traits< ouster::OS1::beam_intrinsics_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::OS1::beam_intrinsics_t& t, Visitor& v )
    {
        v.apply( "beam_altitude_angles", t.beam_altitude_angles );
        v.apply( "beam_azimuth_angles", t.beam_azimuth_angles );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::OS1::beam_intrinsics_t& t, Visitor& v )
    {
        v.apply( "beam_altitude_angles", t.beam_altitude_angles );
        v.apply( "beam_azimuth_angles", t.beam_azimuth_angles );
    }
};

template <> struct traits< ouster::OS1::imu_intrinsics_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::OS1::imu_intrinsics_t& t, Visitor& v )
    {
        v.apply( "imu_to_sensor_transform", t.imu_to_sensor_transform );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::OS1::imu_intrinsics_t& t, Visitor& v )
    {
        v.apply( "imu_to_sensor_transform", t.imu_to_sensor_transform );
    }
};

template <> struct traits< ouster::OS1::lidar_intrinsics_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::OS1::lidar_intrinsics_t& t, Visitor& v )
    {
        v.apply( "lidar_to_sensor_transform", t.lidar_to_sensor_transform );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::OS1::lidar_intrinsics_t& t, Visitor& v )
    {
        v.apply( "lidar_to_sensor_transform", t.lidar_to_sensor_transform );
    }
};

template <> struct traits< ouster::OS1::config_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::OS1::config_t& t, Visitor& v )
    {
        v.apply( "parameters", t.parameters );
        v.apply( "prod_sn", t.serial_number );
        v.apply( "beam_intrinsics", t.beam_intrinsics );
        v.apply( "imu_intrinsics", t.imu_intrinsics );
        v.apply( "lidar_intrinsics", t.lidar_intrinsics );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::OS1::config_t& t, Visitor& v )
    {
        v.apply( "parameters", t.parameters );
        v.apply( "prod_sn", t.serial_number );
        v.apply( "beam_intrinsics", t.beam_intrinsics );
        v.apply( "imu_intrinsics", t.imu_intrinsics );
        v.apply( "lidar_intrinsics", t.lidar_intrinsics );
    }
};

template <> struct traits< ouster::OS1::data_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::OS1::data_block_t& t, Visitor& v )
    {
        v.apply( "range", t.range );
        v.apply( "reflectivity", t.reflectivity );
        v.apply( "signal", t.signal );
        v.apply( "noise", t.noise );
        v.apply( "unused", t.unused );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::OS1::data_block_t& t, Visitor& v )
    {
        v.apply( "range", t.range );
        v.apply( "reflectivity", t.reflectivity );
        v.apply( "signal", t.signal );
        v.apply( "noise", t.noise );
        v.apply( "unused", t.unused );
    }
};

template <> struct traits< ouster::OS1::azimuth_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::OS1::azimuth_block_t& t, Visitor& v )
    {
        v.apply( "timestamp", t.timestamp );
        v.apply( "measurement_id", t.measurement_id );
        v.apply( "encoder_count", t.encoder_count );
        v.apply( "data_blocks", t.data_blocks );
        v.apply( "packet_status", t.packet_status );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::OS1::azimuth_block_t& t, Visitor& v )
    {
        v.apply( "timestamp", t.timestamp );
        v.apply( "measurement_id", t.measurement_id );
        v.apply( "encoder_count", t.encoder_count );
        v.apply( "data_blocks", t.data_blocks );
        v.apply( "packet_status", t.packet_status );
    }
};

template < typename T > struct traits< ouster::timed_xyz_t< T > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::timed_xyz_t< T >& t, Visitor& v )
    {
        v.apply( "t", t.time );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::timed_xyz_t< T >& t, Visitor& v )
    {
        v.apply( "t", t.time );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }
};

template <> struct traits< ouster::OS1::imu_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::OS1::imu_block_t& t, Visitor& v )
    {
        v.apply( "start_read_time", t.start_read_time );
        v.apply( "acceleration_read_time", t.acceleration_read_time );
        v.apply( "gyro_read_time", t.gyro_read_time );
        v.apply( "acceleration_x", t.acceleration_x );
        v.apply( "acceleration_y", t.acceleration_y );
        v.apply( "acceleration_z", t.acceleration_z );
        v.apply( "angular_acceleration_x", t.angular_acceleration_x );
        v.apply( "angular_acceleration_y", t.angular_acceleration_y );
        v.apply( "angular_acceleration_z", t.angular_acceleration_z );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::OS1::imu_block_t& t, Visitor& v )
    {
        v.apply( "start_read_time", t.start_read_time );
        v.apply( "acceleration_read_time", t.acceleration_read_time );
        v.apply( "gyro_read_time", t.gyro_read_time );
        v.apply( "acceleration_x", t.acceleration_x );
        v.apply( "acceleration_y", t.acceleration_y );
        v.apply( "acceleration_z", t.acceleration_z );
        v.apply( "angular_acceleration_x", t.angular_acceleration_x );
        v.apply( "angular_acceleration_y", t.angular_acceleration_y );
        v.apply( "angular_acceleration_z", t.angular_acceleration_z );
    }
};

template <> struct traits< ouster::output_azimuth_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::output_azimuth_block_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "measurement_id", t.measurement_id );
        v.apply( "encoder_count", t.encoder_count );
        v.apply( "block", t.block_id );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::output_azimuth_block_t& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "measurement_id", t.measurement_id );
        v.apply( "encoder_count", t.encoder_count );
        v.apply( "block", t.block_id );
    }
};

template <> struct traits< ouster::output_data_block_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::output_data_block_t& t, Visitor& v )
    {
        v.apply( "channel", t.channel );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
        v.apply( "signal", t.signal );
        v.apply( "reflectivity", t.reflectivity );
        v.apply( "ambient", t.ambient );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::output_data_block_t& t, Visitor& v )
    {
        v.apply( "channel", t.channel );
        v.apply( "range", t.range );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
        v.apply( "signal", t.signal );
        v.apply( "reflectivity", t.reflectivity );
        v.apply( "ambient", t.ambient );
    }
};

template <> struct traits< ouster::output_lidar_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::output_lidar_t& t, Visitor& v )
    {
        v.apply( "azimuth_block", t.azimuth_block );
        v.apply( "data_block", t.data_block );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::output_lidar_t& t, Visitor& v )
    {
        v.apply( "azimuth_block", t.azimuth_block );
        v.apply( "data_block", t.data_block );
    }
};

template <> struct traits< ouster::output_imu_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ouster::output_imu_t& t, Visitor& v )
    {
        v.apply( "start_time", t.start_time );
        v.apply( "acceleration", t.acceleration );
        v.apply( "angular_acceleration", t.angular_acceleration );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ouster::output_imu_t& t, Visitor& v )
    {
        v.apply( "start_time", t.start_time );
        v.apply( "acceleration", t.acceleration );
        v.apply( "angular_acceleration", t.angular_acceleration );
    }
};

} } // namespace comma { namespace visiting {
