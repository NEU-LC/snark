// Copyright (c) 2019 The University of Sydney

#pragma once

#include <string>
#include <vector>
#include <comma/base/types.h>

namespace ouster { namespace OS1 {

struct parameters_t
{
    bool auto_start_flag;
    comma::uint16 tcp_port;
    comma::uint16 udp_ip;
    comma::uint16 udp_port_lidar;
    comma::uint16 udp_port_imu;
    std::string timestamp_mode;
    std::string pps_out_mode;
    std::string pps_out_polarity;
    comma::uint16 pps_rate;
    comma::uint16 pps_angle;
    comma::uint16 pps_pulse_width;
    std::string pps_in_polarity;
    std::string lidar_mode;
    std::string pulse_mode;
    bool window_rejection_enable;
};

struct beam_intrinsics_t
{
    std::vector< double > beam_altitude_angles;
    std::vector< double > beam_azimuth_angles;
};

struct imu_intrinsics_t
{
    std::vector< double > imu_to_sensor_transform;
};

struct lidar_intrinsics_t
{
    std::vector< double > lidar_to_sensor_transform;
};

struct config_t
{
    parameters_t parameters;
    std::string serial_number;
    beam_intrinsics_t beam_intrinsics;
    imu_intrinsics_t imu_intrinsics;
    lidar_intrinsics_t lidar_intrinsics;
};

} } // namespace ouster { namespace OS1 {
