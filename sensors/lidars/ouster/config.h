// Copyright (c) 2019 The University of Sydney

#pragma once

#include <string>
#include <vector>
#include <comma/base/types.h>

namespace snark { namespace ouster { namespace OS1 {

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
    beam_intrinsics_t beam_intrinsics;
    imu_intrinsics_t imu_intrinsics;
    lidar_intrinsics_t lidar_intrinsics;
};

} } } // namespace snark { namespace ouster { namespace OS1 {
