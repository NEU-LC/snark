// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2020 Mission Systems Pty Ltd

#include "types.h"
#include "../../../math/rotation_matrix.h"
#include "../../../timing/time.h"

namespace snark { namespace ouster {

static boost::posix_time::ptime convert_timestamp( comma::uint64 timestamp )
{
    return boost::posix_time::ptime( snark::timing::epoch
                                   , boost::posix_time::microseconds( static_cast< long >( timestamp / 1000 )));
}

transform_t::transform_t( std::vector< double >& transform_vector )
{
    Eigen::Matrix4d mat = Eigen::Map< Eigen::Matrix< double, 4, 4, Eigen::RowMajor > >( transform_vector.data() );
    translation = Eigen::Translation3d( mat.block< 3, 1 >( 0, 3 ) * 0.001 );
    Eigen::Matrix3d rotation_matrix = mat.block< 3, 3 >( 0, 0 );
    rotation = snark::rotation_matrix( rotation_matrix ).roll_pitch_yaw();
}

std::vector< double > transform_t::frame() const
{
    return std::vector< double >({ translation.x(), translation.y(), translation.z()
                                 , rotation.roll(), rotation.pitch(), rotation.yaw()
                                 });
}

output_azimuth_block_t::output_azimuth_block_t( const OS1::azimuth_block_t& azimuth_block
                                              , comma::uint32 block_id )
    : t( convert_timestamp( azimuth_block.timestamp() ))
    , measurement_id( azimuth_block.measurement_id() )
    , encoder_count( azimuth_block.encoder_count() )
    , block_id( block_id )
{
}

output_data_block_t::output_data_block_t( double azimuth_encoder_angle
                                        , const OS1::data_block_t& data_block
                                        , comma::uint16 channel
                                        , const OS1::beam_angle_lut_t& beam_angle_lut )
    : channel( channel )
    , signal( data_block.signal() )
    , reflectivity( data_block.reflectivity() )
    , ambient( data_block.noise() )
{
    // Turn the left-hand curl bearing orientation into right-hand curl,
    // and switch from lidar frame to sensor frame.
    // See §4.1 Sensor Coordinate Frame in the Software User Guide
    range = static_cast< double >( data_block.range() & 0x000FFFFF ) / 1000;
    bearing = M_PI * 2 - ( azimuth_encoder_angle + beam_angle_lut[ channel ].azimuth ) - M_PI;
    elevation = beam_angle_lut[ channel ].altitude;
}

output_imu_t::output_imu_t( const OS1::imu_block_t& imu_block )
    : start_time( convert_timestamp( imu_block.start_read_time() ))
    , acceleration( convert_timestamp( imu_block.acceleration_read_time() )
                  , Eigen::Vector3d( imu_block.acceleration_x()
                                   , imu_block.acceleration_y()
                                   , imu_block.acceleration_z() ))
    , angular_acceleration( convert_timestamp( imu_block.gyro_read_time() )
                          , Eigen::Vector3d( imu_block.angular_acceleration_x() * M_PI / 180
                                           , imu_block.angular_acceleration_y() * M_PI / 180
                                           , imu_block.angular_acceleration_z() * M_PI / 180 ))
{
}

} } // namespace snark { namespace ouster {
