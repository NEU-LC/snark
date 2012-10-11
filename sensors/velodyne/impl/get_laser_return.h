#ifndef SNARK_SENSORS_VELODYNE_IMPL_GETLASERRETURN_H_
#define SNARK_SENSORS_VELODYNE_IMPL_GETLASERRETURN_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <snark/sensors/velodyne/db.h>
#include <snark/sensors/velodyne/laser_return.h>
#include <snark/sensors/velodyne/packet.h>

namespace snark {  namespace velodyne { namespace impl {

laser_return getlaser_return( const packet& packet
                        , unsigned int block
                        , unsigned int laser
                        , const boost::posix_time::ptime& timestamp
                        , double angularSpeed
                        , bool raw = false );

boost::posix_time::time_duration time_offset( unsigned int block, unsigned int laser );

double azimuth( const packet& packet, unsigned int block, unsigned int laser, double angularSpeed );

double azimuth( double rotation, unsigned int laser, double angularSpeed );

} } } // namespace snark {  namespace velodyne { namespace impl {

#endif // SNARK_SENSORS_VELODYNE_IMPL_GETFROMLASERRETURN_H_
