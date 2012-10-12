// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

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
