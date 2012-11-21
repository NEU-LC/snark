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

#ifndef SNARK_SENSORS_VELODYNE_SCAN_TICK_H_
#define SNARK_SENSORS_VELODYNE_SCAN_TICK_H_

#include <snark/sensors/velodyne/packet.h>
#include <snark/sensors/velodyne/impl/get_laser_return.h>

namespace snark {  namespace velodyne {

class scan_tick
{
public:
    scan_tick();
    bool get( const packet& packet );

private:
    bool m_first;
    unsigned int m_angle;
    unsigned int m_last_angle;
};

} } // namespace snark {  namespace velodyne {

#endif /*SNARK_SENSORS_VELODYNE_STREAM_H_*/
