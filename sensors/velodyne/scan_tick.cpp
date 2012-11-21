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

#include <snark/sensors/velodyne/scan_tick.h>

namespace snark {  namespace velodyne {

scan_tick::scan_tick():
    m_first( true ),
    m_angle( 0 ),
    m_last_angle( 0 )
{

}

bool scan_tick::get ( const packet& packet )
{
    bool tick = false;
    m_angle = packet.blocks[0].rotation() + 9000; // 0 = behind the vehicle
    if( m_angle > 36000 ){ m_angle -= 36000; }
    if( m_angle < m_last_angle || m_first ) // new scan?
    {
        m_first = false;
        tick = true;
    }
    m_last_angle = m_angle;
    return tick;
}

    
} }