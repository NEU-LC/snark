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

#include "scan.h"
#include <comma/math/compare.h>

namespace snark {  namespace velodyne { namespace thin {

scan::scan():
    m_scan( 0 ),
    m_output( 0 ),
    m_outputCurrentscan( true ),
    m_empty( false )
{
}

void scan::thin ( velodyne::packet& packet, double rate, double angularSpeed )
{
    m_empty = true;
    for( index i; i.idx < 12 * 32 ; ++i )
    {
        bool valid = packet.blocks[i.block].lasers[i.laser].range() > 0;
        if( !valid ) { continue; }
        if( m_tick.get( packet ) )
        {
            ++m_scan;
            m_outputCurrentscan = ( m_output == 0 ) || ( m_output < rate * m_scan );
            if( m_outputCurrentscan ) { m_output++; }
        }
        if( m_outputCurrentscan )
        {
            m_empty = false;
        }
        else
        {
            packet.blocks[i.block].lasers[i.laser].range = 0;
        }
    }
}



} } } // namespace snark {  namespace velodyne { namespace thin {
