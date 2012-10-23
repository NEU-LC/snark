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

#include <comma/base/exception.h>
#include "./stdin_reader.h"
#include <snark/timing/time.h>

namespace snark {

stdin_reader::stdin_reader():
    m_epoch( timing::epoch )
{
}

const char* stdin_reader::read()
{
    std::cin.read( reinterpret_cast< char* >( &m_microseconds ), sizeof( m_microseconds ) );
    std::cin.read( m_packet.data(), payload_size );
    if( std::cin.eof() )
    {
        return NULL;        
    }
    m_timestamp = m_epoch + boost::posix_time::microseconds( m_microseconds );
    return &m_packet[0];
}


const boost::posix_time::ptime& stdin_reader::timestamp() const
{
    return m_timestamp;    
}

} // namespace snark {

