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

namespace snark {

stdin_reader::stdin_reader()
{
}

const char* stdin_reader::read()
{
    std::cin.read( packet_.data(), payload_size );
    if( std::cin.eof() )
    {
        return NULL;        
    }
    timestamp_ = boost::posix_time::microsec_clock::universal_time();
    return &packet_[0];
}


const boost::posix_time::ptime& stdin_reader::timestamp() const
{
    return timestamp_;    
}

} // namespace snark {

