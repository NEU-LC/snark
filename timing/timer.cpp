// This file is part of Ark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// Ark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Ark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with Ark. If not, see <http://www.gnu.org/licenses/>.

#include <snark/timing/timer.h>

namespace snark {

/// constructor
/// @param duration the timer duration
timer::timer(const boost::posix_time::time_duration& duration):
    m_duration( duration )    
{

}

/// return if the timer has expired
bool timer::expired() const
{
    if( m_end == boost::posix_time::not_a_date_time )
    {
        return false;
    }
    else
    {
        return ( boost::get_system_time() >= m_end );
    }
}

/// restart the timer
void timer::reset()
{
    m_end = boost::get_system_time() + m_duration;
}




} // namespace snark{ 

