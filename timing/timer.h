// This file is part of snark, a generic and flexible library
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

#ifndef SNARK_TIMING_TIMER_H_
#define SNARK_TIMING_TIMER_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread_time.hpp>

namespace snark {

/// simple timer    
class timer
{
public:
    timer( const boost::posix_time::time_duration& duration );
    void reset();
    bool expired() const;
private:
    boost::posix_time::time_duration m_duration;
    boost::posix_time::ptime m_end;    
};

} // namespace snrk {

#endif /*SNARK_TIMING_TIMER_H_*/
