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

#ifndef SNARK_TIMING_PLAY_H
#define SNARK_TIMING_PLAY_H

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace snark
{
namespace timing
{

/// play back timestamped data in a real time manner
class play
{
public:
    play( double speed = 1.0, bool quiet = false, const boost::posix_time::time_duration& precision = boost::posix_time::milliseconds(1) );
    play( const boost::posix_time::ptime& first, double speed = 1.0, bool quiet = false, const boost::posix_time::time_duration& precision = boost::posix_time::milliseconds(1) );

    void wait( const boost::posix_time::ptime& time );

    void wait( const std::string& iso_time );

private:
    boost::posix_time::ptime m_systemFirst; /// system time at first timestamp
    boost::optional< boost::posix_time::time_duration > m_offset; /// offset between timestamps and system time
    boost::posix_time::ptime m_first; /// first timestamp
    boost::posix_time::ptime m_last; /// last timestamp received
    const double m_speed;
    const boost::posix_time::time_duration m_precision;
    bool m_lag;
    unsigned int m_lagCounter;
    bool m_quiet;
};

}
}

#endif // SNARK_TIMING_PLAY_H
