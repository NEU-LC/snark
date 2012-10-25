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

#include <cassert>
#include "clocked_time_stamp.h"

namespace snark{ namespace timing {

/// assumptions:
/// - the system performance is enough to catch up at least once in a while
/// - here we discount the lower latency boundary below which the system cannot go
class clocked_time_stamp::impl
{
    public:
        impl( boost::posix_time::time_duration period )
            : m_period( period )
            , m_first( true )
            , m_totalDeviation( boost::posix_time::seconds( 0 ) )
        {
        }
        
        boost::posix_time::ptime adjusted( boost::posix_time::ptime t, std::size_t ticks  )
        {
            assert( ticks );
            if( m_first )
            {
                m_first = false;
            }
            else
            {
                // nothing more than t - t0 - sum( periods )
                m_totalDeviation += ( t - m_last - m_period * ticks );
                // whenever we get a better approximation, we take it as a better approximation
                if( m_totalDeviation.is_negative() ) { m_totalDeviation = boost::posix_time::seconds( 0 ); }
            }
            m_last = t;
            // first m_totalDeviation = 0, i.e. we consider t the best approximation available
            return t - m_totalDeviation;
        }
        
        boost::posix_time::ptime adjusted( boost::posix_time::ptime t, boost::posix_time::time_duration period, std::size_t ticks  )
        {
            m_period = period;
            return adjusted( t, ticks );
        }
        
        void reset()
        {
            m_first = true;
            m_totalDeviation = boost::posix_time::seconds( 0 );
        }
        
        boost::posix_time::time_duration m_period;
        bool m_first;
        boost::posix_time::ptime m_last;
        boost::posix_time::time_duration m_totalDeviation;
};

clocked_time_stamp::clocked_time_stamp( boost::posix_time::time_duration period )
{
    m_pimpl = new impl( period );
}

clocked_time_stamp::~clocked_time_stamp()
{
    delete m_pimpl;
}
        
boost::posix_time::ptime clocked_time_stamp::adjusted( const boost::posix_time::ptime& t, std::size_t ticks )
{
    return m_pimpl->adjusted( t, ticks );
}

boost::posix_time::ptime clocked_time_stamp::adjusted( const boost::posix_time::ptime& t, boost::posix_time::time_duration period, std::size_t ticks )
{
    return m_pimpl->adjusted( t, period, ticks );
}

boost::posix_time::time_duration clocked_time_stamp::period() const { return m_pimpl->m_period; }

void clocked_time_stamp::reset(){m_pimpl->reset();}
    
} } // namespace snark{ namespace timing

