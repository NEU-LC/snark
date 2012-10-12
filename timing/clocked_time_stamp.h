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

#ifndef SNARK_CLOCKED_TIMESTAMP
#define SNARK_CLOCKED_TIMESTAMP

#ifndef WIN32
#include <stdlib.h>
#endif
#include <boost/noncopyable.hpp>
#include <snark/timing/time.h>

namespace snark{ namespace timing {

/// Helps calculate a better approximation for the timestamp that happens
/// with a very frequency (e.g. laser firing).
///
/// The first few timestamps may be inaccurate, but eventually (quite fast,
/// if the system is not under heavy load) the adjusted timestamp will
/// converge to the true time of event plus a constant system latency (e.g.
/// time to receive a data packet through a certain bandwidth). 
///
/// Assumptions:  
/// - The event frequency is stable enough, e.g. it's a bus clock
/// - Minimum system latency is reasonably stable
/// - System performance is enough to catch up at least once in a while 
class clocked_time_stamp : public boost::noncopyable
{
    public:
        /// constructor, takes event period
        clocked_time_stamp( boost::posix_time::time_duration period );
        
        /// destructor
        ~clocked_time_stamp();
        
        /// take event reception timestamp and return adjusted timestamp;
        /// ticks are the number of clock cycles between this event and the previous one
        boost::posix_time::ptime adjusted( const boost::posix_time::ptime& t, std::size_t ticks = 1 );
        
        /// take event reception timestamp and return adjusted timestamp,
        /// using new value for the period;
        /// ticks are the number of clock cycles between this event and the previous one
        /// @note: this method accounts for small slow variations
        ///        of the period over long time, for example variations of
        ///        clocking frequency due to the change of the ambient
        ///        temperature
        boost::posix_time::ptime adjusted( const boost::posix_time::ptime& t, boost::posix_time::time_duration period, std::size_t ticks = 1 );
        
        /// return current period
        boost::posix_time::time_duration period() const;
        
        /// reset the approximation
        void reset();
            
    private:
        class impl;
        impl* m_pimpl;
};

} } // namespace snark{ namespace timing

#endif /*SNARK_CLOCKED_TIMESTAMP*/
