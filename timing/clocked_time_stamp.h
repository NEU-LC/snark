// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef SNARK_CLOCKED_TIMESTAMP
#define SNARK_CLOCKED_TIMESTAMP

#ifndef WIN32
#include <stdlib.h>
#endif
#include "../timing/time.h"

namespace snark{ namespace timing {

/// Helps calculate a better approximation for the timestamp that happens
/// with a very precise frequency (e.g. laser firing).
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
class clocked_time_stamp
{
public:
    /// constructor, takes event period
    clocked_time_stamp( boost::posix_time::time_duration period );
    
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
    boost::posix_time::time_duration period( void ) const { return m_period; }
    
    /// reset the approximation
    void reset( void );
        
private:
    bool m_first;
    boost::posix_time::time_duration m_period;
    boost::posix_time::ptime m_last;
    boost::posix_time::time_duration m_totalDeviation;
};

class periodic_time_stamp
{
public:
    periodic_time_stamp( boost::posix_time::time_duration const& i_adjusted_period
                       , boost::posix_time::time_duration const& i_adjusted_threshold
                       , boost::posix_time::time_duration const& i_adjusted_reset );
    boost::posix_time::ptime adjusted( const boost::posix_time::ptime& t,std::size_t ticks = 1 );

    void reset( void ) { last_ = boost::posix_time::not_a_date_time; }

    boost::posix_time::time_duration const& adjusted_period( void ) { return adjusted_period_; }

    boost::posix_time::time_duration const& adjusted_threshold( void ) { return adjusted_threshold_; }

    boost::posix_time::time_duration const& adjusted_reset( void ) { return adjusted_reset_; }

private:

    boost::posix_time::time_duration adjusted_period_;
    boost::posix_time::time_duration adjusted_threshold_;
    boost::posix_time::time_duration adjusted_reset_;

    boost::posix_time::ptime last_;
    boost::posix_time::ptime last_reset_;
};

} } // namespace snark{ namespace timing

#endif /*SNARK_CLOCKED_TIMESTAMP*/
