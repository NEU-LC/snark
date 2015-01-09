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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the University of Sydney.
// 4. Neither the name of the University of Sydney nor the
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
