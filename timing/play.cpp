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


#include <snark/timing/play.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/thread_time.hpp>


namespace snark
{
namespace timing
{
    
/// constructor    
play::play( double speed, bool quiet, const boost::posix_time::time_duration& precision ):
    m_speed( speed ),
    m_precision( precision ),
    m_lag( false ),
    m_lagCounter( 0U ),
    m_quiet( quiet )
{
}

/// constructor
/// @param first first timestamp
/// @param speed slow-down factor: 1.0 = real time, 2.0 = twice as slow etc...
/// @param quiet if true, do not output warnings if we can not keep up with the desired playback speed
/// @param precision expected precision from the sleep function
play::play( const boost::posix_time::ptime& first, double speed, bool quiet, const boost::posix_time::time_duration& precision ):

    m_systemFirst( boost::get_system_time() ),
    m_offset( m_systemFirst - first ),
    m_first( first ),
    m_last( first ),
    m_speed( speed ),
    m_precision( precision ),
    m_lag( false ),
    m_lagCounter( 0U ),
    m_quiet( quiet )
{
    
}


/// wait until a timestamp
/// @param time timestamp as ptime
void play::wait( const boost::posix_time::ptime& time )
{

    if ( !m_offset )
    {
        boost::posix_time::ptime systemTime = boost::get_system_time();
        m_offset = systemTime - time;
        m_systemFirst = systemTime;
        m_first = time;
        m_last = time;
    }
    else
    {        
        if ( time > m_last )
        {
            boost::posix_time::ptime systemTime = boost::get_system_time();
            const boost::posix_time::ptime target = m_systemFirst + boost::posix_time::milliseconds( static_cast<long>(( time - m_first ).total_milliseconds() * m_speed ) );
            const boost::posix_time::time_duration lag = systemTime - target;
            if ( !m_quiet && ( lag > m_precision ) ) // no need to be alarmed for a lag less than the expected accuracy
            {
                if( !m_lag )
                {
                    m_lag = true;
                    std::cerr << "csv-play: warning, lagging behind " << lag << std::endl;
                }
                m_lagCounter++;
            }
            else
            {
                if( !m_quiet && m_lag )
                {
                    m_lag = false;
                    std::cerr << "csv-play: recovered after " << m_lagCounter << " packets " << std::endl;
                    m_lagCounter = 0U;
                }
                if ( lag < -m_precision ) // no need to sleep less than the expected accuracy
                {
                    boost::this_thread::sleep( target );
                }
            }
            m_last = time;
        }
        else
        {
            // timestamp same or earlier than last time, nothing to do
        }
    }
}


/// wait until a timestamp
/// @param isoTime timestamp in iso format
void play::wait( const std::string& iso_time )
{
    wait( boost::posix_time::from_iso_string( iso_time ) );
}




}
}
