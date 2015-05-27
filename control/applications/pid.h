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

#ifndef SNARK_CONTROL_PID_HEADER
#define SNARK_CONTROL_PID_HEADER

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>

namespace snark { namespace control {

enum derivative_mode { internal, external };

template< derivative_mode M >
class pid
{
public:
    pid( double p, double i, double d ) : p( p ), i( i ), d( d ), integral( 0 ) {}
    pid( double p, double i, double d, boost::optional< double > threshold ) : p( p ), i( i ), d( d ), integral( 0 ), threshold( threshold ) {}
    double time_increment( boost::posix_time::ptime  t )
    {
        return ( t - time ).total_microseconds() / 1e6;
    }
    void clip( double& value )
    {
        if( threshold )
        {
            if ( value > *threshold ) { value = *threshold; }
            else if ( value < -*threshold ) { value = -*threshold; }
        }
    }
    double update( double error, const boost::posix_time::ptime& t );
    double update( double error, const boost::posix_time::ptime& t, const double& derivative )
    {
        if( time != boost::posix_time::not_a_date_time && t != boost::posix_time::not_a_date_time )
        {
            if( t < time ) { COMMA_THROW( comma::exception, "expected time greater than " << boost::posix_time::to_iso_string( time ) << ", got " << boost::posix_time::to_iso_string( t ) ); }
            integral = time_increment( t ) * error;
            clip( integral );
        }
        time = t;
        return p * error + i * integral + d * derivative;
    }
    void reset()
    {
        integral = 0;
        previous_error = boost::none;
        time = boost::posix_time::not_a_date_time;
    }

private:
    double p;
    double i;
    double d;
    double integral;
    boost::optional< double > threshold;
    boost::optional< double > previous_error;
    boost::posix_time::ptime time;
};

template<> double pid< internal >::update( double error, const boost::posix_time::ptime& t )
{
    if( !previous_error )
    {
        previous_error = error;
        return update( error, t, 0 );
    }
    if( time == boost::posix_time::not_a_date_time || t == boost::posix_time::not_a_date_time )
    {
        return update( error, t, 0 );
    }
    double derivative =  ( error - *previous_error ) / time_increment( t );
    previous_error = error;
    return update( error, t, derivative );
}

template<> double pid< external >::update( double error, const boost::posix_time::ptime& t )
{
    return update( error, t, 0 );
}

} } // namespace snark { namespace control {

#endif // SNARK_CONTROL_PID_HEADER
