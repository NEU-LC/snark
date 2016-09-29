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

#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/math/cyclic.h>
#include "pid.h"
#include "wrap_angle.h"

namespace snark { namespace control {

pid::pid() : p( 0 ), i( 0 ), d( 0 ), integral( 0 ) {}
pid::pid( double p, double i, double d ) : p( p ), i( i ), d( d ), integral( 0 ) {}
pid::pid( double p, double i, double d, double threshold ) : p( p ), i( i ), d( d ), integral( 0 ), threshold( threshold )
{
    if( threshold <= 0 ) COMMA_THROW( comma::exception, "expected positive threshold, got " << threshold );
}

double pid::derivative_( double error, double dt ) { return ( error - *previous_error ) / dt; }

double pid::operator()( double error, const boost::posix_time::ptime& t )
{
    boost::optional< double > dt = get_time_increment( t );
    return update_( error, ( previous_error && dt ) ? derivative_( error, *dt ) : 0, t, dt );
}

double pid::operator()( double error, double derivative, const boost::posix_time::ptime& t )
{
    boost::optional< double > dt = get_time_increment( t );
    return update_( error, derivative, t, dt );
}

void pid::reset()
{
    integral = 0;
    previous_error = boost::none;
    time = boost::posix_time::not_a_date_time;
}

boost::optional< double > pid::get_time_increment( const boost::posix_time::ptime& t )
{
    if( time == boost::posix_time::not_a_date_time || t == boost::posix_time::not_a_date_time ) { return boost::none; }
    if( t <= time ) { COMMA_THROW( comma::exception, "expected time greater than " << boost::posix_time::to_iso_string( time ) << ", got " << boost::posix_time::to_iso_string( t ) ); }
    return ( t - time ).total_microseconds() / 1e6;
}

void pid::clip_integral()
{
    if( !threshold ) { return; }
    if ( integral > *threshold ) { integral = *threshold; }
    else if ( integral < -*threshold ) { integral = -*threshold; }
}

double pid::update_( double error, double derivative, const boost::posix_time::ptime& t, boost::optional< double > dt )
{
    if( dt )
    {
        integral += error * *dt;
        clip_integral();
    }
    previous_error = error;
    time = t;
    return p * error + i * integral + d * derivative;
}

angular_pid::angular_pid( double p, double i, double d ) : pid( p, i, d ) {}

angular_pid::angular_pid( double p, double i, double d, double threshold ) : pid( p, i, d, threshold ) {}

double angular_pid::update_( double error, double derivative, const boost::posix_time::ptime& t, boost::optional< double > dt )
{
    if( dt )
    {
        integral += error * *dt;
        integral = wrap_angle( integral );
        clip_integral();
    }
    previous_error = error;
    time = t;
    return p * error + i * integral + d * derivative;
}

double angular_pid::derivative_( double error, double dt ) { return wrap_angle( ( error - *previous_error ) / dt ); }

} } // namespace snark { namespace control {
