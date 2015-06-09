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
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/string/split.h>

namespace snark { namespace control {

class pid
{
public:
    pid( double p, double i, double d ) : p( p ), i( i ), d( d ), integral( 0 ) {}
    pid( double p, double i, double d, boost::optional< double > threshold ) : p( p ), i( i ), d( d ), integral( 0 ), threshold( threshold )
        { if( threshold && *threshold <= 0 ) { COMMA_THROW( comma::exception, "expected positive threshold, got " << *threshold ); } }
    pid( const std::string& pid_values, char delimiter = ',' )
    {
        std::vector< std::string > v = comma::split( pid_values, delimiter );
        if( v.size() != 3 && v.size() != 4 ) { COMMA_THROW( comma::exception, "expected a string with 3 or 4 elements separated by '" << delimiter << "', got " << v.size() ); }
        p = boost::lexical_cast< double >( v[0] );
        i = boost::lexical_cast< double >( v[1] );
        d = boost::lexical_cast< double >( v[2] );
        integral = 0;
        if( v.size() == 4 )
        {
            threshold = boost::lexical_cast< double >( v[3] );
            if( *threshold <= 0 ) { COMMA_THROW( comma::exception, "expected positive threshold, got " << *threshold ); }
        }
    }
    double operator()( double error, const boost::posix_time::ptime& t = boost::posix_time::not_a_date_time )
    {
        boost::optional< double > dt = get_time_increment( t );
        double derivative = previous_error && dt ? ( error - *previous_error ) / *dt : 0;
        return update( error, derivative, t, dt );
    }
    double operator()( double error, double derivative, const boost::posix_time::ptime& t = boost::posix_time::not_a_date_time )
    {
        boost::optional< double > dt = get_time_increment( t );
        return update( error, derivative, t, dt );
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
    boost::posix_time::ptime time;
    boost::optional< double > previous_error;
    boost::optional< double > get_time_increment( const boost::posix_time::ptime& t )
    {
        if( time == boost::posix_time::not_a_date_time || t == boost::posix_time::not_a_date_time ) { return boost::none; }
        if( t <= time ) { COMMA_THROW( comma::exception, "expected time greater than " << boost::posix_time::to_iso_string( time ) << ", got " << boost::posix_time::to_iso_string( t ) ); }
        return ( t - time ).total_microseconds() / 1e6;
    }
    void clip_integral()
    {
        if( !threshold ) { return; }
        if ( integral > *threshold ) { integral = *threshold; }
        else if ( integral < -*threshold ) { integral = -*threshold; }
    }
    double update( double error, double derivative, const boost::posix_time::ptime& t, boost::optional< double > dt )
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
};

} } // namespace snark { namespace control {

#endif // SNARK_CONTROL_PID_HEADER
