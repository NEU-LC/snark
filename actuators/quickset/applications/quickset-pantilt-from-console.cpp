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
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#include <cmath>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/last_error.h>
#include <comma/io/select.h>

static const long double pi_ = 3.14159265358979323846l;

static void usage()
{
    std::cerr << "take arrow key codes on stdin, output pan/tilt positions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: io-console | quickset-pantilt-from-console [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --step,-s <step>: step in radians, both for pan and tilt" << std::endl;
    std::cerr << "    --pan-step,-p <step>: pan step in radians; default 5 degrees" << std::endl;
    std::cerr << "    --tilt-step,-t <step>: tilt step in radians; default 5 degrees" << std::endl;
    std::cerr << "    --no-inertial: if present, output multiple steps, if key held and decrease, once released" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

class Inertial
{
    public:
        Inertial( boost::posix_time::time_duration decay, unsigned int maximum );
        void poke();
        unsigned int pick();
        unsigned int value() const;
        void reset();
        
    private:
        unsigned int decay_;
        unsigned int maximum_;
        boost::posix_time::ptime last_;
        unsigned int value_;
};

class InertialPair
{
    public:
        InertialPair( boost::posix_time::time_duration decay, unsigned int maximum );
        void operator++();
        void operator--();
        int pick();

    private:
        Inertial up_;
        Inertial down_;
};

int main( int ac, char** av )
{
    bool verbose = false;
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        float panStep = options.value( "--pan-step,-p", options.value( "--step", 5.0 * M_PI / 180 ) );
        float tiltStep = options.value( "--tilt-step,-t", options.value( "--step", 5.0 * M_PI / 180 ) );
        verbose = options.exists( "--verbose,-v" );
        bool inertial = !options.exists( "--no-inertial" );
        if( verbose ) { std::cerr << "quickset-pantilt-from-console: pan,tilt step: " << panStep << "," << tiltStep << std::endl; }
        comma::io::select select;
        select.read().add( 0 );
        InertialPair horizontal( boost::posix_time::millisec( inertial ? 500 : 0 ), inertial ? 5 : 1 );
        InertialPair vertical( boost::posix_time::millisec( inertial ? 500 : 0 ), inertial ? 5 : 1 );
        comma::signal_flag isShutdown;
        char b = 0;
        boost::posix_time::time_duration threshold = boost::posix_time::millisec( 200 );
        boost::posix_time::ptime last = boost::posix_time::microsec_clock::universal_time();
        while( !isShutdown && std::cout.good() && !std::cout.eof() && std::cin.good() && !std::cin.eof() )
        {
            // todo: it does not exit, if std::cout closed
            //       catch sigpipe?
            
            if( select.wait( boost::posix_time::seconds( 1 ) ) == 0 ) { continue; }
            char c;
            if( ::read( 0, &c, 1 ) != 1 ) { break; }
            switch( c )
            {
                case 0x41: ++vertical; break;
                case 0x42: --vertical; break;
                case 0x43: ++horizontal; break;
                case 0x44: --horizontal; break;
                default: continue;
            }
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            if( b != c || ( now - last ) > threshold )
            {
                std::cout << ( panStep * horizontal.pick() ) << "," << ( tiltStep * vertical.pick() ) << std::endl;
                std::cout.flush();
                last = now;
            }
            b = c;
        }
        if( verbose ) { std::cerr << "quickset-pantilt-from-console: done" << std::endl; }
        return 0;
    }
    catch( comma::last_error::interrupted_system_call_exception& )
    {
        if( verbose ) { std::cerr << "quickset-pantilt-from-console: done" << std::endl; }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "quickset-pantilt-from-console: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "quickset-pantilt-from-console: unknown exception" << std::endl;
    }
    usage();
}

Inertial::Inertial( boost::posix_time::time_duration decay, unsigned int maximum )
    : decay_( decay.total_microseconds() )
    , maximum_( maximum )
    , value_( 0 )
{
}

void Inertial::reset()
{
    last_ = boost::posix_time::not_a_date_time;
    value_ = 0;
}

void Inertial::poke()
{
    pick();
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    if(    value_ < maximum_
        && (    last_ == boost::posix_time::not_a_date_time
             || decay_ == 0
             || ( last_ - now ).total_microseconds() < decay_ ) )
    {
        ++value_;
    }
    last_ = now;
}

unsigned int Inertial::pick()
{
    if( last_ == boost::posix_time::not_a_date_time ) { return value_; }
    if( decay_ == 0 ) { unsigned int v = value_; value_ = 0; return v; }
    unsigned int steps = ( boost::posix_time::microsec_clock::universal_time() - last_ ).total_microseconds() / decay_;
    value_ = value_ > steps ? value_ - steps : 0;
    return value_;
}

unsigned int Inertial::value() const { return value_; }

InertialPair::InertialPair( boost::posix_time::time_duration decay, unsigned int maximum )
    : up_( decay, maximum )
    , down_( decay, maximum )
{
}

void InertialPair::operator++()
{
    //std::cerr << "---> ++: down_.value()=" << down_.value() << " up_.value()=" << up_.value() << std::endl;
    down_.pick();
    if( down_.value() == 0 ) { up_.poke(); } else { up_.reset(); }
    down_.reset();
}

void InertialPair::operator--()
{
    //std::cerr << "---> --: down_.value()=" << down_.value() << " up_.value()=" << up_.value() << std::endl;
    up_.pick();
    if( up_.value() == 0 ) { down_.poke(); } else { down_.reset(); }
    up_.reset();
}

int InertialPair::pick() { return int( up_.pick() ) - int( down_.pick() ); }
