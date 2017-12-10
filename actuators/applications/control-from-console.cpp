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

/// @author vsevolod vlaskine

#include <cmath>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/last_error.h>
#include <comma/base/exception.h>
#include <comma/io/select.h>

static void usage()
{
    std::cerr << "take arrow key codes on stdin, output control values as ascii csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: io-console | control-from-console <operation> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --accumulate,-a: if present, output not velocity or turn rate, but absolute accumulated value" << std::endl;
    std::cerr << "    --no-inertial: if present, output multiple steps, if key held; decrease, once released" << std::endl;
    std::cerr << "    --output-fields: output fields for a given operation" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    cartesian: output velocity along x and y axis" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --max=<velocity-x>,<velocity-y>: max value for velocity and turn rate; default: ... for both: todo" << std::endl;
    std::cerr << "            --step,-s <velocity-step-x>,<velocity-step-y>: step for velocity in meters and turn rate in radians; default: 0.1 m/s" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    drive: output velocity/turn_rate values" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --max=<velocity>,<turn-rate>: max value for velocity and turn rate; default: ... for both: todo" << std::endl;
    std::cerr << "            --step,-s <velocity-step>,<turn-rate-step>: step for velocity in meters and turn rate in radians; default: velocity: 0.1 m/s, turn rate: 5 degrees/s" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    pantilt: pan/tilt values" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --max=<pan>,<tilt>: max value for pan/tilt; default: ... for both: todo" << std::endl;
    std::cerr << "            --step,-s=<step>|<pan-step>,<tilt-step>: step in radians, both for pan and tilt; default: 5 degrees/s for both" << std::endl;
    std::cerr << std::endl;
    std::cerr << "example" << std::endl;
    std::cerr << "    io-console | control-from-console drive | my-device-control ..." << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct inertial
{
    class value
    {
        public:
            value( boost::posix_time::time_duration decay, unsigned int maximum );
            void poke();
            unsigned int pick();
            unsigned int operator()() const;
            void reset();
            
        private:
            unsigned int decay_;
            unsigned int maximum_;
            boost::posix_time::ptime last_;
            unsigned int value_;
    };

    class pair
    {
        public:
            pair( boost::posix_time::time_duration decay, unsigned int maximum );
            void operator++();
            void operator--();
            int pick();

        private:
            inertial::value up_;
            inertial::value down_;
    };
};

inertial::value::value( boost::posix_time::time_duration decay, unsigned int maximum ) : decay_( decay.total_microseconds() ), maximum_( maximum ), value_( 0 ) {}

void inertial::value::reset() { last_ = boost::posix_time::not_a_date_time; value_ = 0; }

void inertial::value::poke()
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

unsigned int inertial::value::pick()
{
    if( last_ == boost::posix_time::not_a_date_time ) { return value_; }
    if( decay_ == 0 ) { unsigned int v = value_; value_ = 0; return v; }
    unsigned int steps = ( boost::posix_time::microsec_clock::universal_time() - last_ ).total_microseconds() / decay_;
    value_ = value_ > steps ? value_ - steps : 0;
    return value_;
}

unsigned int inertial::value::operator()() const { return value_; }

inertial::pair::pair( boost::posix_time::time_duration decay, unsigned int maximum ) : up_( decay, maximum ), down_( decay, maximum ) {}

void inertial::pair::operator++()
{
    //std::cerr << "---> ++: down_.operator()()=" << down_.operator()() << " up_.operator()()=" << up_.operator()() << std::endl;
    down_.pick();
    if( down_() == 0 ) { up_.poke(); } else { up_.reset(); }
    down_.reset();
}

void inertial::pair::operator--()
{
    //std::cerr << "---> --: down_.operator()()=" << down_.operator()() << " up_.operator()()=" << up_.operator()() << std::endl;
    up_.pick();
    if( up_() == 0 ) { down_.poke(); } else { down_.reset(); }
    up_.reset();
}

int inertial::pair::pick() { return int( up_.pick() ) - int( down_.pick() ); }

static std::pair< double, double > default_steps( const std::string& operation )
{
    if( operation == "cartesian" ) { return std::make_pair( 0.1, 0.1 ); }
    if( operation == "drive" ) { return std::make_pair( 0.1, 5.0 * M_PI / 180 ); }
    if( operation == "pantilt" ) { return std::make_pair( 5.0 * M_PI / 180, 5.0 * M_PI / 180 ); }
    std::cerr << "control-from-console: expected operation, got: \"" << operation << "\"" << std::endl;
    exit( 1 );
}

int main( int ac, char** av )
{
    bool verbose = false;
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        const auto& unnamed = options.unnamed( "--accumulate,-a,--no-inertial,--output-fields,--verbose,-v" );
        if( unnamed.empty() ) { std::cerr << "control-from-console: please specify operation" << std::endl; return 1; }
        if( unnamed.size() > 1 ) { std::cerr << "control-from-console: expected one operation, got: " << comma::join( unnamed, ',' ) << std::endl; return 1; }
        std::string operation = unnamed[0];
        if( options.exists( "--output-fields" ) )
        {
            if( operation == "cartesian" ) { std::cout << "velocity/x,velocity/y" << std::endl; return 0; }
            if( operation == "drive" ) { std::cout << "velocity,turn_rate" << std::endl; return 0; }
            if( operation == "pantilt" ) { std::cout << "pan,tilt" << std::endl; return 0; }
            std::cerr << "control-from-console: expected operation, got: \"" << operation << "\"" << std::endl;
            return 1;
        }
        std::pair< double, double > steps = default_steps( operation );
        const auto& s = comma::split( options.value< std::string >( "--step,-s", "" ), ',' );
        switch( s.size() )
        {
            case 1:
                if( !s[0].empty() ) { steps.first = boost::lexical_cast< double >( s[0] ); }
                break;
            case 2:
                if( !s[0].empty() ) { steps.first = boost::lexical_cast< double >( s[0] ); }
                if( !s[1].empty() ) { steps.second = boost::lexical_cast< double >( s[1] ); }
                break;
            default:
                std::cerr << "control-from-console: expected --step <first>[,<second>]; got: \"" << options.value< std::string >( "--step,-s", "" ) << "\"" << std::endl;
                return 1;
        }
        bool inertial = !options.exists( "--no-inertial" );
        verbose = options.exists( "--verbose,-v" );
        bool accumulate = options.exists( "--accumulate,-a" );
        if( verbose ) { std::cerr << "control-from-console: step: " << steps.first << "," << steps.second << std::endl; }
        comma::io::select select;
        select.read().add( 0 );
        inertial::pair horizontal( boost::posix_time::millisec( inertial ? 500 : 0 ), inertial ? 5 : 1 );
        inertial::pair vertical( boost::posix_time::millisec( inertial ? 500 : 0 ), inertial ? 5 : 1 );
        comma::signal_flag is_shutdown;
        char b = 0;
        boost::posix_time::time_duration threshold = boost::posix_time::millisec( 200 );
        boost::posix_time::ptime last = boost::posix_time::microsec_clock::universal_time();
        std::pair< double, double > accumulated( 0, 0 );
        while( !is_shutdown && std::cout.good() && !std::cout.eof() && std::cin.good() && !std::cin.eof() )
        {
            // todo: it does not exit, if std::cout closed; catch sigpipe?
            if( select.wait( boost::posix_time::seconds( 1 ) ) == 0 ) { continue; }
            char c;
            if( ::read( 0, &c, 1 ) != 1 ) { break; }
            if( verbose ) { std::cerr << "control-from-console: got " << c << std::endl; }
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
                if( accumulate )
                {
                    accumulated.first += steps.first * horizontal.pick();
                    accumulated.second += steps.second * vertical.pick();
                    std::cout << accumulated.first << "," << accumulated.second << std::endl;
                }
                else
                {
                    std::cout << ( steps.first * horizontal.pick() ) << "," << ( steps.second * vertical.pick() ) << std::endl;
                }
                std::cout.flush();
                last = now;
            }
            b = c;
        }
        if( verbose ) { std::cerr << "control-from-console: done" << std::endl; }
        return 0;
    }
    catch( comma::last_error::interrupted_system_call_exception& ) { if( verbose ) { std::cerr << "control-from-console: done" << std::endl; } return 0; }
    catch( std::exception& ex ) { std::cerr << "control-from-console: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "control-from-console: unknown exception" << std::endl; }
    return 1;
}
