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


#include "../dc1394.h"
#include <boost/program_options.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include <comma/string/string.h>
#include <comma/packed/bits.h>

static const char* name() { return "point-grey"; }

// see Register Reference for Point Grey Digital Cameras Version 3.1 Revised 2/6/2013 (Section 6.3)
static const uint64_t PIO_DIRECTION_ADDRESS = 0x11F8;
struct pin_mode 
{
    pin_mode() : pin0( 0 ), pin1( 0 ), pin2( 0 ), pin3( 0 ), unused( 0 ) {}
    comma::uint32 pin0: 1, pin1: 1, pin2: 1, pin3: 1, unused:28;
};
enum { MODE_IN = 0, MODE_OUT = 1 };

static const unsigned int number_of_pins = snark::camera::dc1394::number_of_pins;

void set_pin_direction( snark::camera::dc1394& camera, unsigned int pin, bool is_out = true )
{
    uint32_t value;
    camera.get_control_register( &value, PIO_DIRECTION_ADDRESS );
    comma::packed::reversed_bits< pin_mode > packed_mode = *reinterpret_cast< comma::packed::reversed_bits< pin_mode >* >( &value );
    pin_mode mode = packed_mode();
    switch( pin )
    {
        case 0: mode.pin0 = is_out ? MODE_OUT : MODE_IN; break;
        case 1: mode.pin1 = is_out ? MODE_OUT : MODE_IN; break;
        case 2: mode.pin2 = is_out ? MODE_OUT : MODE_IN; break;
        case 3: mode.pin3 = is_out ? MODE_OUT : MODE_IN; break;
    }
    packed_mode = mode;
    value = *reinterpret_cast< uint32_t* >( packed_mode.data() );
    camera.set_control_register( value, PIO_DIRECTION_ADDRESS );
}

int main( int argc, char** argv )
{
    try
    {
        std::string config_string; 
        std::string direction;
        unsigned int pin; 
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "verbose,v", "more output; --help --verbose: more help message" )
            ( "config", boost::program_options::value< std::string >( &config_string ), "configuration file for the camera or semicolon-separated name=value string, run 'fire-cat -h -v' for more details" )
            ( "pin", boost::program_options::value< unsigned int >( &pin )->default_value( 0 ), "apply operation to the specified pin (default: 0)" )
            ( "direction", boost::program_options::value< std::string >( &direction ), "set pin direction to 'in' or 'out'" );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) || vm.count( "verbose" ) )
        {
            std::cerr << "perform operations specific to point-grey cameras" << std::endl;
            std::cerr << "Usage: " << name() << " [options] \n" << std::endl;
            std::cerr << description << std::endl;
            std::cerr << std::endl;
            std::cerr << "examples:" << std::endl;
            std::cerr << "\tset direction of pin=2 to 'out': " << name() << " --config=bumblebee.config --pin=2 --direction=out" << std::endl;
            std::cerr << std::endl;
            return 1;
        }
        if( config_string.empty() ) { std::cerr << name() << ": --config is not given" << std::endl; return 1; }
        snark::camera::dc1394::config config;
        bool config_from_command_line = config_string.find_first_of( '=' ) != std::string::npos; // quick and dirty
        if( config_from_command_line )
        {
            config = comma::name_value::parser( ';', '=' ).get< snark::camera::dc1394::config >( config_string );
        }
        else
        {
            std::vector< std::string > v = comma::split( config_string, ':' );
            if( v.size() > 2 ) { std::cerr << name() << ": expected --config=filename or --config=filename:xpath, got '" << config_string << "'" << std::endl; return 1; }
            std::string filename = v[0];
            std::string xpath = ( v.size() == 1 ) ? "" : v[1];
            config = comma::read< snark::camera::dc1394::config >( filename, xpath.c_str() );
        }
        snark::camera::dc1394 camera( config );
        if( pin >= number_of_pins ) { std::cerr <<  name() << "expected pin to be from 0 to " << number_of_pins - 1 << ", got " << pin << std::endl; return 1; }
        if( vm.count( "direction" ) )
        {
            if( direction != "in" && direction != "out" ) { std::cerr << name() << ": expected direction to be either 'in' or 'out', got " << direction << std::endl; }
            set_pin_direction( camera, pin, ( direction == "out" ) );
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << argv[0] << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << argv[0] << ": unknown exception" << std::endl;
    }
    return 1;
}
