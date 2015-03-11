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


#include <snark/sensors/dc1394/dc1394.h>
#include <boost/program_options.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include <comma/packed/bits.h>

static const char* name() { return "point-grey"; }

// quick and dirty for now, just to tear down application/ptree.h
template < typename C >
C config_from_ini( const std::string& filename, const std::string& name = "", const C& defaultconfig = C() )
{
    boost::property_tree::ptree tree;
    C config = defaultconfig;
    std::ifstream file;
    file.open( filename.c_str() );
    if ( !file.is_open() )
    {
        comma::to_ptree v( tree, name );
        comma::visiting::apply( v, config );
        boost::property_tree::ini_parser::write_ini( filename, tree );
    }
    else
    {
        boost::property_tree::ini_parser::read_ini( filename, tree );
        boost::property_tree::ptree::assoc_iterator it = tree.find( name );
        if( it == tree.not_found() && !name.empty() )
        {
            // section not found, put default
            comma::to_ptree v( tree, name );
            comma::visiting::apply( v, config );
            boost::property_tree::ini_parser::write_ini(filename, tree);
        }
        else
        {
            comma::from_ptree v( tree, name, true );
            comma::visiting::apply( v, config );
        }
    }
    return config;
}

static const uint64_t PIO_DIRECTION_ADDRESS = 0x11F8;
static const unsigned int number_of_pins = snark::camera::dc1394::number_of_pins;

void set_pin_direction( snark::camera::dc1394& camera, unsigned int pin, bool is_out = true )
{    
    static const uint32_t enable_out[number_of_pins] = { 0x0001, 0x0002, 0x0004, 0x0008 };
    uint32_t value = 0;
    camera.get_control_register( value, PIO_DIRECTION_ADDRESS );
    comma::packed::reverse_bits< uint32_t >( value );
    if( is_out ) { value |= enable_out[pin]; } else { value &= ~enable_out[pin]; }
    comma::packed::reverse_bits< uint32_t >( value );
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
        
        snark::camera::dc1394::config config;
        if( config_string.find_first_of( '=' ) == std::string::npos ) // quick and dirty
        {
            config = config_from_ini< snark::camera::dc1394::config >( config_string );
        }
        else
        {
            comma::name_value::parser parser( ';', '=' );
            config = parser.get< snark::camera::dc1394::config >( config_string );
        }
        
        snark::camera::dc1394 camera( config );
        
        if( pin >= number_of_pins ) { COMMA_THROW( comma::exception, "expected pin to be from 0 to " << number_of_pins - 1 << ", got " << pin ); }
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
