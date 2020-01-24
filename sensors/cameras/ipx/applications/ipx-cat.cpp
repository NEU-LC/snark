// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

#include <boost/program_options.hpp>
#include <memory>
//#include <boost/regex.hpp>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include "../../../../imaging/cv_mat/pipeline.h"
#include "../camera.h"

typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
std::unique_ptr< snark::tbb::bursty_reader< pair_t > > reader;

static pair_t capture( snark::ipx::stream& stream )
{ 
    static comma::signal_flag is_shutdown;
    if( is_shutdown ) { reader->stop(); return pair_t(); }
    try { return stream.read(); }
    catch( std::exception& ex ) { std::cerr << "ipx-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "ipx-cat: unknown exception" << std::endl; }
    return pair_t();
}

int main( int argc, char** argv )
{
    try
    {
        std::string fields;
        std::string id;
        std::string settings_to_set;
        std::string settings_file;
        std::string directory;
        std::string output_options_string;
        unsigned int discard = 0;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            //( "set", boost::program_options::value< std::string >( &settings_to_set ), "set camera settings as semicolon-separated name-value pairs: todo" )
            //( "save-settings", boost::program_options::value< std::string >( &settings_file ), "save camera settings to file specified in <arg> in camera" )
            //( "load-settings", boost::program_options::value< std::string >( &settings_file ), "load camera settings from file specified in <arg> in camera" )
            //( "validate-settings", boost::program_options::value< std::string >( &settings_file ), "validate camera settings saved in file specified in <arg> in camera" )
            //( "do-and-exit", "perform --set, or --load-settings, or --save-settings and exit" )
            ( "id", boost::program_options::value< std::string >( &id )->default_value( "" ), "camera id; run --list-devices to get device id; if not present, will connect to the first available camera" )
            ( "discard", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "list-devices", "print device list as <interface>,<id>,<description> on stdout and exit" )
            ( "list-interfaces", "print interface list on stdout and exit" )
            ( "list-parameters", "print parameter list of a given device on stdout and exit (likely to be replaced by --save-parameters later)" )
            //( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            //( "list-cameras,l", "list all cameras" )
            //( "list-settings", "list relevant implemented settings for given camera; todo: output settings tree as json" )
            //( "all", "if --list-settings, list all settings, including write-only, unimplemented, and unavailable" )
            //( "unimplemented", "if --list-settings, list only read-write settings still not implemented in ipx-cat" )
            //( "camera-info", "camera info for given camera --id" )
            //( "list-cameras-human-readable", "list all camera ids, output only human-readable part" )
            //( "header", "output header only" )
            //( "no-header", "output image data only" )
            //( "raw", "output raw image data" )
            //( "output-conversion", boost::program_options::value< std::string >( &directory ), "output conversion table to a timestamped csv file in the specified directory")
            ( "output", boost::program_options::value< std::string >( &output_options_string ), "output options; same as cv-cat --output <options>")
            ( "verbose,v", "be more verbose" );

        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        bool verbose = vm.count( "verbose" );
        if ( vm.count( "help" ) )
        {
            std::cerr << "acquire images from a ipx camera" << std::endl;
            std::cerr << "output to stdout as serialized cv::Mat" << std::endl;
            std::cerr << std::endl;
            std::cerr << "usage: ipx-cat [<options>] [<filters>]" << std::endl;
            std::cerr << std::endl;
            std::cerr << "output header format: fields: t,cols,rows,type; binary: t,3ui" << std::endl;
            std::cerr << std::endl;
            std::cerr << description << std::endl;
            //if( verbose )
            //{
            //    std::cerr << "available options for PixelFormat ( raw image data output only ):" << std::endl;
            //    std::cerr << "    visible camera: BayerRG8, BayerRG10, BayerRG12" << std::endl;
            //    std::cerr << "    NIR camera: Mono8, Mono10, Mono12" << std::endl;
            //}
            if( verbose ) { std::cerr << snark::cv_mat::impl::filters<>::usage() << std::endl; }
            else { std::cerr << "for more info run: ipx-cat --help --verbose..." << std::endl; }
            std::cerr << std::endl;
            return 0;
        }
        //if( vm.count( "save-settings" ) + vm.count( "load-settings" ) + vm.count( "validate-settings" ) > 1 ) { std::cerr << "ipx-cat: --save-settings, --load-settings, and --validate-settings mutually exclusive" << std::endl; return 1; }
        //if( vm.count( "header" ) && vm.count( "no-header" ) ) { std::cerr << "ipx-cat: --header and --no-header are mutually exclusive" << std::endl; return 1; }
        //if( vm.count( "fields" ) && vm.count( "no-header" ) ) { std::cerr << "ipx-cat: --fields and --no-header are mutually exclusive" << std::endl; return 1; }
        if( vm.count( "buffer" ) == 0 && vm.count( "discard" ) ) { discard = 1; }
        //const std::vector< std::string >& v = comma::split( fields, "," );
        //comma::csv::format format;
        //for( unsigned int i = 0; i < v.size(); ++i )
        //{
        //    if( v[i] == "t" ) { format += "t"; }
        //    else { format += "ui"; }
        //}
        std::vector< std::string > filter_strings = boost::program_options::collect_unrecognized( parsed.options, boost::program_options::include_positional );
        std::string filters;
        if( filter_strings.size() == 1 ) { filters = filter_strings[0]; }
        if( filter_strings.size() > 1 ) { COMMA_THROW( comma::exception, "expected filters as a single ';'-separated name-value string; got: " << comma::join( filter_strings, ' ' ) ); }
        if( verbose ) { std::cerr << "ipx-cat: creating system..." << std::endl; }
        snark::ipx::system system;
        if( verbose ) { std::cerr << "ipx-cat: created system" << std::endl; }
        if( vm.count( "list-interfaces" ) ) { std::cout << system.interfaces_description(); return 0; }
        if( vm.count( "list-devices" ) ) { std::cout << system.devices_description(); return 0; }
        if( verbose ) { std::cerr << "ipx-cat: obtaining " << ( id.empty() ? "first available device" : "device with id \"" + id + "\"" ) << "..." << std::endl; }
        snark::ipx::camera camera( system.device_info( id ) );
        if( vm.count( "list-parameters" ) ) { std::cout << camera.list_parameters(); return 0; }
        snark::cv_mat::serialization::options output_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( output_options_string );
        snark::cv_mat::serialization serialization( output_options );
        // todo: pass --output to serialization
        //snark::ipx::camera camera;
        if( verbose ) { std::cerr << "ipx-cat: data acquisition: starting..." << std::endl; }
        //camera.connect();
        //snark::ipx::stream stream( camera );
        //camera.start_acquisition();
        if( verbose ) { std::cerr << "ipx-cat: data acquisition: started" << std::endl; }
        //reader.reset( new snark::tbb::bursty_reader< pair_t >( boost::bind( &capture, boost::ref( stream ) ), discard ) );
        //snark::imaging::applications::pipeline pipeline( serialization, filters, *reader ); // todo?! link only with selected filters?
        //pipeline.run();
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << argv[0] << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << argv[0] << ": unknown exception" << std::endl; }
    return 1;
}
