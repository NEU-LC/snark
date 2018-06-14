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

#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include <comma/name_value/serialize.h>
#include "../../../../imaging/cv_mat/pipeline.h"
#include "../camera.h"
#include "../node.h"
#include "../stream.h"
#include "../traits.h"

typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
boost::scoped_ptr< snark::tbb::bursty_reader< pair_t > > reader;

static pair_t capture( snark::jai::stream& stream )
{ 
    static comma::signal_flag is_shutdown;
    if( is_shutdown ) { reader->stop(); return pair_t(); }
    try { return stream.read(); }
    catch( std::exception& ex ) { std::cerr << "jai-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "jai-cat: unknown exception" << std::endl; }
    return pair_t();
}

// todo: quick and dirty, move to camera.cpp?

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

int main( int argc, char** argv )
{
    // todo: quick and dirty, move to camera.cpp?
    ::setenv( "GENICAM_ROOT", STRINGIZED( JAI_GENICAM_ROOT ), 0 );
    ::setenv( STRINGIZED( JAI_GENICAM_ROOT_NAME ), STRINGIZED( JAI_GENICAM_ROOT ), 0 );
    ::setenv( "GENICAM_CONFIG", STRINGIZED( JAI_GENICAM_CONFIG ), 0 );
    ::setenv( "GENICAM_CACHE", STRINGIZED( JAI_GENICAM_CACHE ), 0 );
    
    try
    {
        std::string fields;
        std::string id;
        std::string settings_to_set;
        std::string settings_file;
        std::string directory;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "set", boost::program_options::value< std::string >( &settings_to_set ), "set camera settings as semicolon-separated name-value pairs: todo" )
            ( "save-settings", boost::program_options::value< std::string >( &settings_file ), "save camera settings to file specified in <arg> in camera" )
            ( "load-settings", boost::program_options::value< std::string >( &settings_file ), "load camera settings from file specified in <arg> in camera" )
            ( "validate-settings", boost::program_options::value< std::string >( &settings_file ), "validate camera settings saved in file specified in <arg> in camera" )
            ( "do-and-exit", "perform --set, or --load-settings, or --save-settings and exit" )
            ( "id", boost::program_options::value< std::string >( &id )->default_value( "" ), "any fragment of user-readable part of camera id; connect to the first device with matching id" )
            ( "discard", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            ( "list-cameras,l", "list all cameras" )
            ( "list-settings", "list relevant implemented settings for given camera; todo: output settings tree as json" )
            ( "all", "if --list-settings, list all settings, including write-only, unimplemented, and unavailable" )
            ( "unimplemented", "if --list-settings, list only read-write settings still not implemented in jai-cat" )
            ( "camera-info", "camera info for given camera --id" )
            ( "list-cameras-human-readable,L", "list all camera ids, output only human-readable part" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" )
            ( "raw", "output raw image data" )
            ( "output-conversion", boost::program_options::value< std::string >( &directory ), "output conversion table to a timestamped csv file in the specified directory")
            ( "verbose,v", "be more verbose" );

        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        bool verbose = vm.count( "verbose" );
        if ( vm.count( "help" ) )
        {
            std::cerr << "acquire images from a jai camera" << std::endl;
            std::cerr << "output to stdout as serialized cv::Mat" << std::endl;
            std::cerr << "usage: jai-cat [<options>] [<filters>]\n" << std::endl;
            std::cerr << "output header format: fields: t,cols,rows,type; binary: t,3ui\n" << std::endl;
            std::cerr << description << std::endl;
            if( verbose ) { std::cerr << snark::cv_mat::filters::usage() << std::endl; }
            if( verbose )
            {
                std::cerr << "Available options for PixelFormat ( raw image data output only ):" << std::endl;
                std::cerr << "    visible camera: BayerRG8, BayerRG10, BayerRG12" << std::endl;
                std::cerr << "    NIR camera: Mono8, Mono10, Mono12" << std::endl;
            }
            else { std::cerr << "run: jai-cat --help --verbose for more..." << std::endl; }
            std::cerr << std::endl;
            return 0;
        }
        if( vm.count( "save-settings" ) + vm.count( "load-settings" ) + vm.count( "validate-settings" ) > 1 ) { std::cerr << "jai-cat: --save-settings, --load-settings, and --validate-settings mutually exclusive" << std::endl; return 1; }
        if( vm.count( "header" ) && vm.count( "no-header" ) ) { std::cerr << "jai-cat: --header and --no-header are mutually exclusive" << std::endl; return 1; }
        if( vm.count( "fields" ) && vm.count( "no-header" ) ) { std::cerr << "jai-cat: --fields and --no-header are mutually exclusive" << std::endl; return 1; }
        if( vm.count( "buffer" ) == 0 && vm.count( "discard" ) ) { discard = 1; }
        snark::jai::factory factory;
        if( vm.count( "list-cameras" ) || vm.count( "list-cameras-human-readable" ) )
        {
            bool human_readable = vm.count( "list-cameras-human-readable" );
            const std::vector< std::string >& ids = factory.list_devices();
            std::cerr << "jai-cat: found " << ids.size() << " device(s)" << std::endl;
            for( unsigned int i = 0; i < ids.size(); ++i )
            {
                if( !human_readable ) { std::cout.write( &ids[i][0], ids[i].size() ); continue; }  
                unsigned int size = 0;
                for( ; size < ids[i].size() && ids[i][size] > 31; ++size );
                std::cout << ids[i].substr( 0, size ) << std::endl;
            }
            return 0;
        }
        if( vm.count( "camera-info" ) )
        {
            boost::regex regex( id );
            const std::vector< std::string >& ids = factory.list_devices();
            for( unsigned int i = 0; i < ids.size(); ++i )
            {
                if( !boost::regex_search( ids[i], regex ) ) { continue; }
                comma::write_path_value( factory.camera_info( ids[i] ), std::cout );
                std::cout << std::endl;
            }
            return 0;
        }
        discard = vm.count( "discard" ) ? 1 : 0;
        boost::scoped_ptr< snark::jai::camera > camera;
        {
            comma::signal_flag is_shutdown;
            if( verbose ) { std::cerr << "jai-cat: connecting..." << std::endl; }
            camera.reset( factory.make_camera( id ) );
            if( verbose ) { std::cerr << "jai-cat: connected to a camera" << std::endl; }
            if (is_shutdown) { std::cerr << "jai-cat: received signal while connecting, shutting down..." << std::endl; return 0; }
        }
        if( vm.count( "list-settings" ) )
        {
            bool all = vm.count( "all" );
            bool unimplemented_only = vm.count( "unimplemented" );
            comma::csv::options csv;
            csv.quote.reset();
            comma::csv::output_stream< snark::jai::node > os( std::cout, csv );
            const std::vector< snark::jai::node >& nodes = snark::jai::nodes( *camera );
            for( std::size_t i = 0; i < nodes.size(); ++i ) { if( all || ( nodes[i].readable() && ( unimplemented_only == !nodes[i].implemented() ) ) ) { os.write( nodes[i] ); } }
            return 0;
        }
        if( !settings_to_set.empty() )
        {
            comma::name_value::map m( settings_to_set );
            typedef comma::name_value::map::map_type map_t;
            for( map_t::const_iterator it = m.get().begin(); it != m.get().end(); ++it )
            {
                snark::jai::node( it->first, it->second ).send_to( *camera );
            }
        }
        if( vm.count( "--save-settings" ) )
        {
            snark::jai::camera::settings settings( *camera );
            settings.save( settings_file ); // todo: implement flags
        }
        else if( vm.count( "--load-settings" ) )
        {
            snark::jai::camera::settings settings( *camera );
            settings.load( settings_file ); // todo: implement flags
        }
        else if( vm.count( "--validate-settings" ) )
        {
            snark::jai::camera::settings settings( *camera );
            const std::string& status = settings.validate( settings_file );
            if( !status.empty() ) { std::cerr << "jai-cat: settings in " << settings_file << " are invalid: " << status << std::endl; return 1; }
            std::cerr << "jai-cat: settings in " << settings_file << " are valid" << std::endl;
        }
        if( vm.count( "do-and-exit" ) ) { return 0; }
        const std::vector< std::string >& v = comma::split( fields, "," );
        comma::csv::format format;
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            if( v[i] == "t" ) { format += "t"; }
            else { format += "ui"; }
        }
        std::vector< std::string > filter_strings = boost::program_options::collect_unrecognized( parsed.options, boost::program_options::include_positional );
        std::string filters;
        if( filter_strings.size() == 1 ) { filters = filter_strings[0]; }
        if( filter_strings.size() > 1 ) { COMMA_THROW( comma::exception, "please provide filters as name-value string" ); }
        boost::scoped_ptr< snark::cv_mat::serialization > serialization;
        if( vm.count( "no-header" ) ) { serialization.reset( new snark::cv_mat::serialization( "", format ) ); }
        else { serialization.reset( new snark::cv_mat::serialization( fields, format, vm.count( "header" ) ) ); }
        if( verbose ) { std::cerr << "jai-cat: data acquisition: starting..." << std::endl; }
        snark::jai::stream stream( *camera, 10, vm.count( "raw" ) ); // snark::jai::stream stream( *camera );
        camera->start_acquisition();
        if( verbose ) { std::cerr << "jai-cat: data acquisition: started" << std::endl; }
        reader.reset( new snark::tbb::bursty_reader< pair_t >( boost::bind( &capture, boost::ref( stream ) ), discard ) );
        snark::imaging::applications::pipeline pipeline( *serialization, filters, *reader );
        pipeline.run();
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << argv[0] << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << argv[0] << ": unknown exception" << std::endl; }
    return 1;
}
