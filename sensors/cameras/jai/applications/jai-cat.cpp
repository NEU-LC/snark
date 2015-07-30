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

#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include "../../../../imaging/cv_mat/pipeline.h"
#include "../camera.h"

typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
boost::scoped_ptr< snark::tbb::bursty_reader< pair_t > > reader;
static pair_t capture( snark::jai::camera& camera )
{ 
    static comma::signal_flag is_shutdown;
    if( is_shutdown ) { reader->stop(); return pair_t(); }
    return camera.read();    
}

int main( int argc, char** argv )
{
    try
    {
        std::string fields;
        //std::string address;
        std::string setattributes;
        std::string calibration_file;
        std::string directory;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "set", boost::program_options::value< std::string >( &setattributes ), "set camera attributes as semicolon-separated name-value pairs" )
            ( "set-and-exit", "set camera attributes specified in --set and exit" )
            //( "address", boost::program_options::value< std::string >( &address )->default_value( "" ), "ip address of the camera" )
            ( "discard", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            ( "list-attributes", "output current camera attributes" )
            ( "list-cameras,l", "list all cameras" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" )
            ( "calibration", boost::program_options::value< std::string >( &calibration_file ), "calibration file for thermography")
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
            else { std::cerr << "run: jai-cat --help --verbose for more..." << std::endl; }
            return 0;
        }
        if( vm.count( "header" ) && vm.count( "no-header" ) ) { COMMA_THROW( comma::exception, "--header and --no-header are mutually exclusive" ); }
        if( vm.count( "fields" ) && vm.count( "no-header" ) ) { COMMA_THROW( comma::exception, "--fields and --no-header are mutually exclusive" ); }
        if( vm.count( "buffer" ) == 0 && vm.count( "discard" ) ) { discard = 1; }
        snark::jai::factory factory;
        if( vm.count( "list-cameras" ) )
        {
            const std::vector< std::string >& ids = factory.list_devices();
            std::cerr << "jai-cat: found " << ids.size() << " device(s); output readable part only for now; todo: implement camera id parsing" << std::endl;
            for( unsigned int i = 0; i < ids.size(); ++i )
            {
                unsigned int size = 0;
                for( ; size < ids[i].size() && ids[i][size] > 31; ++size );
                std::cout << ids[i].substr( 0, size ) << std::endl;
            }
            return 0;
        }
        discard = vm.count( "discard" ) ? 1 : 0;
        snark::jai::camera::attributes_type attributes;
        if( vm.count( "set" ) )
        {
            comma::name_value::map m( setattributes, ';', '=' );
            attributes.insert( m.get().begin(), m.get().end() );
        }   
        if( verbose ) { std::cerr << "jai-cat: connecting..." << std::endl; }
        boost::scoped_ptr< snark::jai::camera > camera( factory.make_camera() );
        if( verbose ) { std::cerr << "jai-cat: connected to a camera" << std::endl; }
        if( verbose ) { std::cerr << "jai-cat: total bytes per frame: " << camera->total_bytes_per_frame() << std::endl; }
        if( vm.count( "set-and-exit" ) ) { return 0; }
        if( vm.count( "list-attributes" ) )
        {
            const snark::jai::camera::attributes_type& a = camera->attributes();
            for( snark::jai::camera::attributes_type::const_iterator it = a.begin(); it != a.end(); ++it )
            {
                std::cout << it->first;
                if( !it->second.empty() ) { std::cout << '=' << it->second; }
                std::cout << std::endl;
            }            
            return 0;
        }
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
        reader.reset( new snark::tbb::bursty_reader< pair_t >( boost::bind( &capture, boost::ref( *camera ) ), discard ) );
        snark::imaging::applications::pipeline pipeline( *serialization, filters, *reader );
        pipeline.run();
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << argv[0] << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << argv[0] << ": unknown exception" << std::endl; }
    return 1;
}
