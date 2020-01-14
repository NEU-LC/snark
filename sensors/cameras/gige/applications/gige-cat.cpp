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
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include "../../../../imaging/cv_mat/pipeline.h"
#include "../gige.h"

typedef std::pair< boost::posix_time::ptime, cv::Mat > Pair;

boost::scoped_ptr< snark::tbb::bursty_reader< Pair > > reader;
static Pair capture( snark::camera::gige& camera ) 
{ 
    static comma::signal_flag is_shutdown;
    if( is_shutdown ) { reader->stop(); return Pair(); }
    return camera.read(); 
}
 
int main( int argc, char** argv )
{
    try
    {
        std::string fields;
        unsigned int id;
        unsigned int timeout;
        std::string setattributes;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "camera-clock", "use camera clock; todo: debug, does it even make sense without ptp" )
            ( "set", boost::program_options::value< std::string >( &setattributes ), "set camera attributes as semicolon-separated name-value pairs" )
            ( "set-and-exit", "set camera attributes specified in --set and exit" )
            ( "id", boost::program_options::value< unsigned int >( &id )->default_value( 0 ), "camera id; default: first available camera" )
            ( "frame-timeout,t", boost::program_options::value< unsigned int >( &timeout )->default_value( 0 ), "timeout requesting frames from camera; default: 3x max frame interval up to interval + 500ms" )
            ( "discard", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            ( "list-attributes", "output current camera attributes" )
            ( "list-cameras", "list all cameras and exit" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" )
            ( "verbose,v", "be more verbose" );

        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) )
        {
            std::cerr << "acquire images from a prosilica gige camera" << std::endl;
            std::cerr << "output to stdout as serialized cv::Mat" << std::endl;
            std::cerr << "usage: gige-cat [<options>] [<filters>]\n" << std::endl;
            std::cerr << "output header format: fields: t,cols,rows,type; binary: t,3ui\n" << std::endl;
            std::cerr << description << std::endl;
            std::cerr << snark::cv_mat::impl::filters<>::usage() << std::endl;
            return 1;
        }
        if( vm.count( "header" ) && vm.count( "no-header" ) ) { COMMA_THROW( comma::exception, "--header and --no-header are mutually exclusive" ); }
        if( vm.count( "fields" ) && vm.count( "no-header" ) ) { COMMA_THROW( comma::exception, "--fields and --no-header are mutually exclusive" ); }
        if( vm.count( "buffer" ) == 0 && vm.count( "discard" ) ) { discard = 1; }
        bool verbose = vm.count( "verbose" );
        if( vm.count( "list-cameras" ) )
        {
            const std::vector< tPvCameraInfo >& list = snark::camera::gige::list_cameras();
            for( std::size_t i = 0; i < list.size(); ++i ) // todo: serialize properly with name-value
            {
                std::cout << "id=" << list[i].UniqueId << "," << "name=\"" << list[i].DisplayName << "\"" << "," << "serial=\"" << list[i].SerialString << "\"" << std::endl;
            }
            return 0;
        }
        if ( vm.count( "discard" ) )
        {
            discard = 1;
        }
        
        snark::camera::gige::attributes_type attributes;
        if( vm.count( "set" ) )
        {
            comma::name_value::map m( setattributes, ';', '=' );
            attributes.insert( m.get().begin(), m.get().end() );
        }
        if( verbose ) { std::cerr << "gige-cat: connecting..." << std::endl; }
        snark::camera::gige camera( id, vm.count( "camera-clock" ), attributes );
        if( verbose ) { std::cerr << "gige-cat: connected to camera " << camera.id() << std::endl; }
        if( verbose ) { std::cerr << "gige-cat: total bytes per frame: " << camera.total_bytes_per_frame() << std::endl; }
        if( timeout > 0 ) { camera.set_frame_timeout( timeout ); }
        if( verbose ) { std::cerr << "gige-cat: frame timeout: " << camera.frame_timeout() << " ms" << std::endl; }
        if( vm.count( "set-and-exit" ) ) { return 0; }
        if( vm.count( "list-attributes" ) )
        {
            attributes = camera.attributes(); // quick and dirty
            for( snark::camera::gige::attributes_type::const_iterator it = attributes.begin(); it != attributes.end(); ++it )
            {
                if( it != attributes.begin() ) { std::cout << std::endl; }
                std::cout << it->first;
                if( it->second != "" ) { std::cout << '=' << it->second; }
            }
            return 0;
        }

        std::vector< std::string > v = comma::split( fields, "," );
        comma::csv::format format;
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            if( v[i] == "t" ) { format += "t"; }
            else { format += "ui"; }
        }
        std::vector< std::string > filterStrings = boost::program_options::collect_unrecognized( parsed.options, boost::program_options::include_positional );
        std::string filters;
        if( filterStrings.size() == 1 ) { filters = filterStrings[0]; }
        if( filterStrings.size() > 1 ) { COMMA_THROW( comma::exception, "please provide filters as name-value string" ); }
        boost::scoped_ptr< snark::cv_mat::serialization > serialization;
        if( vm.count( "no-header" ) )
        {
            serialization.reset( new snark::cv_mat::serialization( "", format ) );
        }
        else
        {
            serialization.reset( new snark::cv_mat::serialization( fields, format, vm.count( "header" ) ) );
        }
        reader.reset( new snark::tbb::bursty_reader< Pair >( boost::bind( &capture, boost::ref( camera ) ), discard ) );
        snark::imaging::applications::pipeline pipeline( *serialization, filters, *reader );
        pipeline.run();
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
