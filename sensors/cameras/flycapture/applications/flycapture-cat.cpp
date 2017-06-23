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
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include "../../../../imaging/cv_mat/pipeline.h"
#include "../flycapture.h"
#include "../helpers.h"

typedef std::pair< boost::posix_time::ptime, cv::Mat > Pair;

boost::scoped_ptr< snark::tbb::bursty_reader< Pair > > reader;
static Pair capture( snark::cameras::flycapture::camera& camera )
{
    return camera.read( );
}

int main( int argc, char** argv )
{
    try
    {
        std::string fields;
        unsigned int id;
        std::string setattributes;
        std::string timestamp_time_option;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "set", boost::program_options::value< std::string >( &setattributes ), "set camera attributes as semicolon-separated name-value pairs" )
            ( "set-and-exit", "set camera attributes specified in --set and exit" )
            ( "serial", boost::program_options::value< unsigned int >( &id )->default_value( 0 ), "camera serial number; default: first available camera" )
            ( "discard", boost::program_options::value< unsigned int >( &discard ), "buffer this many frames, discard after" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            ( "timestamp", boost::program_options::value< std::string >( &timestamp_time_option )->default_value( "after" ), "when to take the timestamp of the frame (before or after reading data from the camera, or the average of the two); possible values before, after, average" )
            ( "list-cameras", "list all cameras and exit" )
            ( "list-attributes", "output current camera attributes" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" )
            ( "verbose,v", "be more verbose" );

        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        bool verbose = vm.count( "verbose" );
        if ( vm.count( "help" ) )
        {
            std::cerr << "acquire images from a pointgrey flycapture camera" << std::endl
                << "output to stdout as serialized cv::Mat" << std::endl
                << "usage: flycapture-cat [<options>] [<filters>]" << std::endl
                << "output header format: fields: t,cols,rows,type binary: t,3ui" << std::endl
                << description << std::endl
                << "Known issues:" << std::endl
                << "  * GigE discovery may fail if there are devices on a different subnets." << std::endl
                << "  * Using flycapture with 16.04 may segfault when application closes (driver issue)." << std::endl
                << std::endl;
            return 1;
        }
        if( vm.count( "header" ) && vm.count( "no-header" ) ) { COMMA_THROW( comma::exception, "--header and --no-header are mutually exclusive" ); }
        if( vm.count( "fields" ) && vm.count( "no-header" ) ) { COMMA_THROW( comma::exception, "--fields and --no-header are mutually exclusive" ); }
        if( vm.count( "list-cameras" ) )
        {
            const std::vector< unsigned int >& list = snark::cameras::flycapture::camera::list_camera_serials();
            for( std::size_t i = 0; i < list.size(); ++i )
            {
                std::cout << snark::cameras::flycapture::camera::describe_camera(list[i]) << std::endl;
            }
            return 0;
        }
        snark::cameras::flycapture::camera::attributes_type attributes;
        if( vm.count( "set" ) )
        {
            // comma::name_value::map attribute_map( setattributes, ';', '=' );

            for (auto attr : comma::split( setattributes, ";" ))
            {
                auto name_value_pair = comma::split( attr, "=" );
                if (name_value_pair.size() != 2)
                { COMMA_THROW(comma::exception, "name-value pairs for camera settings must be in format attribute=value"); }
                attributes.push_back( std::make_pair(name_value_pair[0], name_value_pair[1]) );
            }
            // attributes.insert( attribute_map.get().begin(), attribute_map.get().end() );
        }
        snark::cameras::flycapture::camera::timestamp_policy when( timestamp_time_option );
        if( verbose ) { std::cerr << "flycapture-cat: connecting..." << std::endl; }
        snark::cameras::flycapture::camera camera( id, attributes, when );
        if( verbose )
        {
            std::cerr << "flycapture-cat: connected to " << snark::cameras::flycapture::get_interface_string(camera.get_interface())
                << " camera " << camera.id() << std::endl; }
        if( verbose ) { std::cerr << "flycapture-cat: total bytes per frame: " << camera.total_bytes_per_frame() << std::endl; }
        if( vm.count( "set-and-exit" ) ) { return 0; }
        if( vm.count( "list-attributes" ) )
        {
            attributes = camera.attributes(); // quick and dirty
            for( snark::cameras::flycapture::camera::attributes_type::const_iterator it = attributes.begin(); it != attributes.end(); ++it )
            {
                if( it != attributes.begin() ) { std::cout << std::endl; }
                std::cout << it->first;
                if( it->second != "" ) { std::cout << '=' << it->second; }
            }
            std::cout << std::endl;
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
