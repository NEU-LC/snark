// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#include <comma/application/verbose.h>
#include <comma/base/exception.h>

#include "../camera.h"
#include "../error.h"
#include "../system.h"

static const std::string possible_fields = "t,rows,cols,type,size";
static const std::string default_fields = "t,rows,cols,type";

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char * completion_options =
        " --help -h"
        " --verbose -v"
        " --version"
        " --list-cameras"
        " --list-attributes"
        " --id"
        " --set --set-and-exit"
        " --header --no-header"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "Capture data from an Allied Vision GigE camera" << std::endl;
    std::cerr << "Output to stdout as serialized cv::Mat" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: " << comma::verbose.app_name() << " [<options>] [<filters>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options: " << std::endl;
    std::cerr << "    --help,-h:          show this help, --help --verbose for more help" << std::endl;
    std::cerr << "    --verbose,-v:       more output" << std::endl;
    std::cerr << "    --version:          output the library version" << std::endl;
    std::cerr << "    --list-cameras:     list all cameras and exit, --verbose for more detail" << std::endl;
    std::cerr << "    --list-attributes:  list camera attributes, --verbose for more detail" << std::endl;
    std::cerr << "    --set <attributes>: set camera attributes" << std::endl;
    std::cerr << "    --set-and-exit <attributes>: set attributes and exit" << std::endl;
    std::cerr << "    --id=<camera id>:   default: first available camera" << std::endl;
    std::cerr << "    --fields=<fields>:  header fields; default: " << default_fields << std::endl;
    std::cerr << "    --header:           output header only" << std::endl;
    std::cerr << "    --no-header:        output image data only" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    Possible values for <fields> are: " << possible_fields << "." << std::endl;
    std::cerr << "    <attributes> are semicolon-separated name-value pairs." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Examples:" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " --set \"ExposureAuto=Off;ExposureTimeAbs=60\"" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "Camera id:" << std::endl;
        std::cerr << "    --list-cameras will provide two id's for each found camera: the camera id" << std::endl;
        std::cerr << "    and the serial number. Either can be used as the value for the --id option." << std::endl;
        std::cerr << std::endl;
        std::cerr << "Trouble-shooting:" << std::endl;
        std::cerr << "    Before running any vimba application you need to set GENICAM_GENTL64_PATH" << std::endl;
        std::cerr << "    to the path to the transport layer cti file. See the Vimba Manual for" << std::endl;
        std::cerr << "    more detail." << std::endl;
        std::cerr << std::endl;
        // std::cerr << "Image Filters:" << std::endl;
        // std::cerr << snark::cv_mat::filters::usage() << std::endl;
    }
    exit( 0 );
}

comma::csv::format format_from_fields( const std::string& fields )
{
    std::vector< std::string > v = comma::split( fields, "," );
    comma::csv::format format;
    for( unsigned int i = 0; i < v.size(); ++i )
    {
        if( v[i] == "t" ) { format += "t"; }
        else { format += "ui"; }
    }
    return format;
}

std::unique_ptr< snark::cv_mat::serialization > create_serializer( const comma::command_line_options& options )
{
    std::string        fields = options.value< std::string >( "--fields", default_fields );
    comma::csv::format format = format_from_fields( fields );
    bool               header_only = false;

    if( options.exists( "--no-header" ))
    {
        fields = "";
    }
    else
    {
        header_only = ( options.exists( "--header" ));
    }
    std::unique_ptr< snark::cv_mat::serialization > serialization
        ( new snark::cv_mat::serialization( fields, format, header_only ));
    return serialization;
}

int run_cmd( const comma::command_line_options& options )
{
    snark::vimba::system system;                                   // Initialize the Vimba API

    if( options.exists( "--list-cameras" ))
    {
        AVT::VmbAPI::CameraPtrVector cameras = snark::vimba::system::get_cameras();
        if( comma::verbose ) { std::cout << "Cameras found: " << cameras.size() << std::endl; }
        for( auto iter = cameras.cbegin(); iter != cameras.cend(); ++iter )
        {
            snark::vimba::camera camera( *iter );
            camera.print_info( comma::verbose );
        }
        return 0;
    }

    snark::vimba::camera camera( options.exists( "--id" )
                               ? snark::vimba::camera( options.value<std::string>( "--id" ))
                               : snark::vimba::camera( snark::vimba::system::open_first_camera()));

    if( options.exists( "--list-attributes" ))
    {
        camera.list_attributes( comma::verbose );
        return 0;
    }

    if( options.exists( "--set-and-exit" ))
    {
        camera.set_features( options.value<std::string>( "--set-and-exit" ));
        return 0;
    }

    if( options.exists( "--set" ))
    {
        camera.set_features( options.value<std::string>( "--set" ));
    }

    camera.capture_images( create_serializer( options ));

    return 0;
}

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

int main( int argc, char** argv )
{
    // TODO: allow this to be overridden if the environment variable is already set
    ::setenv( "GENICAM_GENTL64_PATH", STRINGIZED( VIMBA_GENICAM_GENTL64_PATH ), 0 );

    int ret_code = 0;

    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );

        if( options.exists( "--version" ))
        {
            VmbVersionInfo_t version = snark::vimba::system::version();
            std::cerr << "Vimba library version: " << version.major << "." << version.minor << "." << version.patch << std::endl;
            return 0;
        }

        if( getenv( "GENICAM_GENTL64_PATH" ))
        {
            if( comma::verbose )
                std::cerr << "GENICAM_GENTL64_PATH=" << getenv( "GENICAM_GENTL64_PATH" ) << std::endl;
        }
        else
        {
            COMMA_THROW( comma::exception, "GENICAM_GENTL64_PATH is not set" );
        }

        ret_code = run_cmd( options );
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
        ret_code = 1;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl;
        ret_code = 1;
    }
    return ret_code;
}
