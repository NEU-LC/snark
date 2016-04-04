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

#include <VimbaCPP/Include/VimbaSystem.h>

#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>

#include "../error.h"
#include "../feature.h"
#include "../frame_observer.h"

AVT::VmbAPI::VimbaSystem& vimba_system( AVT::VmbAPI::VimbaSystem::GetInstance() );

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
    std::cerr << "    --help,-h:         show this help, --help --verbose for more help" << std::endl;
    std::cerr << "    --verbose,-v:      more output" << std::endl;
    std::cerr << "    --version:         output the library version" << std::endl;
    std::cerr << "    --list-cameras:    list all cameras and exit" << std::endl;
    std::cerr << "    --list-attributes: list camera attributes, --verbose for more detail" << std::endl;
    std::cerr << "    --id=<camera id>:  default: first available camera" << std::endl;
    std::cerr << "    --fields=<fields>: header fields; default: " << default_fields << std::endl;
    std::cerr << "    --header:          output header only" << std::endl;
    std::cerr << "    --no-header:       output image data only" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    Possible values for <fields> are: " << possible_fields << std::endl;

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

void write_version_string()
{
    VmbVersionInfo_t version = { 0, 0, 0 };
    VmbErrorType error = vimba_system.QueryVersion( version );
    if( error == VmbErrorSuccess ) {
        std::cerr << "Vimba library version: " << version.major << "." << version.minor << "." << version.patch << std::endl;
    } else {
        snark::vimba::write_error( "QueryVersion() failed", error );
    }
}

void print_camera_info( const AVT::VmbAPI::CameraPtr &camera )
{
    std::string id;
    std::string name;
    std::string model;
    std::string serial_number;
    std::string interface_id;

    VmbErrorType error = camera->GetID( id );
    if( error != VmbErrorSuccess ) { snark::vimba::write_error( "Could not get camera ID", error ); }
                
    error = camera->GetName( name );
    if( error != VmbErrorSuccess ) { snark::vimba::write_error( "Could not get camera name", error ); }

    error = camera->GetModel( model );
    if( error != VmbErrorSuccess ) { snark::vimba::write_error( "Could not get camera mode name", error ); }

    error = camera->GetSerialNumber( serial_number );
    if( error != VmbErrorSuccess ) { snark::vimba::write_error( "Could not get camera serial number", error ); }

    error = camera->GetInterfaceID( interface_id );
    if( error != VmbErrorSuccess ) { snark::vimba::write_error( "Could not get interface ID", error ); }

    std::cout << "Camera Name  : " << name          << "\n"
              << "Model Name   : " << model         << "\n"
              << "Camera ID    : " << id            << "\n"
              << "Serial Number: " << serial_number << "\n"
              << "Interface ID : " << interface_id  << std::endl;
}

void list_cameras()
{
    AVT::VmbAPI::CameraPtrVector cameras;

    VmbErrorType error = vimba_system.GetCameras( cameras );            // Fetch all cameras known to Vimba
    if( error == VmbErrorSuccess )
    {
        std::cout << "Cameras found: " << cameras.size() << "\n\n";

        // Query all static details of all known cameras and print them out.
        // We don't have to open the cameras for that.
        std::for_each( cameras.begin(), cameras.end(), print_camera_info );
    }
    else
    {
        snark::vimba::write_error( "Could not list cameras", error );
    }
}

AVT::VmbAPI::CameraPtr open_camera( boost::optional< std::string > camera_id )
{
    AVT::VmbAPI::CameraPtr camera = AVT::VmbAPI::CameraPtr();
    VmbErrorType error = VmbErrorSuccess;

    if( camera_id )
    {
        // Get and open the camera
        error = vimba_system.OpenCameraByID( camera_id->c_str(), VmbAccessModeFull, camera );
    }
    else                                // Use the first camera
    {
        AVT::VmbAPI::CameraPtrVector cameras;
        error = vimba_system.GetCameras( cameras );
        if( error == VmbErrorSuccess )
        {
            if( !cameras.empty() )
            {
                camera = cameras[0];
                error = camera->Open( VmbAccessModeFull );
            }
            else
            {
                std::cerr << "No cameras found" << std::endl;
            }
        }
    }

    if( error != VmbErrorSuccess )
    {
        snark::vimba::write_error( "Could not open camera", error );
        camera = AVT::VmbAPI::CameraPtr(); // Reset camera pointer
    }
    return camera;
}

// Prints all features and their values of a camera
void list_attributes( AVT::VmbAPI::CameraPtr camera, bool verbose )
{
    AVT::VmbAPI::FeaturePtrVector features;
    VmbErrorType error = VmbErrorSuccess;

    error = camera->GetFeatures( features );
    if( error == VmbErrorSuccess )
    {
        snark::vimba::print_features( features, verbose );
    }
    else
    {
        snark::vimba::write_error( "Could not get features", error );
    }
}

#define NUM_FRAMES 3

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

VmbErrorType start_continuous_image_acquisition( AVT::VmbAPI::CameraPtr camera
                                               , std::unique_ptr< snark::cv_mat::serialization > serialization )
{
    // Set the GeV packet size to the highest possible value
    snark::vimba::set_feature( camera, "GVSPAdjustPacketSize" );

    // Create a frame observer for this camera
    // (This will be wrapped in a shared_ptr so we don't delete it)
    snark::vimba::frame_observer* fo = new snark::vimba::frame_observer( camera, std::move( serialization ));
    // Start streaming
    VmbErrorType status = camera->StartContinuousImageAcquisition( NUM_FRAMES, AVT::VmbAPI::IFrameObserverPtr( fo ));
    return status;
}

void stop_continuous_image_acquisition( AVT::VmbAPI::CameraPtr camera )
{
    camera->StopContinuousImageAcquisition();
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

// "command": "gige-cat  --id 180711 --set \\\"SyncOut3Invert=Off;SyncOut3Mode=Exposing;ExposureMode=Manual;ExposureValue=60;GainValue=12;PixelFormat=Bayer12Packed;StreamBytesPerSecond=120000000;PacketSize=9216;FrameRate=5\\\""

/*
Vimba:
ExposureAuto=Off
ExposureAutoAdjustTol=5
ExposureAutoAlg=Mean
ExposureAutoMax=500000
ExposureAutoMin=10
ExposureAutoOutliers=0
ExposureAutoRate=100
ExposureAutoTarget=50
ExposureMode=Timed
ExposureTimeAbs=60
*/

void capture_images( AVT::VmbAPI::CameraPtr camera, const comma::command_line_options& options )
{
    std::cerr << "capture_images" << std::endl;
    VmbErrorType error;

    error = start_continuous_image_acquisition( camera, create_serializer( options ));
    if ( error == VmbErrorSuccess )
    {
        std::cerr << "Press <enter> to stop acquisition..." << std::endl;
        getchar();

        stop_continuous_image_acquisition( camera );
    }
}

int run_cmd( const comma::command_line_options& options )
{
    if( options.exists( "--list-cameras" ))
    {
        list_cameras();
        return 0;
    }

    // These options require a camera
    // TODO: Wrap in RAII
    AVT::VmbAPI::CameraPtr camera = open_camera( options.optional< std::string >( "--id" ));
    if( camera )
    {
        if( options.exists( "--list-attributes" ))
        {
            list_attributes( camera, comma::verbose );
            camera->Close();
            return 0;
        }

        if( options.exists( "--set" ))
        {
            comma::name_value::map m( options.value<std::string>( "--set" ));
            for( auto it = m.get().cbegin(); it != m.get().cend(); ++it )
            {
                snark::vimba::set_feature( camera, it->first, it->second );
                // if( comma::verbose )
                //     std::cerr << "Setting " << it->first << "=" << it->second << std::endl;
                // VmbErrorType status = set_feature( camera, it->first, it->second );
                // if( status != VmbErrorSuccess )
                // {
                //     std::ostringstream msg;
                //     msg << "Warning: couldn't set " << it->first << "=" << it->second;
                //     snark::vimba::write_error( msg.str(), status );
                // }
            }
        }

        capture_images( camera, options );
        camera->Close();
    }
    return 0;
}

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

int main( int argc, char** argv )
{
    ::setenv( "GENICAM_GENTL64_PATH", STRINGIZED( VIMBA_GENICAM_GENTL64_PATH ), 0 );

    int ret_code = 0;

    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );

        if( options.exists( "--version" ))
        {
            write_version_string();
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

        VmbErrorType error = vimba_system.Startup();               // Initialize the Vimba API
        if( error != VmbErrorSuccess )
        {
            COMMA_THROW( comma::exception, snark::vimba::error_msg( "", error ));
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
    vimba_system.Shutdown();                             // Close Vimba
    return ret_code;
}
