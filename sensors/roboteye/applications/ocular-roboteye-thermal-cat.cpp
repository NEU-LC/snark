// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2018 The University of Sydney
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

/// @author Dave Jennings

#include "../ocular.h"
#include "../traits.h"
#include <Image.h>
#include <RobotEyeThermal.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>

const unsigned int default_image_mode = 0;

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --input-fields --input-format"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

void usage( bool )
{
    std::cerr << "\ncapture images from Ocular RobotEye thermal camera";
    std::cerr << "\noutput to stdout as serialized cv::Mat";
    std::cerr << "\n";
    std::cerr << "\nusage: cat <x,y> | " << comma::verbose.app_name() << " <address> [<options>]";
    std::cerr << "\n    where <address> is ip address of ocular roboteye device";
    std::cerr << "\n";
    std::cerr << "\noptions";
    std::cerr << "\n    --help,-h:    show help";
    std::cerr << "\n    --verbose,-v: show detailed messages";
    std::cerr << "\n    --input-fields: print input fields and exit";
    std::cerr << "\n    --input-format: print input format and exit";
    std::cerr << "\n    --image-mode=[<0-3>]: default=" << default_image_mode << "; set image mode";
    std::cerr << "\n";
    std::cerr << "\n    where image mode is one of: 0 for None";
    std::cerr << "\n                                1 for De-rorate";
    std::cerr << "\n                                2 for Circular Mask";
    std::cerr << "\n                                3 for both De-rorate & Circular Mask";
    std::cerr << "\n";
    std::cerr << "\nNote that you have to set the environment variable GENICAM_ROOT_V3_0 first";
    std::cerr << "\n$ export GENICAM_ROOT_V3_0=" << STRINGIZED( OCULAR_ROBOTEYE_GENICAM_DIR );
    std::cerr << "\n";
    std::cerr << "\nexample:";
    std::cerr << "\n    echo 0,0 | " << comma::verbose.app_name() << " 169.254.111.102 > frames.bin";
    std::cerr << "\n" << std::endl;
}

template< typename T > std::string field_names( bool full_xpath = false ) { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
template< typename T > std::string format( bool full_xpath = false, const std::string& fields = "" ) { return comma::csv::format::value< T >( !fields.empty() ? fields : field_names< T >( full_xpath ), full_xpath ); }

int main( int argc, char** argv )
{
    comma::verbose << "GENICAM_ROOT_V3_0 set to " << ::getenv( "GENICAM_ROOT_V3_0" ) << std::endl;
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) bash_completion( argc, argv );
        if( options.exists( "--input-fields" )) { std::cout << field_names< snark::ocular::roboteye::position_t >( true ) << std::endl; return 0; }
        if( options.exists( "--input-format" )) { std::cout << format< snark::ocular::roboteye::position_t >() << std::endl; return 0; }

        std::vector< std::string > unnamed = options.unnamed( comma::csv::options::valueless_options() + ",--verbose,-v", "-.*" );
        if( unnamed.size() != 1 ) { COMMA_THROW( comma::exception, "require ip address" ); }

        std::string ip_address = unnamed[0];
        comma::verbose << "connecting to RobotEye at " << ip_address << std::endl;

        ocular::Image_Modes_t image_mode = static_cast< ocular::Image_Modes_t >( options.value< unsigned int >( "--image-mode", default_image_mode ));

        ocular::RobotEyeThermal roboteye_thermal( ip_address );
        roboteye_thermal.SetPixelType( ocular::thermal::PIXELTYPE_MONO_8 );

        const unsigned int num_frames = 5;
        std::vector< ocular::Image > image_pool( num_frames );
        unsigned long int i = 0;
        roboteye_thermal.StartAcquisition();
        while( i < num_frames )
        {
            roboteye_thermal.GetImage( image_pool[i], image_mode );
            comma::verbose << "acquired frame " << i << std::endl;
            ++i;
        }

        roboteye_thermal.StopAcquisition();

        i = 0;
        unsigned int count = 0;
        for( auto& image : image_pool )
        {
            const std::string file_name = "./output/frame" + std::to_string(i) + ".png";
            if( image.SaveFrame( file_name ))
            {
                ++count;
            }
            ++i;
        }
        comma::verbose << count << " frames saved successfully." << std::endl;

        /*
        comma::csv::options csv( options );
        comma::csv::input_stream< snark::ocular::roboteye::position_t > istream( std::cin, csv );
        comma::signal_flag is_shutdown;

        while( !is_shutdown && std::cin.good() )
        {
            const position_t* position = istream.read();
            if( position )
            {
                comma::verbose << "position: " << position->pan << "," << position->tilt << std::endl;
            }
        }
        */
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
