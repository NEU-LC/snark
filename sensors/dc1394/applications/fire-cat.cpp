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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#include <snark/imaging/cv_mat/pipeline.h>
#include <snark/sensors/dc1394/dc1394.h>
#include <boost/program_options.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>

typedef std::pair< boost::posix_time::ptime, cv::Mat > Pair;

/// camera capture
static Pair capture( snark::camera::dc1394& camera )
{
    cv::Mat image = camera.read();
    return std::make_pair( camera.time(), image.clone() );
}

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

int main( int argc, char** argv )
{
    try
    {
        std::string config_string; 
        std::string fields;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "long-help", "display long help message" )
            ( "list", "list cameras on the bus with guids" )
            ( "list-attributes", "output current camera attributes" )
            ( "discard,d", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "config,c", boost::program_options::value< std::string >( &config_string )->default_value( "fire-cat.ini" ), "configuration file for the camera or semicolon-separated name=value string, see long help for details" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" );

        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        
        if( vm.count( "header" ) + vm.count( "no-header" ) > 1 )
        {
            COMMA_THROW( comma::exception, "--header and --no-header are mutually exclusive" );
        }

        if( vm.count( "fields" ) && vm.count( "no-header" ) > 1 )
        {
            COMMA_THROW( comma::exception, "--fields and --no-header are mutually exclusive" );
        }

        if ( vm.count( "help" ) || vm.count( "long-help" ) )
        {
            std::cerr << "acquire images from a firewire camera using libdc1394 and output them to std::out in OpenCV format" << std::endl;
            std::cerr << "Usage: fire-cat [options] [<filters>]\n" << std::endl;
            std::cerr << "output header format: fields: t,cols,rows,type; binary: t,3ui\n" << std::endl;
            std::cerr << description << std::endl;
            std::cerr << snark::cv_mat::filters::usage() << std::endl;

            if ( vm.count( "long-help" ) )
            {
                std::cerr << std::endl << "config file options:" << std::endl;
                std::cerr << "\toutput-type: output image type" << std::endl;
                std::cerr << "\tvideo-mode: dc1394 video mode" << std::endl;
                std::cerr << "\toperation-mode: dc1394 operation mode" << std::endl;
                std::cerr << "\tiso-speed: dc1394 iso speed" << std::endl;
                std::cerr << "\tframe-rate: dc1394 frame rate" << std::endl;
                std::cerr << "\tshutter: camera shutter speed (absolute)" << std::endl;
                std::cerr << "\tgain: camera gain (absolute)" << std::endl;
                std::cerr << "\trelative-shutter: camera shutter speed (relative)" << std::endl;
                std::cerr << "\trelative-gain: camera gain (relative)" << std::endl;
                std::cerr << "\texposure: camera exposure" << std::endl;
                std::cerr << std::endl << "allowed output types: " << std::endl;
                std::cerr << "\tRGB: convert the camera output to RGB8 using dc1394_convert_frames" << std::endl;
                std::cerr << "\tBGR: convert the camera output to BGR8 using dc1394_convert_frames" << std::endl;
                std::cerr << "\tRaw: no conversion, memcpy the camera output to cv::Mat, only 8 or 24 bits per pixel supported" << std::endl;
                std::cerr << std::endl << "allowed video modes, use coriander to see what your camera supports: " << std::endl;
                snark::camera::print_video_modes();
                std::cerr << std::endl << "allowed operation modes: " << std::endl;
                snark::camera::print_operation_modes();
                std::cerr << std::endl << "allowed iso speeds: " << std::endl;
                snark::camera::print_iso_speeds();
                std::cerr << std::endl << "allowed frame rates: " << std::endl;
                snark::camera::print_frame_rates();
                std::cerr << std::endl << "ini file example for bumblebee on shrimp:" << std::endl;
                std::cerr << "--------------------------------------------" << std::endl;
                std::cerr << "output-type=RGB\nvideo-mode=DC1394_VIDEO_MODE_FORMAT7_3\noperation-mode=DC1394_OPERATION_MODE_1394B\n";
                std::cerr << "iso-speed=DC1394_ISO_SPEED_800\nframe-rate=DC1394_FRAMERATE_240\nguid=49712223529993963" << std::endl;
                std::cerr << "--------------------------------------------" << std::endl;
                std::cerr << std::endl << "ini file example for ladybug on shrimp:" << std::endl;
                std::cerr << "--------------------------------------------" << std::endl;
                std::cerr << "output-type=Raw\nvideo-mode=DC1394_VIDEO_MODE_FORMAT7_0\noperation-mode=DC1394_OPERATION_MODE_1394B\n";
                std::cerr << "iso-speed=DC1394_ISO_SPEED_800\nframe-rate=DC1394_FRAMERATE_240\nguid=49712223530115149" << std::endl;
                std::cerr << "--------------------------------------------" << std::endl;
                std::cerr << std::endl << "ini file example for pika2 on shrimp:" << std::endl;
                std::cerr << "--------------------------------------------" << std::endl;
                std::cerr << "output-type=Raw\nvideo-mode=DC1394_VIDEO_MODE_FORMAT7_2\noperation-mode=DC1394_OPERATION_MODE_1394B\n";
                std::cerr << "iso-speed=DC1394_ISO_SPEED_800\nframe-rate=DC1394_FRAMERATE_240\nguid=49712223534632451\n";
                std::cerr << "left=0\ntop=0\nwidth=640\nheight=240\n" << std::endl;
                std::cerr << "--------------------------------------------" << std::endl;
            }

            std::cerr << "examples:" << std::endl;
            std::cerr << "\tview all 3 bumblebee cameras: fire-cat \"split;bayer=4;resize=640,1440;view\" > /dev/null" << std::endl;
            std::cerr << "\tview all 6 ladybug cameras: fire-cat \"bayer=1;resize=808,3696;transpose;view\" > /dev/null" << std::endl;
            std::cerr << std::endl;
            return 1;
        }


        if ( vm.count( "list" ) )
        {
            snark::camera::dc1394::list_cameras();
            return 1;
        }

        if ( vm.count( "discard" ) )
        {
            discard = 1;
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
        if( filterStrings.size() == 1 )
        {
            filters = filterStrings[0];
        }
        if( filterStrings.size() > 1 )
        {
            COMMA_THROW( comma::exception, "please provide filters as name-value string" );
        }

        boost::scoped_ptr< snark::cv_mat::serialization > serialization;
        if( vm.count( "no-header" ) )
        {
            serialization.reset( new snark::cv_mat::serialization( "", format ) );
        }
        else
        {
            serialization.reset( new snark::cv_mat::serialization( fields, format, vm.count( "header" ) ) );
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
        
        if( vm.count( "list-attributes" ) )
        {
            camera.list_attributes();    
            return 0;
        }

        snark::tbb::bursty_reader< Pair > reader( boost::bind( &capture, boost::ref( camera ) ), discard );
        snark::imaging::applications::pipeline pipeline( *serialization, filters, reader );
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
