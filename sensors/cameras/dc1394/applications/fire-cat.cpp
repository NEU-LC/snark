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


#include "../../../../imaging/cv_mat/pipeline.h"
#include "../dc1394.h"
#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include <comma/application/signal_flag.h>
#include <comma/string/string.h>

typedef std::pair< boost::posix_time::ptime, cv::Mat > Pair;
static const char* name() { return "fire-cat"; }

/// camera capture
boost::scoped_ptr< snark::tbb::bursty_reader< Pair > > reader;
static Pair capture( snark::camera::dc1394& camera )
{
    static comma::signal_flag is_shutdown;
    if( is_shutdown ) { reader->stop(); return Pair(); }
    cv::Mat image = camera.read();
    return std::make_pair( camera.time(), image.clone() );
}

int main( int argc, char** argv )
{
    try
    {
        std::string config_string;
        std::string fields;
        std::string strobe_string;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "verbose,v", "more output; --help --verbose: more help message" )
            ( "list", "list cameras on the bus with guids" )
            ( "list-attributes", "output current camera attributes" )
            ( "discard,d", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "config,c", boost::program_options::value< std::string >( &config_string ), "configuration file for the camera or semicolon-separated name=value string, see long help for details" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" )
            ( "strobe", boost::program_options::value< std::string >( &strobe_string ), "strobe control" );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        if( vm.count( "header" ) + vm.count( "no-header" ) > 1 ) { COMMA_THROW( comma::exception, "--header and --no-header are mutually exclusive" ); }
        if( vm.count( "fields" ) && vm.count( "no-header" ) > 1 ) { COMMA_THROW( comma::exception, "--fields and --no-header are mutually exclusive" ); }
        if ( vm.count( "help" ) || vm.count( "verbose" ) )
        {
            std::cerr << "acquire images from a firewire camera using libdc1394 and output them to std::out in OpenCV format" << std::endl;
            std::cerr << "Usage: fire-cat [options] [<filters>]\n" << std::endl;
            std::cerr << "output header format: fields: t,cols,rows,type; binary: t,3ui\n" << std::endl;
            std::cerr << description << std::endl;
            std::cerr << "strobe: " << std::endl;
            std::cerr << "    parameters: " << std::endl;
            std::cerr << "        pin: GPIO pin for strobe (its direction should be set to 'Out', use point-grey utility if necessary)" << std::endl;
            std::cerr << "        polarity: low/high" << std::endl;
            std::cerr << "        delay: delay after start of exposure until the strobe signal asserts" << std::endl;
            std::cerr << "        duration: duration of the strobe signal" << std::endl;
            std::cerr << "    note: delay and duration are given in ticks of the 1.024MHz clock (needs to be confirmed)" << std::endl;
            std::cerr << "    optional commands (by default strobe will be on while the camera is in use): " << std::endl;
            std::cerr << "        on: turn strobe on and exit" << std::endl;
            std::cerr << "        off: turn strobe off and exit" << std::endl;
            std::cerr << "    examples: " << std::endl;
            std::cerr << "        --strobe=\"pin=2;polarity=high;delay=4095;duration=4095\"" << std::endl;
            std::cerr << "        --strobe=\"on;pin=2\"" << std::endl;
            std::cerr << "        --strobe=\"off;pin=2\"" << std::endl;
            std::cerr << "    default parameters: \"pin=0;polarity=high;delay=0;duration=0\"" << std::endl;
            std::cerr << std::endl;
            std::cerr << "examples:" << std::endl;
            std::cerr << "\tview all 3 bumblebee cameras: fire-cat --config=bumblebee.config \"split;bayer=4;resize=640,1440;view\" > /dev/null" << std::endl;
            std::cerr << "\tview all 6 ladybug cameras: fire-cat --config=ladybug.config \"bayer=1;resize=808,3696;transpose;view\" > /dev/null" << std::endl;
            std::cerr << std::endl;
            if ( vm.count( "verbose" ) )
            {
                std::cerr << snark::cv_mat::filters::usage() << std::endl;
                std::cerr << std::endl << "config file options:" << std::endl;
                std::cerr << "\tvideo-mode: dc1394 video mode" << std::endl;
                std::cerr << "\toperation-mode: dc1394 operation mode" << std::endl;
                std::cerr << "\tiso-speed: dc1394 iso speed" << std::endl;
                std::cerr << "\tframe-rate: dc1394 frame rate" << std::endl;
                std::cerr << "\tshutter: camera shutter speed (absolute)" << std::endl;
                std::cerr << "\tgain: camera gain (absolute)" << std::endl;
                std::cerr << "\trelative-shutter: camera shutter speed (relative)" << std::endl;
                std::cerr << "\trelative-gain: camera gain (relative)" << std::endl;
                std::cerr << "\t(shutter and gain work as a pair, a non-zero shutter activates its corresponding gain)" << std::endl;
                std::cerr << "\texposure: camera exposure" << std::endl;
                std::cerr << "\twidth and height default to 0, meaning no ROI is used" << std::endl;
                std::cerr << "\twidth: format7 image width (default 0 : maximum width in given video-mode)" << std::endl;
                std::cerr << "\theight: format7 image height (default 0 : maximum height in given video-mode)" << std::endl;
                std::cerr << "\tleft: format7 horizontal offset from left, must have non-zero width and height (default 0)" << std::endl;
                std::cerr << "\ttop: format7 vertical offset from top, must have non-zero width height (default 0)" << std::endl;
                std::cerr << "\tpacket-size: format7 data packet size (default 0 : maximum available)" << std::endl;
                std::cerr << "\tdeinterlace: splits one RAW/MONO16 image into 2 8bit mono images (default false)" << std::endl;
                std::cerr << std::endl << "allowed video modes, use coriander to see what your camera supports: " << std::endl;
                snark::camera::print_video_modes();
                std::cerr << std::endl << "allowed operation modes: " << std::endl;
                snark::camera::print_operation_modes();
                std::cerr << std::endl << "allowed iso speeds: " << std::endl;
                snark::camera::print_iso_speeds();
                std::cerr << std::endl << "allowed frame rates (note: format7 ignores this, use packet-size to control format7 framerate): " << std::endl;
                snark::camera::print_frame_rates();
                std::cerr << std::endl << "allowed color codings for format7 op modes: " << std::endl;
                snark::camera::print_color_coding();
                std::cerr << std::endl << "config file example( --config=\"filename:bumblebee2\" ):" << std::endl;
                std::cerr << "{                                                              " << std::endl;
                std::cerr << "    \"bumblebee2\":                                            " << std::endl;
                std::cerr << "    {                                                          " << std::endl;
                std::cerr << "        \"video-mode\": DC1394_VIDEO_MODE_FORMAT7_3,           " << std::endl;
                std::cerr << "        \"operation-mode\": DC1394_OPERATION_MODE_1394B,       " << std::endl;
                std::cerr << "        \"iso-speed\": DC1394_ISO_SPEED_400,                   " << std::endl;
                std::cerr << "        \"frame-rate\": DC1394_FRAMERATE_240,                  " << std::endl;
                std::cerr << "        \"color-coding\": DC1394_COLOR_CODING_RAW16,           " << std::endl;
                std::cerr << "        \"shutter\": 0.000075,                                 " << std::endl;
                std::cerr << "        \"gain\": 5,                                           " << std::endl;
                std::cerr << "        \"packet-size\": 1536,                                 " << std::endl;
                std::cerr << "        \"deinterlace\": 1,                                    " << std::endl;
                std::cerr << "        \"guid\": 49712223535733607                            " << std::endl;
                std::cerr << "    }                                                          " << std::endl;
                std::cerr << "}                                                              " << std::endl;
                std::cerr << std::endl;
                std::cerr << std::endl << "default values:" << std::endl; // todo 
                comma::write_json< snark::camera::dc1394::config >( snark::camera::dc1394::config(), std::cerr );
                std::cerr << std::endl;
            }
            return 1;
        }
        if ( vm.count( "list" ) ) { snark::camera::dc1394::list_cameras(); return 1; }
        if ( vm.count( "discard" ) ) { discard = 1; }
        std::vector< std::string > v = comma::split( fields, "," );
        comma::csv::format format;
        for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "t" ) { format += "t"; } else { format += "ui"; } }
        std::vector< std::string > filterStrings = boost::program_options::collect_unrecognized( parsed.options, boost::program_options::include_positional );
        std::string filters;
        if( filterStrings.size() == 1 ) { filters = filterStrings[0]; }
        if( filterStrings.size() > 1 ) { COMMA_THROW( comma::exception, "please provide filters as name-value string" ); }
        boost::scoped_ptr< snark::cv_mat::serialization > serialization;
        if( vm.count( "no-header" ) ) { serialization.reset( new snark::cv_mat::serialization( "", format ) ); }
        else { serialization.reset( new snark::cv_mat::serialization( fields, format, vm.count( "header" ) ) ); }
        if( config_string.empty() ) { std::cerr << name() << ": --config is not given" << std::endl; return 1; }
        snark::camera::dc1394::config config;
        bool config_from_command_line = config_string.find_first_of( '=' ) != std::string::npos; // quick and dirty
        if( config_from_command_line )
        {
            config = comma::name_value::parser( ';', '=' ).get< snark::camera::dc1394::config >( config_string );
        }
        else
        {
            std::vector< std::string > v = comma::split( config_string, ':' );
            if( v.size() > 2 ) { std::cerr << name() << ": expected --config=filename or --config=filename:xpath, got '" << config_string << "'" << std::endl; return 1; }
            std::string filename = v[0];
            std::string xpath = ( v.size() == 1 ) ? "" : v[1];
            try { config = comma::read< snark::camera::dc1394::config >( filename, xpath.c_str() ); }
            catch(...) { config = comma::read_ini< snark::camera::dc1394::config >( filename, xpath.c_str() ); }
        }
        snark::camera::dc1394::strobe strobe;
        bool trigger_strobe_and_exit = false;
        if( vm.count( "strobe" ) )
        {
            std::vector< std::string > v = comma::split( strobe_string, ';' );
            if( v.empty() ) { std::cerr << name() << ": strobe parameters are not given (e.g. --strobe=\"pin=2\")" << std::endl; return 1; }
            if( v[0] == "on" || v[0] == "off" ) { trigger_strobe_and_exit = true; v[0] = "command=" + v[0]; } else { v.push_back( "command=auto" ); }
            strobe = comma::name_value::parser( ';', '=' ).get< snark::camera::dc1394::strobe >( comma::join< std::vector< std::string > >( v, ';' ) );
        }
        snark::camera::dc1394 camera( config, strobe );
        if( trigger_strobe_and_exit ) { return 0; }
        if( vm.count( "list-attributes" ) ) { camera.list_attributes(); return 0; }
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
