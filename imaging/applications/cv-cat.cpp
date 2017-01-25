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


#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <iostream>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <comma/application/signal_flag.h>
#include <comma/name_value/parser.h>
#include <comma/application/verbose.h>
#include "../cv_mat/pipeline.h"
#include "../cv_mat/detail/help.h"

typedef std::pair< boost::posix_time::ptime, cv::Mat > pair;
using snark::tbb::bursty_reader;

class rate_limit /// timer class, sleeping if faster than the specified fps
{
    public:
        rate_limit( double fps ) { if( fps > 1e-5 ) { m_period = boost::posix_time::microseconds( 1e6 / fps ); } }

        void wait()
        {
            if( m_period.is_not_a_date_time() ) { return; }
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            if( !m_lastOutput.is_not_a_date_time() && ( now < m_lastOutput + m_period ) )
            {
                boost::this_thread::sleep( m_lastOutput + m_period );
            }
            m_lastOutput = now;
        }

    private:
        boost::posix_time::time_duration m_period;
        boost::posix_time::ptime m_lastOutput;
};

static comma::signal_flag is_shutdown( comma::signal_flag::hard );

static pair capture( cv::VideoCapture& capture, rate_limit& rate )
{
    cv::Mat image;
    capture >> image;
    rate.wait();
    return std::make_pair( boost::posix_time::microsec_clock::universal_time(), image );
}

static pair output_single_image( const cv::Mat& image )
{
    static bool done = false;
    if( done ) { return pair(); }
    done = true;
    return std::make_pair( boost::posix_time::microsec_clock::universal_time(), image );
}

static pair read( snark::cv_mat::serialization& input, rate_limit& rate )
{
    if( is_shutdown || std::cin.eof() || std::cin.bad() || !std::cin.good() ) { return pair(); }
    rate.wait();
    return input.read( std::cin );
}

void skip( unsigned int number_of_frames_to_skip, cv::VideoCapture& video_capture, rate_limit& rate ) { for( unsigned int i=0; i<number_of_frames_to_skip; i++ ) { capture( video_capture, rate ); } }
void skip( unsigned int number_of_frames_to_skip, snark::cv_mat::serialization& input, rate_limit& rate ) { for( unsigned int i=0; i<number_of_frames_to_skip; i++ ) { read( input, rate ); } }

int main( int argc, char** argv )
{
    try
    {
        #ifdef WIN32
        _setmode( _fileno( stdin ), _O_BINARY );
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif

        std::string name;
        int device;
        unsigned int discard;
        double fps;
        std::string input_options_string;
        std::string output_options_string;
        unsigned int capacity = 16;
        unsigned int number_of_threads = 0;
        unsigned int number_of_frames_to_skip = 0;
        boost::program_options::options_description description( "options" );
        std::string help_command;
        description.add_options()
            ( "help,h", boost::program_options::value< std::string >( &help_command )->default_value( "" )->implicit_value( "" ), "display help message; if '--help command' is specified, focus on the 'command'-specific help" )
            ( "verbose,v", "more output; --help --verbose: more help" )
            ( "discard,d", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "camera", "use first available opencv-supported camera" )
            ( "file", boost::program_options::value< std::string >( &name ), "video file name" )
            ( "id", boost::program_options::value< int >( &device ), "specify specific device by id ( OpenCV-supported camera )" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "fps", boost::program_options::value< double >( &fps )->default_value( 0 ), "specify max fps ( useful for files, may block if used with cameras ) " )
            ( "input", boost::program_options::value< std::string >( &input_options_string ), "input options, when reading from stdin (see --help --verbose)" )
            ( "output", boost::program_options::value< std::string >( &output_options_string ), "output options (see --help --verbose); default: same as --input" )
            ( "capacity", boost::program_options::value< unsigned int >( &capacity )->default_value( 16 ), "maximum input queue size before the reader thread blocks" )
            ( "threads", boost::program_options::value< unsigned int >( &number_of_threads )->default_value( 0 ), "number of threads; default: 0 (auto)" )
            ( "skip", boost::program_options::value< unsigned int >( &number_of_frames_to_skip )->default_value( 0 ), "number of initial frames to skip; default: 0" )
            ( "stay", "do not close at end of stream" )
            ( "video", "has effect in opencv versions 2.12(?) and above; explicitly specify that filename given by --file refers to a video; e.g. --file ABC_0001.jpg will read a single image, --file ABC_0001.jpg will read images ABC_0001.jpg, ABC_0002.jpg, etc, if present" );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        comma::verbose.init(vm.count( "verbose" ), argv[0]);
        if ( vm.count( "help" ) )
        {
            std::string command = vm[ "help" ].as< std::string >();
            if ( ! command.empty() ) { std::cerr << snark::cv_mat::command_specific_help( "cv-cat", command ) << std::endl; return 0; }
            std::cerr << "acquire images using opencv, apply filters and output with header" << std::endl;
            if( !vm.count( "verbose" ) ) { std::cerr << "see --help --verbose for filters usage" << std::endl; }
            std::cerr << std::endl;
            std::cerr << "usage: cv-cat [options] [<filters>]\n" << std::endl;
            std::cerr << "output header format: fields: t,rows,cols,type; binary: t,3ui\n" << std::endl;
            std::cerr << description << std::endl;
            std::cerr << std::endl;
            std::cerr << "examples" << std::endl;
            std::cerr << "    take bayer-encoded images with 1000 rows and 500 columns, no header" << std::endl;
            std::cerr << "    do bayer conversion, transpose, and output without header to the file converted.bin" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        cat images.bin | cv-cat --input=\"rows=1000;cols=500;no-header;type=ub\" \"bayer=1;transpose\" --output=no-header > converted.bin" << std::endl;
            std::cerr << std::endl;
            std::cerr << "    view the result of the previous example" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        cat converted.bin | cv-cat --input=\"rows=500;cols=1000;no-header;type=3ub\" \"view\" > /dev/null" << std::endl;
            std::cerr << std::endl;
            std::cerr << "    take output of the first found gige camera, resize, view as you go, and save in the file" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        gige-cat | cv-cat \"resize=640,380;view\" > gige-output.bin" << std::endl;
            std::cerr << std::endl;
            std::cerr << "    play back and view gige-output.bin from the previous example" << std::endl;
            std::cerr << "    header format (by default): t,3ui (timestamp, cols, rows, type)" << std::endl;
            std::cerr << "    image size will be 640*380*3=729600" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        cat gige-output.bin | csv-play --binary=t,3ui,729600ub | cv-cat view > /dev/null" << std::endl;
            std::cerr << std::endl;
            std::cerr << "    print image header (e.g. to figure out the image size or type)" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        gige-cat --output=\"header-only;fields=rows,cols,size,type\" | csv-from-bin 4ui | head" << std::endl;
            std::cerr << "    create a video ( -b: bitrate, -r: input/output framerate:" << std::endl;
            std::cerr << "        gige-cat | cv-cat \"encode=ppm\" --output=no-header | avconv -y -f image2pipe -vcodec ppm -r 25 -i pipe: -vcodec libx264  -threads 0 -b 2000k -r 25 video.mkv" << std::endl;
            std::cerr << std::endl;
            std::cerr << "    overlay a ruler on input stream to show scale (the overlay image can be created in a script using csv-to-svg)" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        create_ruler_svg > tmp/r.svg" << std::endl;
            std::cerr << "        convert -background transparent tmp/r.svg tmp/r.png" << std::endl;
            std::cerr << "        ...  | cv-cat \"overlay=tmp/r.png,10,10;view;null\" " << std::endl;
            std::cerr << std::endl;
            if( vm.count( "verbose" ) )
            {
                std::cerr << std::endl;
                std::cerr << snark::cv_mat::serialization::options::usage() << std::endl;
                std::cerr << std::endl;
                std::cerr << snark::cv_mat::filters::usage() << std::endl;
            }
            std::cerr << std::endl;
            return 0;
        }
        if( vm.count( "file" ) + vm.count( "camera" ) + vm.count( "id" ) > 1 ) { std::cerr << "cv-cat: --file, --camera, and --id are mutually exclusive" << std::endl; return 1; }
        if( vm.count( "discard" ) ) { discard = 1; }
        snark::cv_mat::serialization::options input_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( input_options_string );
        snark::cv_mat::serialization::options output_options = output_options_string.empty()
                                                             ? input_options
                                                             : comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( output_options_string );
        std::vector< std::string > filterStrings = boost::program_options::collect_unrecognized( parsed.options, boost::program_options::include_positional );
        std::string filters;
        if( filterStrings.size() == 1 ) { filters = filterStrings[0]; }
        if( filterStrings.size() > 1 ) { std::cerr << "please provide filters as a single name-value string" << std::endl; return 1; }
        if( filters.find( "encode" ) != filters.npos && !output_options.no_header )
        {
            std::cerr << "encoding image and not using no-header, are you sure ?" << std::endl;
        }
        if( vm.count( "camera" ) ) { device = 0; }
        rate_limit rate( fps );
        cv::VideoCapture video_capture;
        snark::cv_mat::serialization input( input_options );
        snark::cv_mat::serialization output( output_options );
        boost::scoped_ptr< bursty_reader< pair > > reader;
        cv::Mat image;
        if( vm.count( "file" ) )
        {
            if( !vm.count( "video" ) ) { image = cv::imread( name ); }
            if( image.data )
            {
                reader.reset( new bursty_reader< pair >( boost::bind( &output_single_image, boost::cref( image ) ), discard, capacity ) );
            }
            else
            {
                video_capture.open( name );
                skip( number_of_frames_to_skip, video_capture, rate );
                reader.reset( new bursty_reader< pair >( boost::bind( &capture, boost::ref( video_capture ), boost::ref( rate ) ), discard, capacity ) );
            }
        }
        else if( vm.count( "camera" ) || vm.count( "id" ) )
        {
            video_capture.open( device );
            skip( number_of_frames_to_skip, video_capture, rate );
            reader.reset( new bursty_reader< pair >( boost::bind( &capture, boost::ref( video_capture ), boost::ref( rate ) ), discard ) );
        }
        else
        {
            skip( number_of_frames_to_skip, input, rate );
            reader.reset( new bursty_reader< pair >( boost::bind( &read, boost::ref( input ), boost::ref( rate ) ), discard, capacity ) );
        }
        const unsigned int default_delay = vm.count( "file" ) == 0 ? 1 : 200; // HACK to make view work on single files
        snark::imaging::applications::pipeline pipeline( output, snark::cv_mat::filters::make( filters, default_delay ), *reader, number_of_threads );
        pipeline.run();
        if( vm.count( "stay" ) ) { while( !is_shutdown ) { boost::this_thread::sleep( boost::posix_time::seconds( 1 ) ); } }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << argv[0] << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << argv[0] << ": unknown exception" << std::endl; }
    return 1;
}
