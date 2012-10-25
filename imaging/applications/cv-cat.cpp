// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifdef WIN32
#include <winsock2.h>
#include <windows.h> 
#endif
#include <iostream>
#include <boost/program_options.hpp>
#include <comma/application/signal_flag.h>
#include <comma/name_value/parser.h>
#include <snark/imaging/cv_mat/pipeline.h>
#include <opencv2/highgui/highgui.hpp>

#ifdef WIN32
#include <fcntl.h>
#include <io.h>
#endif

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

static comma::signal_flag is_shutdown;

static pair capture( cv::VideoCapture& capture, rate_limit& rate )
{
    cv::Mat image;
    capture >> image;
    rate.wait();
    return std::make_pair( boost::posix_time::microsec_clock::universal_time(), image );
}

static pair read( snark::cv_mat::serialization& input, rate_limit& rate )
{
    if( is_shutdown || std::cin.eof() || std::cin.bad() || !std::cin.good() ) { return pair(); }
    rate.wait();
    return input.read( std::cin );
}

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
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "long-help", "display long help message" )
            ( "discard,d", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "camera", "use first available opencv-supported camera" )
            ( "file", boost::program_options::value< std::string >( &name ), "video file name" )
            ( "id", boost::program_options::value< int >( &device ), "specify specific device by id ( OpenCV-supported camera )" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames, default: unlimited" )
            ( "fps", boost::program_options::value< double >( &fps )->default_value( 0 ), "specify max fps ( useful for files, may block if used with cameras ) " )
            ( "input", boost::program_options::value< std::string >( &input_options_string ), "input options, when reading from stdin (see --long-help)" )
            ( "output", boost::program_options::value< std::string >( &output_options_string ), "output options (see --long-help); default: same as --input" )
            ( "stay", "do not close at end of stream" );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) || vm.count( "long-help" ) )
        {
            std::cerr << "acquire images using opencv, apply filters and output with header" << std::endl;
            if( !vm.count( "long-help" ) ) { std::cerr << "see long-help for filters usage" << std::endl; }
            std::cerr << std::endl;
            std::cerr << "usage: cv-cat [options] [<filters>]\n" << std::endl;
            std::cerr << "output header format: fields: t,cols,rows,type; binary: t,3ui\n" << std::endl;
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
            std::cerr << std::endl;
            if ( vm.count( "long-help" ) )
            {
                std::cerr << std::endl;
                std::cerr << snark::cv_mat::serialization::options::usage() << std::endl;
                std::cerr << std::endl;
                std::cerr << snark::cv_mat::filters::usage() << std::endl;
            }
            std::cerr << std::endl;
            return 1;
        }
        if( vm.count( "file" ) + vm.count( "camera" ) + vm.count( "id" ) > 1 ) { std::cerr << "cv-cat: --file, --camera, and --id are mutually exclusive" << std::endl; return 1; }        
        if( vm.count( "discard" ) ) { discard = 1; }
        snark::cv_mat::serialization::options input_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( input_options_string );
        snark::cv_mat::serialization::options output_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( output_options_string, input_options );
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
        cv::VideoCapture videoCapture;
        snark::cv_mat::serialization input( input_options );
        snark::cv_mat::serialization output( output_options );        
        boost::scoped_ptr< bursty_reader< pair > > reader;
        static const unsigned int defaultCapacity = 16;
        if( vm.count( "file" ) )
        {
            videoCapture.open( name );
            reader.reset( new bursty_reader< pair >( boost::bind( &capture, boost::ref( videoCapture ), boost::ref( rate ) ), discard, defaultCapacity ) );
        }
        else if( vm.count( "camera" ) || vm.count( "id" ) )
        {
            videoCapture.open( device );
            reader.reset( new bursty_reader< pair >( boost::bind( &capture, boost::ref( videoCapture ), boost::ref( rate ) ), discard ) );
        }
        else
        {
            reader.reset( new bursty_reader< pair >( boost::bind( &read, boost::ref( input ), boost::ref( rate ) ), discard, defaultCapacity ) );
        }
        snark::imaging::applications::pipeline pipeline( output, filters, *reader );
        pipeline.run();
        if( vm.count( "stay" ) )
        {
            while( !is_shutdown ) { boost::this_thread::sleep( boost::posix_time::seconds( 1 ) ); }
        }
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
