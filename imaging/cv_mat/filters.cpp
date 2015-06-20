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

#include <fstream>
#include <queue>
#include <sstream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/csv/options.h>
#include <comma/string/string.h>
#include <comma/name_value/parser.h>
#include <Eigen/Core>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "filters.h"
#include "serialization.h"

struct map_input_t
{
    typedef double value_type;
    typedef int key_type;
    key_type key;
    value_type value;
};

namespace comma { namespace visiting {

template <> struct traits< map_input_t >
{
    template< typename K, typename V > static void visit( const K&, map_input_t& t, V& v )
    {
        v.apply( "key", t.key );
        v.apply( "value", t.value );
    }
    template< typename K, typename V > static void visit( const K&, const map_input_t& t, V& v )
    {
        v.apply( "key", t.key );
        v.apply( "value", t.value );
    }
};

} } // namespace comma { namespace visiting {

namespace snark{ namespace cv_mat {

static boost::unordered_map< std::string, int > fill_types_()
{
    boost::unordered_map< std::string, int > types;
    types[ "CV_8UC1" ] = types[ "ub" ] = CV_8UC1;
    types[ "CV_8UC2" ] = types[ "2ub" ] = CV_8UC2;
    types[ "CV_8UC3" ] = types[ "3ub" ] = CV_8UC3;
    types[ "CV_8UC4" ] = types[ "4ub" ] = CV_8UC4;
    types[ "CV_8SC1" ] = types[ "b" ] = CV_8SC1;
    types[ "CV_8SC2" ] = types[ "2b" ] = CV_8SC2;
    types[ "CV_8SC3" ] = types[ "3b" ] = CV_8SC3;
    types[ "CV_8SC4" ] = types[ "4b" ] = CV_8SC4;
    types[ "CV_16UC1" ] = types[ "uw" ] = CV_16UC1;
    types[ "CV_16UC2" ] = types[ "2uw" ] = CV_16UC2;
    types[ "CV_16UC3" ] = types[ "3uw" ] = CV_16UC3;
    types[ "CV_16UC4" ] = types[ "4uw" ] = CV_16UC4;
    types[ "CV_16SC1" ] = types[ "w" ] = CV_16SC1;
    types[ "CV_16SC2" ] = types[ "2w" ] = CV_16SC2;
    types[ "CV_16SC3" ] = types[ "3w" ] = CV_16SC3;
    types[ "CV_16SC4" ] = types[ "4w" ] = CV_16SC4;
    types[ "CV_32SC1" ] = types[ "i" ] = CV_32SC1;
    types[ "CV_32SC2" ] = types[ "2i" ] = CV_32SC2;
    types[ "CV_32SC3" ] = types[ "3i" ] = CV_32SC3;
    types[ "CV_32SC4" ] = types[ "4i" ] = CV_32SC4;
    types[ "CV_32FC1" ] = types[ "f" ] = CV_32FC1;
    types[ "CV_32FC2" ] = types[ "2f" ] = CV_32FC2;
    types[ "CV_32FC3" ] = types[ "3f" ] = CV_32FC3;
    types[ "CV_32FC4" ] = types[ "4f" ] = CV_32FC4;
    types[ "CV_64FC1" ] = types[ "d" ] = CV_64FC1;
    types[ "CV_64FC2" ] = types[ "2d" ] = CV_64FC2;
    types[ "CV_64FC3" ] = types[ "3d" ] = CV_64FC3;
    types[ "CV_64FC4" ] = types[ "4d" ] = CV_64FC4;
    return types;
}

static boost::unordered_map< int, std::string > fill_types_as_string_()
{
    boost::unordered_map< int, std::string > types;
    types[ CV_8UC1 ] = "CV_8UC1";
    types[ CV_8UC2 ] = "CV_8UC2";
    types[ CV_8UC3 ] = "CV_8UC3";
    types[ CV_8UC4 ] = "CV_8UC4";
    types[ CV_8SC1 ] = "CV_8SC1";
    types[ CV_8SC2 ] = "CV_8SC2";
    types[ CV_8SC3 ] = "CV_8SC3";
    types[ CV_8SC4 ] = "CV_8SC4";
    types[ CV_16UC1 ] = "CV_16UC1";
    types[ CV_16UC2 ] = "CV_16UC2";
    types[ CV_16UC3 ] = "CV_16UC3";
    types[ CV_16UC4 ] = "CV_16UC4";
    types[ CV_16SC1 ] = "CV_16SC1";
    types[ CV_16SC2 ] = "CV_16SC2";
    types[ CV_16SC3 ] = "CV_16SC3";
    types[ CV_16SC4 ] = "CV_16SC4";
    types[ CV_32SC1 ] = "CV_32SC1";
    types[ CV_32SC2 ] = "CV_32SC2";
    types[ CV_32SC3 ] = "CV_32SC3";
    types[ CV_32SC4 ] = "CV_32SC4";
    types[ CV_32FC1 ] = "CV_32FC1";
    types[ CV_32FC2 ] = "CV_32FC2";
    types[ CV_32FC3 ] = "CV_32FC3";
    types[ CV_32FC4 ] = "CV_32FC4";
    types[ CV_64FC1 ] = "CV_64FC1";
    types[ CV_64FC2 ] = "CV_64FC2";
    types[ CV_64FC3 ] = "CV_64FC3";
    types[ CV_64FC4 ] = "CV_64FC4";
    return types;
}

static const boost::unordered_map< std::string, int > types_ = fill_types_();
static const boost::unordered_map< int, std::string > types_as_string = fill_types_as_string_();

static std::string type_as_string( int t ) // to avoid compilation warning
{
    boost::unordered_map< int, std::string >::const_iterator it = types_as_string.find( t );
    return it == types_as_string.end() ? boost::lexical_cast< std::string >( t ) : it->second;
}
    
static filters::value_type cvt_color_impl_( filters::value_type m, unsigned int which )
{
    filters::value_type n;
    n.first = m.first;
    if( m.second.channels() == 3 )
    {
        cv::Mat grey;
        cv::cvtColor( m.second, grey, CV_RGB2GRAY );
        m.second = grey;
    }
    cv::cvtColor( m.second, n.second, which + 45u ); // HACK, bayer as unsigned int, but I don't find enum { BG2RGB, GB2BGR ... } more usefull
    return n;
}

static filters::value_type head_impl_( filters::value_type m, unsigned int number_of_frames )
{
    static unsigned int frame_number = 0;
    if( frame_number < number_of_frames ) { frame_number++; return m; } else { return filters::value_type(); }
}

static filters::value_type crop_impl_( filters::value_type m, unsigned int x, unsigned int y, unsigned int w, unsigned int h )
{
    return filters::value_type( m.first, cv::Mat( m.second, cv::Rect( x, y, w, h ) ) );
}

typedef std::pair< unsigned int, unsigned int > tile_t;

static filters::value_type crop_tile_impl_( filters::value_type input, unsigned int number_of_tile_cols, unsigned int number_of_tile_rows, const std::vector< tile_t >& tiles, bool vertical )
{
    unsigned int w = input.second.cols / number_of_tile_cols;
    unsigned int h = input.second.rows / number_of_tile_rows;
    unsigned int s = tiles.size();
    filters::value_type output( input.first, cv::Mat( vertical ? h*s : h, vertical ? w : w*s, input.second.type() ) );
    for( std::size_t i = 0; i < tiles.size(); ++i)
    {
        unsigned int x = tiles[i].first * w;
        unsigned int y = tiles[i].second * h;
        cv::Mat tile( output.second,  cv::Rect( vertical ? 0 : i*w, vertical ? i*h: 0, w, h ) );
        cv::Mat( input.second, cv::Rect( x, y, w, h ) ).copyTo( tile );
    }
    return output;
}

class accumulate_impl_
{
    public:
        accumulate_impl_( unsigned int how_many ) : how_many_ ( how_many ), defined_ ( false ) {}
        filters::value_type operator()( filters::value_type input )
        {
            if( !defined_ )
            {
                cols_ = input.second.cols;
                h_ = input.second.rows;
                rows_ = h_ * how_many_;
                type_ = input.second.type();
                accumulated_image_ = cv::Mat::zeros( rows_, cols_, type_ );
                rect_for_new_data_ = cv::Rect( 0, 0, cols_, h_ );
                rect_for_old_data_ = cv::Rect( 0, h_, cols_, rows_ - h_ );
                rect_to_keep_ = cv::Rect( 0, 0, cols_, rows_ - h_ );                
                defined_ = true;
            }
            if( input.second.cols != cols_ ) { COMMA_THROW( comma::exception, "accumulate: expected input image with " << cols_ << " columns, got " << input.second.cols << " columns"); }
            if( input.second.rows != h_ ) { COMMA_THROW( comma::exception, "accumulate: expected input image with " << h_ << " rows, got " << input.second.rows << " rows"); }
            if( input.second.type() != type_ ) { COMMA_THROW( comma::exception, "accumulate: expected input image of type " << type_ << ", got type " << input.second.type() << " rows"); }
            filters::value_type output( input.first, cv::Mat( accumulated_image_.size(), accumulated_image_.type() ) );
            cv::Mat new_data( output.second, rect_for_new_data_ );
            input.second.copyTo( new_data );
            cv::Mat old_data( output.second, rect_for_old_data_ );
            cv::Mat( accumulated_image_, rect_to_keep_ ).copyTo( old_data );
            output.second.copyTo( accumulated_image_ );
            return output;
        }
    private:
        unsigned int how_many_;
        bool defined_;
        int cols_, h_, rows_, type_;
        cv::Rect rect_for_new_data_, rect_for_old_data_, rect_to_keep_;
        cv::Mat accumulated_image_;
};

static filters::value_type convert_to_impl_( filters::value_type m, int type, double scale, double offset )
{
    filters::value_type n;
    n.first = m.first;
    m.second.convertTo( n.second, type, scale, offset );
    return n;
}

static filters::value_type flip_impl_( filters::value_type m, int how )
{
    filters::value_type n;
    n.first = m.first;
    cv::flip( m.second, n.second, how );
    return n;
}

static filters::value_type resize_impl_( filters::value_type m, unsigned int width, unsigned int height, double w, double h )
{
    filters::value_type n;
    n.first = m.first;
    cv::resize( m.second, n.second, cv::Size( width ? width : m.second.cols * w, height ? height : m.second.rows * h ) );
    return n;
}

static filters::value_type brightness_impl_( filters::value_type m, double scale, double offset )
{
    filters::value_type n;
    n.first = m.first;
    n.second = (m.second * scale) + offset;
    return n;
}

static filters::value_type transpose_impl_( filters::value_type m )
{
    filters::value_type n;
    n.first = m.first;
    cv::transpose( m.second, n.second );
    return n;
}

static int single_channel_type_( int t )
{
    switch( t )
    {
        case CV_8UC1:
        case CV_8UC2:
        case CV_8UC3:
        case CV_8UC4:
            return CV_8UC1;
        case CV_8SC1:
        case CV_8SC2:
        case CV_8SC3:
        case CV_8SC4:
            return CV_8SC1;
        case CV_16UC1:
        case CV_16UC2:
        case CV_16UC3:
        case CV_16UC4:
            return CV_16UC1;
        case CV_16SC1:
        case CV_16SC2:
        case CV_16SC3:
        case CV_16SC4:
            return CV_16SC1;
        case CV_32SC1:
        case CV_32SC2:
        case CV_32SC3:
        case CV_32SC4:
            return CV_32SC1;
        case CV_32FC1:
        case CV_32FC2:
        case CV_32FC3:
        case CV_32FC4:
            return CV_32FC1;
        case CV_64FC1:
        case CV_64FC2:
        case CV_64FC3:
        case CV_64FC4:
            return CV_64FC1;
    }
    return CV_8UC1;
}

static filters::value_type split_impl_( filters::value_type m )
{
    filters::value_type n;
    n.first = m.first;
    n.second = cv::Mat( m.second.rows * m.second.channels(), m.second.cols, single_channel_type_( m.second.type() ) ); // todo: check number of channels!
    std::vector< cv::Mat > channels;
    channels.reserve( m.second.channels() );    
    for( unsigned int i = 0; i < static_cast< unsigned int >( m.second.channels() ); ++i )
    {
        channels.push_back( cv::Mat( n.second, cv::Rect( 0, i * m.second.rows, m.second.cols, m.second.rows ) ) );
    }
    cv::split( m.second, channels );
    return n;
}

static filters::value_type merge_impl_( filters::value_type m, unsigned int nchannels )
{
    filters::value_type n;
    n.first = m.first;
    if( m.second.rows % nchannels != 0 ) { COMMA_THROW( comma::exception, "merge: expected " << nchannels << " horizontal strips of equal height, got " << m.second.rows << " rows, which is not a multiple of " << nchannels ); }
    std::vector< cv::Mat > channels( nchannels );
    for( std::size_t i = 0; i < nchannels; ++i ) { channels[i] = cv::Mat( m.second, cv::Rect( 0, i * m.second.rows / nchannels, m.second.cols, m.second.rows / nchannels ) ); }
    cv::merge( channels, n.second ); 
    return n;
}

static filters::value_type view_impl_( filters::value_type m, std::string name, unsigned int delay )
{
    cv::imshow( &name[0], m.second );
    char c = cv::waitKey( delay );
    if( c == 27 ) { return filters::value_type(); } // HACK to notify application to exit
    if( c == ' ' )
    {
        std::stringstream filename;
        filename <<  boost::posix_time::to_iso_string( m.first.is_not_a_date_time() ? boost::posix_time::microsec_clock::universal_time() : m.first ) << ".ppm";
        cv::imwrite( filename.str(), m.second );
    }
    return m;
}

static filters::value_type thumb_impl_( filters::value_type m, std::string name, unsigned int cols = 100, unsigned int delay = 1 )
{
    cv::Mat n;
    unsigned int rows = m.second.rows * ( double( cols ) / m.second.cols );
    if( rows == 0 ) { rows = 1; }
    cv::resize( m.second, n, cv::Size( cols, rows ) );
    cv::imshow( &name[0], n );
    char c = cv::waitKey( delay );
    return c == 27 ? filters::value_type() : m; // HACK to notify application to exit
}

static filters::value_type cross_impl_( filters::value_type m, boost::optional< Eigen::Vector2i > xy )
{
    if( !xy )
    {
        xy = Eigen::Vector2i();
        xy->x() = m.second.size().width / 2;
        xy->y() = m.second.size().height / 2;
    }
    cv::circle( m.second, cv::Point( xy->x(), xy->y() ), 4, cv::Scalar( 0, 255, 0 ), 1, CV_AA );
    cv::line( m.second, cv::Point( xy->x(), 0 ), cv::Point( xy->x(), m.second.size().height ), cv::Scalar( 0, 255, 0 ) );
    cv::line( m.second, cv::Point( 0, xy->y() ), cv::Point( m.second.size().width, xy->y() ), cv::Scalar( 0, 255, 0 ) );
    return m;
}

static filters::value_type encode_impl_( filters::value_type m, const std::string& type )
{
    std::vector< unsigned char > buffer;
    std::string format = "." + type;
    cv::imencode( format, m.second, buffer );
    filters::value_type p;
    p.first = m.first;
    p.second = cv::Mat( buffer.size(), 1, CV_8UC1 );
    ::memcpy( p.second.data, &buffer[0] , buffer.size() );
    return p;
}

static comma::csv::options make_header_csv()
{
    comma::csv::options csv;
    csv.fields = "t,rows,cols,type";
    csv.format( "t,3ui" );
    return csv;
}

static filters::value_type histogram_impl_( filters::value_type m )
{
    static comma::csv::output_stream< serialization::header > os( std::cout, make_header_csv() ); // todo: quick and dirty; generalize imaging::serialization::pipeline
    if( single_channel_type_( m.second.type() ) != CV_8UC1 ) { std::cerr << "cv-cat: histogram: expected an unsigned char image type; got " << type_as_string( m.second.type() ) << std::endl; exit( 1 ); }
    typedef boost::array< comma::uint32, 256 > channel_t;
    std::vector< channel_t > channels( m.second.channels() );
    for( unsigned int i = 0; i < channels.size(); ++i ) { ::memset( ( char* )( &channels[i][0] ), 0, sizeof( comma::uint32 ) * 256 ); }
    for( const unsigned char* p = m.second.datastart; p < m.second.dataend; )
    {
        for( unsigned int i = 0; i < channels.size(); ++channels[i][*p], ++i, ++p );
    }
    serialization::header h;
    h.timestamp = m.first;
    h.rows = m.second.rows;
    h.cols = m.second.cols;
    h.type = m.second.type();
    os.write( h );
    os.flush();
    for( unsigned int i = 0; i < channels.size(); ++i ) { std::cout.write( ( char* )( &channels[i][0] ), sizeof( comma::uint32 ) * 256 ); }
    return m;
}

static filters::value_type grab_impl_( filters::value_type m, const std::string& type )
{
    std::string filename = boost::posix_time::to_iso_string( m.first.is_not_a_date_time() ? boost::posix_time::microsec_clock::universal_time() : m.first );
    filename += "." + type;
    cv::imwrite( filename, m.second );
    return filters::value_type(); // HACK to notify application to exit
}

static filters::value_type file_impl_( filters::value_type m, const std::string& type )
{
    std::string filename = boost::posix_time::to_iso_string( m.first.is_not_a_date_time() ? boost::posix_time::microsec_clock::universal_time() : m.first );
    filename += "." + type;
    cv::imwrite( filename, m.second );
    return m;
}

static filters::value_type timestamp_impl_( filters::value_type m )
{
    cv::rectangle( m.second, cv::Point( 5, 5 ), cv::Point( 228, 25 ), cv::Scalar( 0xffff, 0xffff, 0xffff ), CV_FILLED, CV_AA );
    cv::putText( m.second, boost::posix_time::to_iso_string( m.first ), cv::Point( 10, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA );
    return m;
}

struct count_impl_
{
    count_impl_() : count( 0 ) {}
    
    unsigned int count;
    
    filters::value_type operator()( filters::value_type m )
    {
        cv::rectangle( m.second, cv::Point( 5, 5 ), cv::Point( 80, 25 ), cv::Scalar( 0xffff, 0xffff, 0xffff ), CV_FILLED, CV_AA );
        cv::putText( m.second, boost::lexical_cast< std::string >( count++ ), cv::Point( 10, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA );
        return m;
    }
};

static filters::value_type invert_impl_( filters::value_type m )
{
    if( m.second.type() != CV_8UC1 && m.second.type() != CV_8UC2 && m.second.type() != CV_8UC3 && m.second.type() != CV_8UC4 ) { COMMA_THROW( comma::exception, "expected image type ub, 2ub, 3ub, 4ub; got: " << type_as_string( m.second.type() ) ); }
    for( unsigned char* c = m.second.datastart; c < m.second.dataend; *c = 255 - *c, ++c );
    return m;
}

static filters::value_type text_impl_( filters::value_type m, const std::string& s, const cv::Point& origin, const cv::Scalar& colour )
{
    cv::putText( m.second, s, origin, cv::FONT_HERSHEY_SIMPLEX, 1.0, colour, 1, CV_AA );
    return m;
}

class undistort_impl_
{
    public:
        undistort_impl_( const std::string& filename ) : filename_( filename ) {}

        filters::value_type operator()( filters::value_type m )
        {
            init_map_( m.second.rows, m.second.cols );
            filters::value_type n( m.first, cv::Mat( m.second.size(), m.second.type(), cv::Scalar::all(0) ) );
            cv::remap( m.second, n.second, x_, y_, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT );
            return n;
        }

    private:
        std::string filename_;
        std::vector< char > xbuf_;
        std::vector< char > ybuf_;
        cv::Mat x_;
        cv::Mat y_;
        void init_map_( unsigned int rows, unsigned int cols )
        {
            if( !x_.empty() ) { return; }
            std::ifstream stream( filename_.c_str() );
            if( !stream ) { COMMA_THROW( comma::exception, "failed to open undistort map in \"" << filename_ << "\"" ); }
            std::size_t size = rows * cols * 4;
            xbuf_.resize( size );
            stream.read( &xbuf_[0], size );
            if( stream.gcount() < 0 || std::size_t( stream.gcount() ) != size ) { COMMA_THROW( comma::exception, "failed to read \"" << filename_ << "\"" ); }
            ybuf_.resize( size );
            stream.read( &ybuf_[0], size );
            if( stream.gcount() < 0 || std::size_t( stream.gcount() ) != size ) { COMMA_THROW( comma::exception, "failed to read \"" << filename_ << "\"" ); }
            stream.peek(); // quick and dirty
            if( !stream.eof() ) { COMMA_THROW( comma::exception, "expected " << ( size * 2 ) << " bytes in \"" << filename_ << "\", got more" ); }
            x_ = cv::Mat( rows, cols, CV_32FC1, &xbuf_[0] );
            y_ = cv::Mat( rows, cols, CV_32FC1, &ybuf_[0] );
        }
};

class max_impl_ // experimental, to debug
{
    public:
        max_impl_( unsigned int size, bool is_max ) : size_( size ), is_max_( is_max ) {}

        filters::value_type operator()( filters::value_type m )
        {
            if( deque_.size() == size_ ) { deque_.pop_front(); }
            deque_.push_back( filters::value_type() );
            m.second.copyTo( deque_.back().second );
            filters::value_type s( m.first, cv::Mat( m.second.rows, m.second.cols, m.second.type() ) );
            ::memset( m.second.datastart, 0, m.second.rows * m.second.cols * m.second.channels() );
            static unsigned int count = 0;
            for( unsigned int i = 0; i < deque_.size(); ++i )
            {
                unsigned char* p = deque_[i].second.datastart;
                for( unsigned char* q = s.second.datastart; q < s.second.dataend; *q = is_max_ ? std::max( *p, *q ) : std::min( *p, *q ), ++p, ++q );
            }
            ++count;
            return s;
        }

    private:
        unsigned int size_;
        bool is_max_;
        std::deque< filters::value_type > deque_; // use vector?
};

class map_impl_
{
    typedef int key_type;
    typedef double output_value_type;
    public:
        map_impl_( const std::string& map_filter_options, bool permissive ) : permissive_ ( permissive )
        {
            comma::csv::options csv_options = comma::name_value::parser( "filename", '&' , '=' ).get< comma::csv::options >( map_filter_options );
            std::string default_csv_fields = "value";
            bool no_key_field = true;
            if( csv_options.fields.empty() ) { csv_options.fields = default_csv_fields; }
            else
            {
                if( !csv_options.has_field( "value" ) ) { COMMA_THROW( comma::exception, "map filter: fields option is given but \"value\" field is not found" ); }
                no_key_field = !csv_options.has_field( "key" );
            }    
            std::ifstream ifs( &csv_options.filename[0] );
            if( !ifs ) { COMMA_THROW( comma::exception, "map filter: failed to open \"" << csv_options.filename << "\"" ); }
            BOOST_STATIC_ASSERT( ( boost::is_same< map_input_t::key_type, key_type >::value ) );
            BOOST_STATIC_ASSERT( ( boost::is_same< map_input_t::value_type, output_value_type >::value ) );
            comma::csv::input_stream< map_input_t > map_stream( ifs , csv_options );
            for( key_type counter = 0; map_stream.ready() || ( ifs.good() && !ifs.eof() ) ; ++counter )
            {
                const map_input_t* map_input = map_stream.read();
                if( !map_input ) { break; }
                key_type key = no_key_field ? counter : map_input->key;
                map_.insert( std::pair< key_type, output_value_type >( key, map_input->value ) );
            }
        }
        
        filters::value_type operator()( filters::value_type m )
        {
            if( m.second.channels() != 1 ) { std::cerr << "map filter: expected single channel cv type, got " << m.second.channels() << " channels" << std::endl; return filters::value_type(); }
            filters::value_type n( m.first, cv::Mat( m.second.size(), cv::DataType< output_value_type >::type ) );
            try 
            {
                switch( m.second.type() )
                {
                    case cv::DataType< unsigned char >::type : apply_map< unsigned char >( m.second, n.second ); break;
                    case cv::DataType< comma::uint16 >::type : apply_map< comma::uint16 >( m.second, n.second ); break;
                    case cv::DataType< char >::type : apply_map< char >( m.second, n.second ); break;
                    case cv::DataType< comma::int16 >::type : apply_map< comma::int16 >( m.second, n.second ); break;
                    case cv::DataType< comma::int32 >::type : apply_map< comma::int32 >( m.second, n.second ); break;
                    default: std::cerr << "map filter: expected integer cv type, got " << m.second.type() << std::endl; return filters::value_type(); 
                }
            } catch ( std::out_of_range ) { return filters::value_type(); }
            return n;
        }
        
    private:
        typedef boost::unordered_map< key_type, output_value_type > map_t_;
        map_t_ map_;
        bool permissive_;
        
        template < typename input_value_type >
        void apply_map( const cv::Mat& input, cv::Mat& output )
        {
            for( int i=0; i < input.rows; ++i )
            {
                for( int j=0; j < input.cols; ++j )
                {
                    key_type key = input.at< input_value_type >(i,j);
                    map_t_::const_iterator it = map_.find( key );
                    if( it != map_.end() ) { output.at< output_value_type >(i,j) = map_.at( key ); }
                    else
                    {
                        if( permissive_ ) { output.at< output_value_type >(i,j) = key; } 
                        else { std::cerr << "map filter: expected a pixel value from the map, got: pixel at " << i << "," << j << " with value " << key << std::endl; throw std::out_of_range(""); }
                    }
                }
            }
        }
};

static filters::value_type magnitude_impl_( filters::value_type m )
{
    if( m.second.channels() != 2 ) { std::cerr << "cv filters: magnitude: expected 2 channels, got " << m.second.channels() << std::endl; return filters::value_type(); }
    boost::array< cv::Mat, 2 > planes;
    filters::value_type n;
    n.first = m.first;
    cv::split( m.second, &planes[0] );
    cv::magnitude( planes[0], planes[1], n.second );
    return n;
}

static filters::value_type convert( filters::value_type m, bool scale, bool complex, bool magnitude, bool log_scale, bool normalize )
{
    filters::value_type n;
    n.first = m.first;
    cv::dft( m.second, n.second, ( scale ? cv::DFT_SCALE : cv::DFT_INVERSE ) | ( complex ? cv::DFT_COMPLEX_OUTPUT : cv::DFT_REAL_OUTPUT ) );
    if( !magnitude ) { return n; }
    boost::array< cv::Mat, 2 > planes = {{ cv::Mat::zeros( m.second.size(), m.second.type() ), cv::Mat::zeros( m.second.size(), m.second.type() ) }};
    cv::split( n.second, &planes[0] );
    cv::magnitude( planes[0], planes[1], n.second ); // make separate filters: magnitude, log, scale, normalize?
    if( log_scale )
    {
        n.second += cv::Scalar::all( 1 );
        cv::log( n.second, n.second ); // todo: optional
    }
    if( normalize ) { cv::normalize( n.second, n.second, 0, 1, CV_MINMAX ); }
    return n;
}

template < typename T, int Type >
static filters::value_type convert( filters::value_type m, bool magnitude, bool log_scale, bool normalize )
{
    cv::Mat padded;
    int padded_rows = cv::getOptimalDFTSize( m.second.rows );
    int padded_cols = cv::getOptimalDFTSize( m.second.cols );
    cv::copyMakeBorder( m.second, padded, 0, padded_rows - m.second.rows, 0, padded_cols - m.second.cols, cv::BORDER_CONSTANT, cv::Scalar::all( 0 ) );
    boost::array< cv::Mat, 2 > planes = {{ cv::Mat_< T >( padded ), cv::Mat::zeros( padded.size(), Type ) }};
    filters::value_type p;
    p.first = m.first;
    cv::merge( &planes[0], 2, p.second );
    cv::dft( p.second, p.second );
    if( !magnitude ) { return p; }
    cv::split( p.second, &planes[0] );
    cv::magnitude( planes[0], planes[1], planes[0] );
    filters::value_type n;
    n.first = m.first;
    n.second = planes[0];
    if( log_scale )
    {
        n.second += cv::Scalar::all( 1 );
        cv::log( n.second, n.second ); // todo: optional
    }
    n.second = n.second( cv::Rect( 0, 0, n.second.cols & -2, n.second.rows & -2 ) );

    int cx = n.second.cols / 2 ;
    int cy = n.second.rows / 2 ;
    
    cv::Mat q0(n.second, cv::Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    cv::Mat q1(n.second, cv::Rect(cx, 0, cx, cy));  // Top-Right
    cv::Mat q2(n.second, cv::Rect(0, cy, cx, cy));  // Bottom-Left
    cv::Mat q3(n.second, cv::Rect(cx, cy, cx, cy)); // Bottom-Right
    
    cv::Mat tmp;
    q0.copyTo( tmp ); // swap top-left with bottom-right
    q3.copyTo( q0 );
    tmp.copyTo( q3 );
    q1.copyTo( tmp ); // swap top-right with bottom-left
    q2.copyTo( q1 );
    tmp.copyTo( q2 );
    
    if( normalize ) { cv::normalize( n.second, n.second, 0, 1, CV_MINMAX ); }
    return n;
}

filters::value_type fft_impl_( filters::value_type m, bool direct, bool complex, bool magnitude, bool log_scale, bool normalize )
{
    switch( m.second.type() )
    {
        case CV_32FC1:
            //return convert< float, CV_32FC1 >( m, magnitude, log_scale, normalize );
        case CV_32FC2:
            return convert( m, direct, complex, magnitude, log_scale, normalize );
        case CV_32FC3:
        case CV_32FC4:
            std::cerr << "fft: multichannel image support: todo, got: " << type_as_string( m.second.type() ) << std::endl;
            return filters::value_type();
        case CV_64FC1:
            //return convert< double, CV_64FC1 >( m, magnitude, log_scale, normalize, normalize );
        case CV_64FC2:
            return convert( m, direct, complex, magnitude, log_scale, normalize );
        case CV_64FC3:
        case CV_64FC4:
            std::cerr << "fft: multichannel image support: todo, got: " << type_as_string( m.second.type() ) << std::endl;
            return filters::value_type();
        default:
            std::cerr << "fft: expected a floating-point image type, got: " << type_as_string( m.second.type() ) << std::endl;
            return filters::value_type();
    }
}

std::vector< filter > filters::make( const std::string& how, unsigned int default_delay )
{
    std::vector< std::string > v = comma::split( how, ';' );
    std::vector< filter > f;
    if( how == "" ) { return f; }
    std::string name;
    bool modified = false;
    for( std::size_t i = 0; i < v.size(); name += ( i > 0 ? ";" : "" ) + v[i], ++i )
    {
        std::vector< std::string > e = comma::split( v[i], '=' );
        if( e[0] == "bayer" )
        {
            if( modified ) { COMMA_THROW( comma::exception, "cannot covert from bayer after transforms: " << name ); }
            unsigned int which = boost::lexical_cast< unsigned int >( e[1] );
            f.push_back( filter( boost::bind( &cvt_color_impl_, _1, which ) ) );
        }
        else if( e[0] == "count" )
        {
            count_impl_ c;
            f.push_back( filter( c ) );
        }
        else if( e[0] == "crop" )
        {
            unsigned int x = 0;
            unsigned int y = 0;
            unsigned int w, h;
            std::vector< std::string > s = comma::split( e[1], ',' );
            switch( s.size() )
            {
                case 2:
                    w = boost::lexical_cast< unsigned int >( s[0] );
                    h = boost::lexical_cast< unsigned int >( s[1] );
                    break;
                case 4:
                    x = boost::lexical_cast< unsigned int >( s[0] );
                    y = boost::lexical_cast< unsigned int >( s[1] );
                    w = boost::lexical_cast< unsigned int >( s[2] );
                    h = boost::lexical_cast< unsigned int >( s[3] );
                    break;
                default:
                    COMMA_THROW( comma::exception, "expected crop=[x,y,]width,height, got \"" << v[i] << "\"" );
            }
            f.push_back( filter( boost::bind( &crop_impl_, _1, x, y, w, h ) ) );
        }
        else if( e[0] == "crop-tile" )
        {
            if( e.size() < 2 ) { COMMA_THROW( comma::exception, "crop-tile: specify number of tiles along x and y, and at least one tile, e.g. crop-tile=1,1,0,0" ); }
            std::vector< std::string > items = comma::split( e[1], '&' );
            bool vertical = std::find( items.begin()+1, items.end(), "horizontal" ) == items.end();
            std::vector< std::string > s = comma::split( items[0], ',' );
            if( s.size() < 4 ) { COMMA_THROW( comma::exception, "crop-tile: expected number of tiles along x and y, and at least one tile, got " << e[1] ); }
            if( s.size()%2 != 0 ) { COMMA_THROW( comma::exception, "crop-tile: expected number of tiles along x and y, followed by pairs of integers representing tiles, got " << e[1] ); }
            std::vector< unsigned int > v( s.size() );
            for( std::size_t i=0; i < s.size(); ++i ) { v[i] = boost::lexical_cast< unsigned int >( s[i] ); }
            unsigned int number_of_tile_cols, number_of_tile_rows;
            std::vector< tile_t > tiles;
            if( v.size() == 4 && v[0] < v[2] && v[1] < v[3]) // for backward compatibility only
            {
                number_of_tile_cols = v[2];
                number_of_tile_rows = v[3];
                tiles.push_back( tile_t( v[0], v[1] ) );
            }
            else
            {
                number_of_tile_cols = v[0];
                number_of_tile_rows = v[1];
                for( std::size_t i=2; i < v.size()-1; i+=2 )
                { 
                    if( v[i] >= number_of_tile_cols || v[i+1] >= number_of_tile_rows ) { COMMA_THROW( comma::exception, "crop-tile: encountered an invalid tile " << v[i] << "," << v[i+1] ); }
                    tiles.push_back( tile_t( v[i], v[i+1] ) ); 
                }
            }
            if( number_of_tile_cols == 0 || number_of_tile_rows == 0 ) { COMMA_THROW( comma::exception, "crop-tile: expected positive number of tiles along x and y, got " << number_of_tile_cols << "," << number_of_tile_rows ); }
            f.push_back( filter( boost::bind( &crop_tile_impl_, _1, number_of_tile_cols, number_of_tile_rows, tiles, vertical ) ) );
        }
        else if( e[0] == "accumulate" )
        {
            unsigned int how_many = boost::lexical_cast< unsigned int >( e[1] );
            if ( how_many == 0 ) { COMMA_THROW( comma::exception, "expected positive number of images to accumulate in accumulate filter, got " << how_many ); }
            f.push_back( filter( accumulate_impl_( how_many ) ) );
        }
        else if( e[0] == "cross" )
        {
            boost::optional< Eigen::Vector2i > center;
            if( e.size() > 1 )
            {
                center = Eigen::Vector2i( 0, 0 );
                std::vector< std::string > s = comma::split( e[1], ',' );
                if( s.size() < 2 ) { COMMA_THROW( comma::exception, "expected cross-hair x,y; got \"" << e[1] << "\"" ); }
                center->x() = boost::lexical_cast< unsigned int >( s[0] );
                center->y() = boost::lexical_cast< unsigned int >( s[1] );
            }
            f.push_back( filter( boost::bind( &cross_impl_, _1, center ) ) );
        }
        else if( e[0] == "fft" )
        {
            bool direct = true;
            bool complex = true;
            bool magnitude = false;
            bool log_scale = false;
            bool normalize = false;
            if( e.size() > 1 )
            {
                const std::vector< std::string >& w = comma::split( e[1], ',' );
                for( unsigned int i = 0; i < w.size(); ++i )
                {
                    if( w[i] == "direct" ) { direct = true; }
                    else if( w[i] == "inverse" ) { direct = false; }
                    else if( w[i] == "complex" ) { complex = true; }
                    else if( w[i] == "real" ) { complex = false; }
                    else if( w[i] == "magnitude" ) { magnitude = true; }
                    else if( w[i] == "normalize" ) { normalize = true; }
                    else if( w[i] == "log" || w[i] == "log-scale" ) { log_scale = true; }
                }
            }
            f.push_back( filter( boost::bind( &fft_impl_, _1, direct, complex, magnitude, log_scale, normalize ) ) );
        }
        else if( e[0] == "flip" )
        {
            f.push_back( filter( boost::bind( &flip_impl_, _1, 0 ) ) );
        }
        else if( e[0] == "flop" )
        {
            f.push_back( filter( boost::bind( &flip_impl_, _1, 1 ) ) );
        }
        else if( e[0] == "magnitude" )
        {
            f.push_back( filter( boost::bind( &magnitude_impl_, _1 ) ) );
        }
        else if( e[0] == "text" )
        {
            if( e.size() <= 1 ) { COMMA_THROW( comma::exception, "expected text value" ); }
            std::vector< std::string > w = comma::split( e[1], ',' );
            cv::Point p( 10, 10 );
            if( w.size() >= 3 ) { p = cv::Point( boost::lexical_cast< unsigned int >( w[1] ), boost::lexical_cast< unsigned int >( w[2] ) ); }
            cv::Scalar s( 0, 255, 255 );
            if( w.size() >= 4 )
            {
                if( w[3] == "red" ) { s = cv::Scalar( 0, 0, 255 ); }
                else if( w[3] == "green" ) { s = cv::Scalar( 0, 255, 0 ); }
                else if( w[3] == "blue" ) { s = cv::Scalar( 255, 0, 0 ); }
                else if( w[3] == "white" ) { s = cv::Scalar( 255, 255, 255 ); }
                else if( w[3] == "black" ) { s = cv::Scalar( 0, 0, 0 ); }
                else if( w[3] == "yellow" ) { s = cv::Scalar( 0, 255, 255 ); }
                else { COMMA_THROW( comma::exception, "expected colour of text in \"" << v[i] << "\", got '" << w[3] << "'" ); }
            }
            f.push_back( filter( boost::bind( &text_impl_, _1, w[0], p, s ) ) );
        }
        else if( e[0] == "convert-to" || e[0] == "convert_to" )
        {
            if( e.size() <= 1 ) { COMMA_THROW( comma::exception, "convert-to: expected options, got none" ); }
            const std::vector< std::string >& w = comma::split( e[1], ',' );
            boost::unordered_map< std::string, int >::const_iterator it = types_.find( w[0] );
            if( it == types_.end() ) { COMMA_THROW( comma::exception, "convert-to: expected target type, got \"" << w[0] << "\"" ); }
            double scale = w.size() > 1 ? boost::lexical_cast< double >( w[1] ) : 1.0;
            double offset = w.size() > 2 ? boost::lexical_cast< double >( w[2] ) : 0.0;
            f.push_back( filter( boost::bind( &convert_to_impl_, _1, it->second, scale, offset ) ) );
        }
        else if( e[0] == "resize" )
        {
            unsigned int width = 0;
            unsigned int height = 0;
            double w = 0;
            double h = 0;
            std::vector< std::string > r = comma::split( e[1], ',' );
            switch( r.size() )
            {
                case 1:
                    w = h = boost::lexical_cast< double >( r[0] );
                    break;
                case 2:
                    try { width = boost::lexical_cast< unsigned int >( r[0] ); }
                    catch ( ... ) { w = boost::lexical_cast< double >( r[0] ); }
                    try { height = boost::lexical_cast< unsigned int >( r[1] ); }
                    catch ( ... ) { h = boost::lexical_cast< double >( r[1] ); }
                    break;
                default:
                    COMMA_THROW( comma::exception, "expected resize=<width>,<height>, got: \"" << e[1] << "\"" );
            }
            f.push_back( filter( boost::bind( &resize_impl_, _1, width, height, w, h ) ) );
        }
        else if( e[0] == "max" ) // todo: remove this filter; not thread-safe, should be run with --threads=1
        {
            f.push_back( filter( max_impl_( boost::lexical_cast< unsigned int >( e[1] ), true ), false ) );
        }
        else if( e[0] == "min" ) // todo: remove this filter; not thread-safe, should be run with --threads=1
        {
            f.push_back( filter( max_impl_( boost::lexical_cast< unsigned int >( e[1] ), false ), false ) );
        }
        else if( e[0] == "timestamp" )
        {
            f.push_back( filter( &timestamp_impl_ ) );
        }
        else if( e[0] == "transpose" )
        {
            f.push_back( filter( &transpose_impl_ ) );
        }
        else if( e[0] == "split" )
        {
            f.push_back( filter( &split_impl_ ) );
        }
        else if( e[0] == "merge" )
        {
            unsigned int default_number_of_channels = 3;
            unsigned int nchannels = e.size() == 1 ? default_number_of_channels : boost::lexical_cast< unsigned int >( e[1] );
            if ( nchannels == 0 ) { COMMA_THROW( comma::exception, "expected positive number of channels in merge filter, got " << nchannels ); }
            f.push_back( filter( boost::bind( &merge_impl_, _1, nchannels ) ) );
        }        
        else if( e[0] == "undistort" )
        {
            f.push_back( filter( undistort_impl_( e[1] ) ) );
        }
        else if( e[0] == "invert" )
        {
            f.push_back( filter( &invert_impl_ ) );
        }
        else if( e[0] == "view" )
        {
            unsigned int delay = e.size() == 1 ? default_delay : boost::lexical_cast< unsigned int >( e[1] );
            f.push_back( filter( boost::bind( &view_impl_, _1, name, delay ), false ) );
        }
        else if( e[0] == "thumb" )
        {
            unsigned int cols = 200;
            unsigned int delay = default_delay;
            if( e.size() > 1 )
            {
                std::vector< std::string > v = comma::split( e[1], ',' );
                if( v.size() >= 1 ) { cols = boost::lexical_cast< unsigned int >( v[0] ); }
                if( v.size() >= 2 ) { delay = boost::lexical_cast< unsigned int >( v[1] ); }
            }   
            f.push_back( filter( boost::bind( &thumb_impl_, _1, name, cols, delay ), false ) );
        }
        else if( e[0] == "encode" )
        {
            if( i < v.size()-1 )
            {
                std::string next_filter = comma::split( v[i+1], '=' )[0];
                if( next_filter != "head" ) COMMA_THROW( comma::exception, "cannot have a filter after encode unless next filter is head" ); 
            }
            if( e.size() < 2 ) { COMMA_THROW( comma::exception, "expected encoding type like jpg, ppm, etc" ); }
            std::string s = e[1];
            f.push_back( filter( boost::bind( &encode_impl_, _1, s ) ) );
        }
        else if( e[0] == "grab" )
        {
            if( e.size() < 2 ) { COMMA_THROW( comma::exception, "expected encoding type like jpg, ppm, etc" ); }
            std::string s = e[1];
            f.push_back( filter( boost::bind( &grab_impl_, _1, s ) ) );
        }
        else if( e[0] == "file" )
        {
            if( e.size() < 2 ) { COMMA_THROW( comma::exception, "expected file type like jpg, ppm, etc" ); }
            std::string s = e[1];
            f.push_back( filter( boost::bind( &file_impl_, _1, s ) ) );
        }
        else if( e[0] == "histogram" )
        {
            if( i < v.size() - 1 ) { COMMA_THROW( comma::exception, "expected 'histogram' as the last filter, got \"" << how << "\"" ); }
            f.push_back( filter( boost::bind( &histogram_impl_, _1 ) ) );
            f.push_back( filter( NULL ) ); // quick and dirty
        }
        else if( e[0] == "null" )
        {
            if( i < v.size() - 1 ) { COMMA_THROW( comma::exception, "expected 'null' as the last filter, got \"" << how << "\"" ); }
            if( i == 0 ) { COMMA_THROW( comma::exception, "'null' as the only filter is not supported; use cv-cat > /dev/null, if you need" ); }
            f.push_back( filter( NULL ) );
        }
        else if( e[0] == "brightness" )
        {
            const std::vector< std::string >& s = comma::split( e[1], ',' );
            double scale = boost::lexical_cast< double >( s[0] );
            double offset = s.size() == 1 ? 0.0 : boost::lexical_cast< double >( s[1] );
            f.push_back( filter( boost::bind( &brightness_impl_, _1, scale, offset ) ) );
        }        
        else if( e[0] == "map" )
        {
            if( e.size() < 2 ) { COMMA_THROW( comma::exception, "expected file name with the map, e.g. map=f.csv" ); }
            std::stringstream s; s << e[1]; for( std::size_t i = 2; i < e.size(); ++i ) { s << "=" << e[i]; }
            std::string map_filter_options = s.str();
            std::vector< std::string > items = comma::split( map_filter_options, '&' );
            bool permissive = std::find( items.begin()+1, items.end(), "permissive" ) != items.end();
            f.push_back( filter( map_impl_( map_filter_options, permissive ) ) );
        }
        else if ( e[0] == "head" )
        {
            if( i < v.size()-1 )
            {
                std::string next_filter = comma::split( v[i+1], '=' )[0];
                if( next_filter != "null" ) { COMMA_THROW( comma::exception, "cannot have a filter after encode unless next filter is null" ); }
            }
            if( e.size() < 2 ) { COMMA_THROW( comma::exception, "expected number of frames, e.g. head=1" ); }
            unsigned int n = boost::lexical_cast< unsigned int >( e[1] );
            f.push_back( filter( boost::bind( &head_impl_, _1, n ) ) );
        }
        else
        {
            COMMA_THROW( comma::exception, "expected filter, got \"" << v[i] << "\"" );
        }
        modified = ( v[i] != "view" && v[i] != "thumb" && v[i] != "split" );
    }
    return f;
}

filters::value_type filters::apply( std::vector< filter >& filters, filters::value_type m )
{
    for( std::size_t i = 0; i < filters.size(); m = filters[ i++ ].filter_function( m ) );
    return m;
}

static std::string usage_impl_()
{
    std::ostringstream oss;
    oss << "    cv::Mat image filters usage (';'-separated):" << std::endl;
    oss << "        accumulate=<n>: accumulate the last n images and concatenate them vertically (useful for slit-scan and spectral cameras like pika2)" << std::endl;
    oss << "            example: cat slit-scan.bin | cv-cat \"accumulate=400;view;null\"" << std::endl;
    oss << "        bayer=<mode>: convert from bayer, <mode>=1-4" << std::endl;
    oss << "        brightness=<scale>[,<offset>]: output=(scale*input)+offset; default offset=0" << std::endl;
    oss << "        convert-to,convert_to=<type>[,<scale>[,<offset>]]: convert to given type; should be the same number of channels; see opencv convertTo for details" << std::endl;
    oss << "        count: write frame number on images" << std::endl;
    oss << "        crop=[<x>,<y>],<width>,<height>: crop the portion of the image starting at x,y with size width x height" << std::endl;
    oss << "        crop-tile=<ncols>,<nrows>,<i>,<j>,...[&horizontal]: divide the image into a grid of tiles (ncols-by-nrows), and output an image made of the croped tiles defined by i,j (count from zero)" << std::endl;
    oss << "            <horizontal>: if present, tiles will be stacked horizontally (by default, vertical stacking is used)" << std::endl;
    oss << "            example: \"crop-tile=2,5,1,0,1,4&horizontal\"" << std::endl;
    oss << "            deprecated: old syntax <i>,<j>,<ncols>,<nrows> is used for one tile if i < ncols and j < ncols" << std::endl;
    oss << "        cross[=<x>,<y>]: draw cross-hair at x,y; default: at image center" << std::endl;
    oss << "        encode=<format>: encode images to the specified format. <format>: jpg|ppm|png|tiff..., make sure to use --no-header" << std::endl;
    oss << "        fft[=<options>]: do fft on a floating point image" << std::endl;
    oss << "            options: inverse: do inverse fft" << std::endl;
    oss << "                     real: output real part only" << std::endl;
    oss << "                     magnitude: output magnitude only" << std::endl;
    oss << "            examples: cv-cat --file image.jpg \"split;crop-tile=0,0,1,3;convert-to=f,0.0039;fft;fft=inverse,magnitude;view;null\"" << std::endl;
    oss << "                      cv-cat --file image.jpg \"split;crop-tile=0,0,1,3;convert-to=f,0.0039;fft=magnitude;convert-to=f,40000;view;null\"" << std::endl;
    oss << "        file=<format>: write images to files with timestamp as name in the specified format. <format>: jpg|ppm|png|tiff...; if no timestamp, system time is used" << std::endl;
    oss << "        flip: flip vertically" << std::endl;
    oss << "        flop: flip horizontally" << std::endl;
    oss << "        grab=<format>: write an image to file with timestamp as name in the specified format. <format>: jpg|ppm|png|tiff..., if no timestamp, system time is used" << std::endl;
    oss << "        head=<n>: output <n> frames and exit" << std::endl;
    oss << "        invert: invert image (to negative)" << std::endl;
    oss << "        magnitude: calculate magnitude for a 2-channel image; see cv::magnitude() for details" << std::endl;    
    oss << "        map=<map file>[&<csv options>][&permissive]: map integer values to floating point values read from the map file" << std::endl;
    oss << "             <csv options>: usual csv options for map file, but &-separated (running out of separator characters)" << std::endl;
    oss << "                  fields: key,value; default: value" << std::endl;
    oss << "                  default: read a single column of floating point values (with the row counter starting from zero used as key)" << std::endl;
    oss << "             <permissive>: if present, integer values in the input are simply copied to the output unless they are in the map" << std::endl;
    oss << "                  default: filter fails with an error message if it encounters an integer value which is not in the map" << std::endl;
    oss << "             example: \"map=map.bin&fields=,key,value&binary=2ui,d\"" << std::endl;
    oss << "        merge=<n>: split an image into n horizontal bands of equal height and merge them into an n-channel image (the number of rows must be a multiple of n)" << std::endl;
    oss << "        null: same as linux /dev/null (since windows does not have it)" << std::endl;
    oss << "        resize=<width>,<height>: e.g:" << std::endl;
    oss << "            resize=512,1024 : resize to 512x1024 pixels" << std::endl;
    oss << "            resize=0.2,0.4 : resize to 20% of width and 40% of height" << std::endl;
    oss << "            resize=0.5 : resize proportionally to 50%" << std::endl;
    oss << "            resize=0.5,1024 : 50% of width; heigth 1024 pixels" << std::endl;
    oss << "            note: if no decimal dot '.', size is in pixels; if decimal dot present, size as a fraction" << std::endl;
    oss << "                  i.e. 5 means 5 pixels; 5.0 means 5 times" << std::endl;
    oss << "        split: split n-channel image into a nx1 grey-scale image" << std::endl;
    oss << "        text=<text>[,x,y][,colour]: print text; default x,y: 10,10; default colour: yellow" << std::endl;
    oss << "        thumb[=<cols>[,<wait-interval>]]: view resized image; a convenience for debugging and filter pipeline monitoring" << std::endl;
    oss << "                                          <cols>: image width in pixels; default: 100" << std::endl;
    oss << "                                          <wait-interval>: a hack for now; milliseconds to wait for image display and key press; default: 1" << std::endl;
    oss << "        timestamp: write timestamp on images" << std::endl;
    oss << "        transpose: transpose the image (swap rows and columns)" << std::endl;
    oss << "        undistort=<undistort map file>: undistort" << std::endl;
    oss << "        view[=<wait-interval>]: view image; press <space> to save image (timestamp or system time as filename); <esc>: to close" << std::endl;
    oss << "                                <wait-interval>: a hack for now; milliseconds to wait for image display and key press; default 1" << std::endl;    
    return oss.str();
}

const std::string& filters::usage()
{
    static const std::string s = usage_impl_();
    return s;
}

} } // namespace snark{ namespace cv_mat {
