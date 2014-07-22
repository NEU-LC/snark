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


#include <fstream>
#include <queue>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include <comma/string/string.h>
#include "./filters.h"
#include <Eigen/Core>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

namespace snark{ namespace cv_mat {

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

static filters::value_type crop_impl_( filters::value_type m, unsigned int x, unsigned int y, unsigned int w, unsigned int h )
{
    return filters::value_type( m.first, cv::Mat( m.second, cv::Rect( x, y, w, h ) ) );
}

static filters::value_type crop_tile_impl_( filters::value_type m, unsigned int x, unsigned int y, unsigned int w, unsigned int h )
{
    unsigned int tileWidth = m.second.cols / w;
    unsigned int tileHeight = m.second.rows / h;
    unsigned int cropX = x * tileWidth;
    unsigned int cropY = y * tileHeight;
    return filters::value_type( m.first, cv::Mat( m.second, cv::Rect( cropX, cropY, tileWidth, tileHeight ) ) );
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

static filters::value_type split_impl_( filters::value_type m )
{
    filters::value_type n;
    n.first = m.first;
    n.second = cv::Mat( m.second.rows * 3, m.second.cols, CV_8UC1 ); // todo: check number of channels!
    std::vector< cv::Mat > channels;
    channels.reserve( 3 );
    channels.push_back( cv::Mat( n.second, cv::Rect( 0, 0, m.second.cols, m.second.rows ) ) );
    channels.push_back( cv::Mat( n.second, cv::Rect( 0, m.second.rows, m.second.cols, m.second.rows ) ) );
    channels.push_back( cv::Mat( n.second, cv::Rect( 0, 2 * m.second.rows, m.second.cols, m.second.rows ) ) );
    cv::split( m.second, channels );
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

static filters::value_type invert_impl_( filters::value_type m )
{
    unsigned int c = ( m.second.dataend - m.second.datastart ) / ( m.second.rows * m.second.cols );
    if( c != 3 && c != 1 ) { COMMA_THROW( comma::exception, "expected 1 or 3 channels, got: " << c ); } // quick and dirty
    filters::value_type n;
    n.first = m.first;
    m.second.copyTo( n.second );
    for( unsigned char* c = n.second.datastart; c < n.second.dataend; *c = 255 - *c, ++c );
    return n;
}

static filters::value_type text_impl_( filters::value_type m, const std::string& s, const cv::Point& origin, const cv::Scalar& colour )
{
    cv::putText( m.second, s, origin, cv::FONT_HERSHEY_SIMPLEX, 1.0, colour, 1, CV_AA );
    return m;
}

class undistort_impl_
{
    public:
        undistort_impl_( const std::string filename ) : filename_( filename ) {}

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
            ::memset( m.second.datastart, 0, m.second.dataend - m.second.datastart );
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

std::vector< filter > filters::make( const std::string& how, unsigned int default_delay )
{
    std::vector< std::string > v = comma::split( how, ';' );
    std::vector< filter > f;
    if( how == "" ) { return f; }
    std::string name;
    bool modified = false;
    bool last = false;
    for( std::size_t i = 0; i < v.size(); name += ( i > 0 ? ";" : "" ) + v[i], ++i )
    {
        if( last )
        {
            COMMA_THROW( comma::exception, "cannot have a filter after encode" );
        }
        std::vector< std::string > e = comma::split( v[i], '=' );
        if( e[0] == "bayer" )
        {
            if( modified ) { COMMA_THROW( comma::exception, "cannot covert from bayer after transforms: " << name ); }
            unsigned int which = boost::lexical_cast< unsigned int >( e[1] );
            f.push_back( filter( boost::bind( &cvt_color_impl_, _1, which ) ) );
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
            unsigned int x, y, w, h;
            std::vector< std::string > s = comma::split( e[1], ',' );
            x = boost::lexical_cast< unsigned int >( s[0] );
            y = boost::lexical_cast< unsigned int >( s[1] );
            w = boost::lexical_cast< unsigned int >( s[2] );
            h = boost::lexical_cast< unsigned int >( s[3] );
            f.push_back( filter( boost::bind( &crop_tile_impl_, _1, x, y, w, h ) ) );
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
        else if( e[0] == "flip" )
        {
            f.push_back( filter( boost::bind( &flip_impl_, _1, 0 ) ) );
        }
        else if( e[0] == "flop" )
        {
            f.push_back( filter( boost::bind( &flip_impl_, _1, 1 ) ) );
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
            if( e.size() < 2 ) { COMMA_THROW( comma::exception, "expected encoding type like jpg, ppm, etc" ); }
            std::string s = e[1];
            f.push_back( filter( boost::bind( &encode_impl_, _1, s ) ) );
            last = true;
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
        else if( e[0] == "null" )
        {
            if( ( i + 1 ) != v.size() ) { COMMA_THROW( comma::exception, "expected 'null' as the last filter, got \"" << how << "\"" ); }
            if( i == 0 ) { COMMA_THROW( comma::exception, "'null' as the only filter is not supported; use cv-cat > /dev/null, if you need" ); }
            f.push_back( filter( NULL ) );
        }
        else if( e[0] == "brightness" )
        {
            double scale, offset;
            std::vector< std::string > s = comma::split( e[1], ',' );
            scale = boost::lexical_cast< double >( s[0] );
            offset = boost::lexical_cast< double >( s[1] );
            f.push_back( filter( boost::bind( &brightness_impl_, _1, scale, offset ) ) );
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
    oss << "        bayer=<mode>: convert from bayer, <mode>=1-4" << std::endl;
    oss << "        crop=[<x>,<y>],<width>,<height>: crop the portion of the image starting at x,y with size width x height" << std::endl;
    oss << "        crop-tile=[<x>,<y>],<num-tile-x>,<num-tile-y>: divide the image in num-tile-x x num-tile-y tiles, and crop the tile x,y (count from zero)" << std::endl;
    oss << "        cross[=<x>,<y>]: draw cross-hair at x,y; default: at image center" << std::endl;
    oss << "        flip: flip vertically" << std::endl;
    oss << "        flop: flip horizontally" << std::endl;
    oss << "        invert: invert image (to negative)" << std::endl;
    oss << "        brightness=<scale>,<offset>: output=(scale*input)+offset" << std::endl;
    oss << "        split: split r,g,b channels into a 3x1 gray image" << std::endl;
    oss << "        text=<text>[,x,y][,colour]: print text; default x,y: 10,10; default colour: yellow" << std::endl;
    oss << "        null: same as linux /dev/null (since windows does not have it)" << std::endl;
    oss << "        resize=<width>,<height>: e.g:" << std::endl;
    oss << "            resize=512,1024 : resize to 512x1024 pixels" << std::endl;
    oss << "            resize=0.2,0.4 : resize to 20% of width and 40% of height" << std::endl;
    oss << "            resize=0.5 : resize proportionally to 50%" << std::endl;
    oss << "            resize=0.5,1024 : 50% of width; heigth 1024 pixels" << std::endl;
    oss << "            note: if no decimal dot '.', size is in pixels; if decimal dot present, size as a fraction" << std::endl;
    oss << "                  i.e. 5 means 5 pixels; 5.0 means 5 times" << std::endl;
    oss << "        thumb[=<cols>[,<wait-interval>]]: view resized image; a convenience for debugging and filter pipeline monitoring" << std::endl;
    oss << "                                          <cols>: image width in pixels; default: 100" << std::endl;
    oss << "                                          <wait-interval>: a hack for now; milliseconds to wait for image display and key press; default: 1" << std::endl;
    oss << "        timestamp: write timestamp on images" << std::endl;
    oss << "        transpose: transpose the image (swap rows and columns)" << std::endl;
    oss << "        undistort=<map file>: undistort" << std::endl;
    oss << "        view[=<wait-interval>]: view image; press <space> to save image (timestamp or system time as filename); <esc>: to close" << std::endl;
    oss << "                                <wait-interval>: a hack for now; milliseconds to wait for image display and key press; default 1" << std::endl;
    oss << "        encode=<format>: encode images to the specified format. <format>: jpg|ppm|png|tiff..., make sure to use --no-header" << std::endl;
    oss << "        grab=<format>: write an image to file with timestamp as name in the specified format. <format>: jpg|ppm|png|tiff..., if no timestamp, system time is used" << std::endl;
    oss << "        file=<format>: write images to files with timestamp as name in the specified format. <format>: jpg|ppm|png|tiff...; if no timestamp, system time is used" << std::endl;
    return oss.str();
}

const std::string& filters::usage()
{
    static const std::string s = usage_impl_();
    return s;
}

} } // namespace snark{ namespace cv_mat {
