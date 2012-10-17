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

#include <fstream>
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

static filters::value_type resize_impl_( filters::value_type m, unsigned int width, unsigned int height )
{
    filters::value_type n;
    n.first = m.first;
    cv::resize( m.second, n.second, cv::Size( width, height ) );
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
    n.second = cv::Mat( m.second.rows * 3, m.second.cols, CV_8UC1 );
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
    cv::imshow( name.c_str(), m.second );
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


static filters::value_type file_impl_( filters::value_type m, const std::string& type )
{
    std::string filename = boost::posix_time::to_iso_string( m.first.is_not_a_date_time() ? boost::posix_time::microsec_clock::universal_time() : m.first );
    filename += "." + type;
    cv::imwrite( filename, m.second );
    return m;
}

static filters::value_type timestamp_impl_( filters::value_type m )
{
    cv::rectangle( m.second, cv::Point( 5, 5 ), cv::Point( 228, 25 ), cv::Scalar( 240, 240, 240 ), CV_FILLED, CV_AA );
    cv::putText( m.second, boost::posix_time::to_iso_string( m.first ), cv::Point( 10, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA );
    return m;
}

static filters::value_type empty_impl_( filters::value_type m )
{
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

std::vector< filter > filters::make( const std::string& how )
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
            std::pair< unsigned int, unsigned int > p = comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( e[1] );
            f.push_back( filter( boost::bind( &resize_impl_, _1, p.first, p.second ) ) );
        }
        else if( e[0] == "timestamp" )
        {
            f.push_back( filter( &timestamp_impl_ ) );
        }
        else if( e[0] == "empty" )
        {
            f.push_back( filter( &empty_impl_ ) );
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
        else if( e[0] == "view" )
        {
            unsigned int delay = e.size() == 1 ? 1 : boost::lexical_cast< unsigned int >( e[1] );
            f.push_back( filter( boost::bind( &view_impl_, _1, name, delay ), false ) );
        }
        else if( e[0] == "encode" )
        {
            if( e.size() < 2 ) { COMMA_THROW( comma::exception, "expected encoding type like jpg, ppm, etc" ); }
            std::string s = e[1];
            f.push_back( filter( boost::bind( &encode_impl_, _1, s ) ) );
            last = true;
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
            f.push_back( filter( NULL ) );
        }
        else
        {
            COMMA_THROW( comma::exception, "expected filter, got \"" << v[i] << "\"" );
        }
        modified = ( v[i] != "view" && v[i] != "split" );
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
    oss << "        crop-tile=[<x>,<y>],<num-tile-x>,<num-tile-y>: divide the image in num-tile-x x num-tile-y tiles, and crop the tile x,y" << std::endl;
    oss << "        cross[=<x>,<y>]: draw cross-hair at x,y; default: at image centre" << std::endl;
    oss << "        empty: do nothing; convenient for debugging or data formatting" << std::endl;
    oss << "        flip: flip vertically" << std::endl;
    oss << "        flop: flip horizontally" << std::endl;
    oss << "        text=<text>[,x,y][,colour]: print text; default x,y: 10,10; default colour: yellow" << std::endl;
    oss << "        null: same as linux /dev/null (since windows does not have it)" << std::endl;
    oss << "        resize=<width>,<height>: resize" << std::endl;
    oss << "        timestamp: write timestamp on images" << std::endl;
    oss << "        transpose: transpose the image (swap rows and columns)" << std::endl;
    oss << "        undistort=<map file>: undistort" << std::endl;    
    oss << "        view[=<wait-interval>]: view image; press <space> to save image (timestamp or system time as filename); <esc>: to close" << std::endl;
    oss << "                                <wait-interval>: a hack for now; milliseconds to wait for image display and key press; default 1" << std::endl;
    oss << "        encode=<format>: encode images to the specified format. <format>=jpg|ppm|png|tiff..., make sure to use --no-header" << std::endl;
    oss << "        file=<format>: write images to files with timestamp as name in the specified format. <format>=jpg|ppm|png|tiff...; if no timestamp, system time used" << std::endl;
    return oss.str();
}

const std::string& filters::usage()
{
    static const std::string s = usage_impl_();
    return s;
}

} } // namespace snark{ namespace cv_mat {
