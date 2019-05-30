// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

/// @author vsevolod vlaskine

#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include "unstride.h"

namespace snark { namespace cv_calc { namespace unstride {
    
std::string options()
{
    std::ostringstream oss;
    oss << "        --fit-last; last stride is fit exactly to the image size, i.e. last stride may be irregular" << std::endl;
    oss << "        --how-to-handle-overlaps,--how=<how>; default: last; how to handle overlaps" << std::endl;
    oss << "            last: simply put the next tile on top on the previous one" << std::endl;
    oss << "            linear: linearly interpolate across stride overlaps (todo)" << std::endl;
    oss << "            min: take min pixel value: todo" << std::endl;
    oss << "            max: take max pixel value" << std::endl;
    oss << "            other policies: todo" << std::endl;
    oss << "        --input=[<options>]; input options; run cv-cat --help --verbose for details" << std::endl;
    oss << "        --output=[<options>]; output options; run cv-cat --help --verbose for details" << std::endl;
    oss << "        --output-number-of-strides,--number-of-strides: output number of strides as <x>,<y> to stdout and exit" << std::endl;
    //oss << "        --padding=[<padding>]; padding, 'same' or 'valid' (see e.g. tensorflow for the meaning); default: valid" << std::endl;
    oss << "        --shape,--kernel,--size=<x>,<y>; image size" << std::endl;
    oss << "        --strides=[<x>,<y>]; stride size; default: 1,1" << std::endl;
    oss << "        --unstrided-size,--unstrided=<width>,<height>; original (unstrided) image size" << std::endl;
    return oss.str();
}

namespace overlap {

struct base
{
    virtual ~base() {}
    
    virtual void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y ) = 0;
};
    
struct last: public overlap::base
{
    void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y ) { tile.copyTo( cv::Mat( image, cv::Rect( x, y, tile.cols, tile.rows ) ) ); }
};

struct min: public overlap::base
{
    void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y ) { COMMA_THROW( comma::exception, "todo" ); }
};

struct max: public overlap::base
{
    void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y )
    {
        cv::Mat m( image, cv::Rect( x, y, tile.cols, tile.rows ) );
        cv::max( m, tile, m );
    }
};
    
class linear: public overlap::base
{
    public:
        linear(): x_( 0 ), y_( 0 ), y_prev_( 0 ) {}
        
        void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y )
        {
            if( image.type() != CV_32F ) { std::cerr << "cv-calc: unstride: --how=linear: currently works only on images of type f (CV_32F, " << CV_32F << "); got: " << image.type() << std::endl; exit( 1 ); }
            if( x == 0 ) { y_ = y_prev_; }
            cv::Mat s = y == 0 ? tile : vertical_( image, tile, x, y );
            cv::Mat t = x == 0 ? s : horizontal_( image, s, x, y );
            t.copyTo( cv::Mat( image, cv::Rect( x, y, tile.cols, tile.rows ) ) );
            x_ = x;
            y_prev_ = y;
        }
        
    private:
        unsigned int x_;
        unsigned int y_;
        unsigned int y_prev_;
        cv::Mat horizontal_( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y ) // todo: more reuse between horizontal_() and vertical_()
        {
            cv::Mat t;
            tile.copyTo( t );
            unsigned int overlap = x_ + tile.cols - x;
            tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, tile.rows ), [&]( const tbb::blocked_range< std::size_t >& r )
            {
                for( unsigned int i = r.begin(); i < r.end(); ++i )
                {
                    for( unsigned int j = 0; j < overlap; ++j )
                    {
                        double ratio = double( j ) / overlap;
                        t.at< float >( i, j ) = t.at< float >( i, j ) * ratio + image.at< float >( y + i, x + j ) * ( 1 - ratio );
                    }
                }
            } );
            return t;
        }
        cv::Mat vertical_( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y )
        {
            cv::Mat t;
            tile.copyTo( t );
            unsigned int overlap = y_ + tile.rows - y;
            tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, overlap ), [&]( const tbb::blocked_range< std::size_t >& r )
            {
                for( unsigned int i = r.begin(); i < r.end(); ++i )
                {
                    double ratio = double( i ) / overlap;
                    for( unsigned int j = 0; int( j ) < tile.cols; ++j )
                    {
                        t.at< float >( i, j ) = t.at< float >( i, j ) * ratio + image.at< float >( y + i, x + j ) * ( 1 - ratio );
                    }
                }
            } );
            return t;
        }
};

class most_central: public overlap::base
{
    public:
        most_central()
        {
            // todo: get resulting image size, striding size, step, and fit last
            // todo: make cv::mat: each pixel: coordinates of tile centre to which this pixel belongs
            COMMA_THROW( comma::exception, "todo" );
        }
        
        void append( cv::Mat image, cv::Mat tile, unsigned int x, unsigned int y )
        {
            // todo: get tile centre
            // todo: in parallel for, for each pixel
            //       ? check if pixel is set; if it is, continue
            //       - find corresponding tile centre coordinates
            //       - if given tile centre is same as target tile centre, update pixel
        }
        
    private:
        
};

base* make( const std::string& how )
{
    if( how == "most-central" ) { return new overlap::most_central; }
    if( how == "last" ) { return new overlap::last; }
    if( how == "linear" ) { return new overlap::linear; }
    if( how == "min" ) { return new overlap::min; }
    if( how == "max" ) { return new overlap::max; }
    std::cerr << "cv-calc: unstride: expected policy to handle overlaps, got: '" << how << "' (more policies: todo)" << std::endl;
    exit( 1 );
}

} // namespace overlap {

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options )
{
    boost::scoped_ptr< overlap::base > overlap( overlap::make( options.value< std::string >( "--how-to-handle-overlaps,--how", "last" ) ) );
    bool fit_last = options.exists( "--fit-last" );
    snark::cv_mat::serialization input_serialization( input_options );
    snark::cv_mat::serialization output_serialization( output_options );
    const std::vector< std::string >& unstrided_vector = comma::split( options.value< std::string >( "--unstrided-size,--unstrided" ), ',' );
    if( unstrided_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected --unstrided-size as <width>,<height>, got: \"" << options.value< std::string >( "--unstrided-size,--unstrided" ) << std::endl; return 1; }
    cv::Point2i unstrided( boost::lexical_cast< unsigned int >( unstrided_vector[0] ), boost::lexical_cast< unsigned int >( unstrided_vector[1] ) );
    const std::vector< std::string >& strides_vector = comma::split( options.value< std::string >( "--strides", "1,1" ), ',' );
    if( strides_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected strides as <x>,<y>, got: \"" << options.value< std::string >( "--strides" ) << std::endl; return 1; }
    cv::Point2i strides( boost::lexical_cast< unsigned int >( strides_vector[0] ), boost::lexical_cast< unsigned int >( strides_vector[1] ) );
    const std::vector< std::string >& shape_vector = comma::split( options.value< std::string >( "--shape,--size,--kernel" ), ',' );
    if( shape_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected shape as <x>,<y>, got: \"" << options.value< std::string >( "--shape,--size,--kernel" ) << std::endl; return 1; }
    cv::Point2i shape( boost::lexical_cast< unsigned int >( shape_vector[0] ), boost::lexical_cast< unsigned int >( shape_vector[1] ) );
    unsigned int stride_rows = ( unstrided.y - shape.y ) / strides.y + 1;
    unsigned int stride_cols = ( unstrided.x - shape.x ) / strides.x + 1;
    if( fit_last )
    {
        if( int( stride_rows - 1 ) * strides.y + shape.y < unstrided.y ) { ++stride_rows; }
        if( int( stride_cols - 1 ) * strides.x + shape.x < unstrided.x ) { ++stride_cols; }
    }
    if( options.exists( "--output-number-of-strides,--number-of-strides" ) ) { std::cout << stride_cols << "," << stride_rows << std::endl; exit( 0 ); }
    typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
    pair_t output;
    unsigned int ix = 0;
    unsigned int iy = 0;
    while( std::cin.good() && !std::cin.eof() )
    {
        pair_t p = input_serialization.read< boost::posix_time::ptime >( std::cin );
        if( p.second.empty() ) { return 0; }
        if( output.second.empty() ) { output.second = cv::Mat( unstrided.y, unstrided.x, p.second.type() ); }
        if( output.second.type() != p.second.type() ) { std::cerr << "cv-calc: unstride: expected input image type " << output.second.type() << "; got: " << p.second.type() << std::endl; exit( 1 ); }
        if( p.second.rows != shape.y ) { std::cerr << "cv-calc: unstride: expected input image with " << shape.y << "; got: " << p.second.rows << std::endl; exit( 1 ); }
        if( p.second.cols != shape.x ) { std::cerr << "cv-calc: unstride: expected input image with " << shape.x << "; got: " << p.second.cols << std::endl; exit( 1 ); }
        if( ix == 0 && iy == 0 ) { output.first = p.first; }
        unsigned int x = ix * strides.x;
        unsigned int y = iy * strides.y;
        if( fit_last )
        {
            if( ix + 1 == stride_cols ) { x = unstrided.x - shape.x; }
            if( iy + 1 == stride_rows ) { y = unstrided.y - shape.y; }
        }
        overlap->append( output.second, p.second, x, y );
        ++ix;
        if( ix >= stride_cols )
        {
            ix = 0;
            ++iy;
            if( iy >= stride_rows ) { iy = 0; }
        }
        if( ix == 0 && iy == 0 ) { output_serialization.write_to_stdout( output, true ); output.second.setTo( 0 ); }
    }
    return 0;
}

} } } // namespace snark { namespace cv_calc { namespace unstride {
