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

#include <algorithm>
#include <fstream>
#include <numeric>
#include <queue>
#include <sstream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/static_assert.hpp>
#include <boost/thread.hpp>
#include <boost/type_traits.hpp>
#include <boost/unordered_map.hpp>
#include <boost/assign/list_of.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/csv/options.h>
#include <comma/string/string.h>
#include <comma/name_value/parser.h>
#include <comma/application/verbose.h>
#include <Eigen/Core>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include "../../timing/timestamped.h"
#include "../../timing/traits.h"
#include "filters.h"
#include "detail/ratio.h"
#include "serialization.h"
#include "traits.h"
#include "depth_traits.h"
#include "../vegetation/filters.h"
#include "tbb/parallel_reduce.h"

namespace {

    struct map_input_t
    {
        typedef double value_type;
        typedef comma::int32 key_type;
        key_type key;
        value_type value;
    };

} // anonymous

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

std::string type_as_string( int t ) // to avoid compilation warning
{
    boost::unordered_map< int, std::string >::const_iterator it = types_as_string.find( t );
    return it == types_as_string.end() ? boost::lexical_cast< std::string >( t ) : it->second;
}

static boost::unordered_map< std::string, unsigned int > fill_cvt_color_types_()
{
    boost::unordered_map<std::string, unsigned int> types;
    //note RGB is exactly the same as BGR
    types[ "CV_BGR2GRAY" ] = types[ "BGR,GRAY" ] = types[ "CV_RGB2GRAY" ] = types[ "RGB,GRAY" ] = CV_BGR2GRAY;
    types[ "CV_GRAY2BGR" ] = types[ "GRAY,BGR" ] = types[ "CV_GRAY2RGB" ] = types[ "GRAY,RGB" ] = CV_GRAY2BGR;
    types[ "CV_BGR2XYZ" ] = types[ "BGR,XYZ" ] = types[ "CV_RGB2XYZ" ] = types[ "RGB,XYZ" ] = CV_BGR2XYZ;
    types[ "CV_XYZ2BGR" ] = types[ "XYZ,BGR" ] = types[ "CV_XYZ2RGB" ] = types[ "XYZ,RGB" ] = CV_XYZ2BGR;
    types[ "CV_BGR2HSV" ] = types[ "BGR,HSV" ] = types[ "CV_RGB2HSV" ] = types[ "RGB,HSV" ] = CV_BGR2HSV;
    types[ "CV_HSV2BGR" ] = types[ "HSV,BGR" ] = types[ "CV_HSV2RGB" ] = types[ "HSV,RGB" ] = CV_HSV2BGR;
    types[ "CV_BGR2Lab" ] = types[ "BGR,Lab" ] = types[ "CV_RGB2Lab" ] = types[ "RGB,Lab" ] = CV_BGR2Lab;
    types[ "CV_Lab2BGR" ] = types[ "Lab,BGR" ] = types[ "CV_Lab2RGB" ] = types[ "Lab,RGB" ] = CV_Lab2BGR;
    types[ "CV_BayerBG2BGR" ] = types[ "BayerBG,BGR" ] = types[ "CV_BayerBG2RGB" ] = types[ "BayerBG,RGB" ] = CV_BayerBG2BGR;
    types[ "CV_BayerGB2BGR" ] = types[ "BayerGB,BGR" ] = types[ "CV_BayerGB2RGB" ] = types[ "BayerGB,RGB" ] = CV_BayerGB2BGR;
    types[ "CV_BayerRG2BGR" ] = types[ "BayerRG,BGR" ] = types[ "CV_BayerRG2RGB" ] = types[ "BayerRG,RGB" ] = CV_BayerRG2BGR;
    types[ "CV_BayerGR2BGR" ] = types[ "BayerGR,BGR" ] = types[ "CV_BayerGR2RGB" ] = types[ "BayerGR,RGB" ] = CV_BayerGR2BGR;
    types[ "CV_BayerBG2GRAY" ] = types[ "BayerBG,GRAY" ] = CV_BayerBG2GRAY;
    types[ "CV_BayerGB2GRAY" ] = types[ "BayerGB,GRAY" ] = CV_BayerGB2GRAY;
    types[ "CV_BayerRG2GRAY" ] = types[ "BayerRG,GRAY" ] = CV_BayerRG2GRAY;
    types[ "CV_BayerGR2GRAY" ] = types[ "BayerGR,GRAY" ] = CV_BayerGR2GRAY;
    types[ "CV_BGR2RGB" ] = types[ "BGR,RGB" ] = CV_BGR2RGB;
    types[ "CV_RGB2BGR" ] = types[ "RGB,BGR" ] = CV_RGB2BGR;
    return types;
}

static boost::unordered_map< std::string, unsigned int > cvt_color_types_ = fill_cvt_color_types_();
unsigned int cvt_color_type_from_string( const std::string& t ) // to avoid compilation warning
{
    boost::unordered_map< std::string, unsigned int >::const_iterator it = cvt_color_types_.find( t );
    if (it == cvt_color_types_.end()) { COMMA_THROW(comma::exception, "unknown conversion enum '" << t << "' for convert-color"); }
    return it->second;
}

static filters::value_type cvt_color_impl_( filters::value_type m, unsigned int which )
{
    filters::value_type n;
    n.first = m.first;
    cv::cvtColor( m.second, n.second, which );
    return n;
}

static cv::Scalar scalar_from_strings( const std::string* begin, unsigned int size )
{
    switch( size )
    {
        case 1: return cv::Scalar( boost::lexical_cast< float >( begin[0] ) );
        case 2: return cv::Scalar( boost::lexical_cast< float >( begin[0] ), boost::lexical_cast< float >( begin[1] ) );
        case 3: return cv::Scalar( boost::lexical_cast< float >( begin[0] ), boost::lexical_cast< float >( begin[1] ), boost::lexical_cast< float >( begin[2] ) );
        case 4: return cv::Scalar( boost::lexical_cast< float >( begin[0] ), boost::lexical_cast< float >( begin[1] ), boost::lexical_cast< float >( begin[2] ), boost::lexical_cast< float >( begin[3] ) );
        default: break;
    }
    COMMA_THROW( comma::exception, "expected a scalar of the size up to 4, got: " << size << " elements" );
}

static filters::value_type unpack12_impl_( filters::value_type m )
{
    //if(m.second.channels()!=1) { COMMA_THROW( comma::exception, "expected one channel input, got: ", m.second.channels() );}
    if(m.second.type()!=CV_8UC1 && m.second.type()!=CV_16UC1) { COMMA_THROW( comma::exception, "expected CV_8UC1("<<int(CV_8UC1)<<") or CV_16UC1("<<int(CV_16UC1)<<") , got: "<< m.second.type() );}
    //number of bytes
    int size=m.second.cols;
    if(m.second.type()==CV_16UC1){size*=2;}
    if(size%3!=0) { COMMA_THROW( comma::exception, "size is not divisible by three: "<<size);}
    cv::Mat mat(m.second.rows, (2*size)/3, CV_16UC1);
    for(int j=0;j<m.second.rows;j++)
    {
        const unsigned char* ptr=m.second.ptr(j);
        unsigned short* out_ptr=mat.ptr<unsigned short>(j);
        for(int col=0, i=0;i<size;i+=3)
        {
            out_ptr[col++] = ((unsigned(ptr[i])<<4) +  (unsigned(ptr[i+1]&0xF0) >> 4)) << 4;
            out_ptr[col++] = ((unsigned(ptr[i+2])<<4) + unsigned(ptr[i+1]&0x0F)) << 4;
            //out_ptr[col++] = (unsigned(ptr[i+2])<<4 | unsigned(ptr[i+1]&0x0F) >> 4) << 4;
        }
   }
    return filters::value_type(m.first, mat);
}

static filters::value_type head_impl_( filters::value_type m, unsigned int number_of_frames )
{
    static unsigned int frame_number = 0;
    if( frame_number < number_of_frames ) { frame_number++; return m; } else { return filters::value_type(); }
}

static filters::value_type crop_impl_( filters::value_type m, unsigned int x, unsigned int y, unsigned int w, unsigned int h )
{
    cv::Mat cropped;
    m.second( cv::Rect( x, y, w, h ) ).copyTo( cropped );
    return filters::value_type( m.first, cropped );
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

typedef std::pair< unsigned int, unsigned int > stripe_t;

static unsigned int sum_up( unsigned int v, const stripe_t & s ){ return v + s.second; }

static filters::value_type crop_cols_impl_( filters::value_type input, const std::vector< stripe_t > & cols )
{
    unsigned int h = input.second.rows;
    unsigned int w = std::accumulate( cols.begin(), cols.end(), 0, sum_up );
    filters::value_type output( input.first, cv::Mat( h, w, input.second.type() ) );
    unsigned int offset = 0;
    for( std::size_t i = 0; i < cols.size(); ++i)
    {
        cv::Mat tile( output.second, cv::Rect( offset, 0, cols[i].second, h ) );
        cv::Mat( input.second, cv::Rect( cols[i].first, 0, cols[i].second, h ) ).copyTo( tile );
        offset += cols[i].second;
    }
    return output;
}

static filters::value_type crop_rows_impl_( filters::value_type input, const std::vector< stripe_t > & rows )
{
    unsigned int w = input.second.cols;
    unsigned int h = std::accumulate( rows.begin(), rows.end(), 0, sum_up );
    filters::value_type output( input.first, cv::Mat( h, w, input.second.type() ) );
    unsigned int offset = 0;
    for( std::size_t i = 0; i < rows.size(); ++i)
    {
        cv::Mat tile( output.second, cv::Rect( 0, offset, w, rows[i].second ) );
        cv::Mat( input.second, cv::Rect( 0, rows[i].first, w, rows[i].second ) ).copyTo( tile );
        offset += rows[i].second;
    }
    return output;
}

static const int bands_method_default = CV_REDUCE_AVG;

static filters::value_type bands_to_cols_impl_( filters::value_type input, bool bands_to_cols, const std::vector< stripe_t > & bands, int cv_reduce_method, int cv_reduce_dtype = -1 )
{
    unsigned int w = input.second.cols;
    unsigned int h = input.second.rows;
    static std::set< int > good_input_depths = boost::assign::list_of( CV_8U )( CV_16U )( CV_16S )( CV_32F )( CV_64F );
    if ( good_input_depths.find( input.second.depth() ) == good_input_depths.end() )
    {
        COMMA_THROW( comma::exception, "depth of the " << type_as_string( input.second.type() ) << " image type is not supported by cv::reduce; consider 'convert-to' before processing" );
    }
    unsigned int output_type = cv_reduce_dtype >= 0
                             ? CV_MAKETYPE( CV_MAT_DEPTH( cv_reduce_dtype ), input.second.channels() )
                             : input.second.type() ;
    filters::value_type output( input.first, cv::Mat( bands_to_cols ? h : bands.size()
                                                    , bands_to_cols ? bands.size() : w
                                                    , output_type ) );
    for( std::size_t i = 0; i < bands.size(); ++i )
    {
        cv::Mat intile( input.second, bands_to_cols
                                    ? cv::Rect( bands[i].first, 0, bands[i].second, h )
                                    : cv::Rect( 0, bands[i].first, w, bands[i].second ) );
        cv::Mat outtile( output.second, bands_to_cols
                                      ? cv::Rect( i, 0, 1, h )
                                      : cv::Rect( 0, i, w, 1 ) );
        int cv_reduce_dim = bands_to_cols ? 1 : 0 ;
        cv::reduce( intile, outtile, cv_reduce_dim, cv_reduce_method, output_type );
    }
    return output;
}

static filters::value_type cols_to_channels_impl_( filters::value_type input, bool cols_to_channels, const std::vector< unsigned int > & values, double padding_value, unsigned int repeat )
{
    if ( input.second.channels() != 1 ) { COMMA_THROW( comma::exception, "input image for cols-to-channels operation must have a single channel" ); }
    std::string element = cols_to_channels ? "column" : "row";

    unsigned int w = input.second.cols;
    unsigned int h = input.second.rows;

    unsigned int min_element = *std::min_element( values.begin(), values.end() );
    if ( min_element >= ( cols_to_channels ? w : h ) ) { COMMA_THROW( comma::exception, "the first output " << element << " is outside of the input image" ); }
    unsigned int output_w = cols_to_channels
                          ? ( repeat ? ( w - min_element ) / repeat : 1 )
                          : w ;
    unsigned int output_h = cols_to_channels
                          ? h
                          : ( repeat ? ( h - min_element ) / repeat : 1 ) ;
    unsigned int output_c = values.size() == 2 ? 3 : values.size();
    unsigned int output_t = CV_MAKETYPE( input.second.depth(), output_c );
    filters::value_type output( input.first, cv::Mat( output_h, output_w, output_t ) );

    std::vector< int > from_to;
    from_to.reserve( 2 * output_c );
    for ( size_t i = 0; i < output_c; ++i ) { from_to.push_back( i ); from_to.push_back( i ); }

    std::vector< cv::Mat > src( output_c );
    std::vector< cv::Mat > dst( 1 );
    cv::Mat padding = ( cols_to_channels
                      ? cv::Mat::ones( h, 1, input.second.type() )
                      : cv::Mat::ones( 1, w, input.second.type() ) ) * padding_value; // instantiation is lazy
    std::vector< unsigned int > indices( values );
    for( size_t i = indices.size(); i < output_c; ++i ) { src[i] = padding; }
    for ( unsigned int opos = 0; opos < ( cols_to_channels ? output_w : output_h ) ; ++opos )
    {
        for( size_t i = 0; i < indices.size(); ++i )
        {
            unsigned int j = indices[i];
            if ( j >= ( cols_to_channels ? w : h ) ) {
                src[i] = padding;
            } else {
                src[i] = cols_to_channels
                       ? cv::Mat( input.second, cv::Rect( j, 0, 1, h ) )
                       : cv::Mat( input.second, cv::Rect( 0, j, w, 1 ) ) ;
            }
            indices[i] += repeat;
        }
        dst[0] = cols_to_channels
               ? cv::Mat( output.second, cv::Rect( opos, 0, 1, h ) )
               : cv::Mat( output.second, cv::Rect( 0, opos, w, 1 ) ) ;
        cv::mixChannels( src, dst, &from_to[0], output_c );
    }
    return output;
}

static filters::value_type channels_to_cols_impl_( filters::value_type input, bool channels_to_cols )
{
    unsigned int w = input.second.cols;
    unsigned int h = input.second.rows;

    unsigned int input_c = input.second.channels();
    unsigned int output_t = CV_MAKETYPE( input.second.depth(), 1 );
    filters::value_type output( input.first, cv::Mat( channels_to_cols ? h : input_c * h
                                                    , channels_to_cols ? input_c * w : w
                                                    , output_t ) );

    std::vector< int > from_to;
    from_to.reserve( 2 * input_c );
    for ( size_t i = 0; i < input_c; ++i ) { from_to.push_back( i ); from_to.push_back( i ); }

    std::vector< cv::Mat > src( 1 );
    std::vector< cv::Mat > dst( input_c );
    for ( unsigned int ipos = 0; ipos < ( channels_to_cols ? w : h ) ; ++ipos )
    {
        for( size_t i = 0; i < input_c; ++i )
        {
            dst[i] = cv::Mat( output.second, channels_to_cols
                                           ? cv::Rect( ipos * input_c + i, 0, 1, h )
                                           : cv::Rect( 0, ipos * input_c + i, w, 1 ) );
        }
        src[0] = cv::Mat( input.second, channels_to_cols
                                      ? cv::Rect( ipos, 0, 1, h )
                                      : cv::Rect( 0, ipos, w, 1 ) );
        cv::mixChannels( src, dst, &from_to[0], input_c );
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

static filters::value_type resize_impl_( filters::value_type m, unsigned int width, unsigned int height, double w, double h, int interpolation )
{
    filters::value_type n;
    n.first = m.first;
    cv::resize( m.second, n.second, cv::Size( width ? width : m.second.cols * w, height ? height : m.second.rows * h ), 0, 0, interpolation );
    return n;
}

static filters::value_type brightness_impl_( filters::value_type m, double scale, double offset )
{
    filters::value_type n;
    n.first = m.first;
    n.second = (m.second * scale) + offset;
    return n;
}

static filters::value_type colour_map_impl_( filters::value_type m, int type )
{
    filters::value_type n;
    n.first = m.first;
    cv::applyColorMap( m.second, n.second, type );
    return n;
}

static filters::value_type mask_impl_( filters::value_type m, boost::function< filters::value_type( filters::value_type ) > mask ) // have to pass mask by value, since filter functors may change on call
{
    filters::value_type n;
    n.first = m.first;
    const cv::Mat & f = mask( m ).second;
    if ( f.depth() != CV_8U ) { COMMA_THROW( comma::exception, "the mask type is " << type_as_string( f.type() ) << ", must have CV_8U depth; use convert-to explicitly" ); }
    m.second.copyTo( n.second, f );
    return n;
}

struct blur_t
{
    enum types { box, gaussian, median, bilateral, adaptive_bilateral };
    types blur_type;
    cv::Size kernel_size;
    cv::Point2d std;
    int neighbourhood_size;
    double sigma_colour;
    double sigma_space;

    static types from_string( const std::string& s )
    {
        if( s == "box" ) { return box; }
        if( s == "gaussian" ) { return gaussian; }
        if( s == "median") { return median; }
        if( s == "bilateral") { return bilateral; }
        if( s == "adaptive-bilateral") { return adaptive_bilateral; }
        COMMA_THROW( comma::exception, "expected blur type; got: \"" << s << "\"" );
    }
    blur_t() : neighbourhood_size(0), sigma_colour(0), sigma_space(0) {}
};

static filters::value_type blur_impl_( filters::value_type m, blur_t params )
{
    filters::value_type n;
    n.first = m.first;
    switch( params.blur_type )
    {
        case blur_t::box:
            cv::blur(m.second, n.second, params.kernel_size);
            break;
        case blur_t::gaussian:
            cv::GaussianBlur(m.second, n.second, params.kernel_size, params.std.x, params.std.y);
            break;
        case blur_t::median:
            cv::medianBlur(m.second, n.second, params.neighbourhood_size);
            break;
        case blur_t::bilateral:
            cv::bilateralFilter(m.second, n.second, params.neighbourhood_size, params.sigma_colour, params.sigma_space);
            break;
        case blur_t::adaptive_bilateral:
            cv::adaptiveBilateralFilter(m.second, n.second, params.kernel_size, params.sigma_colour, params.sigma_space);
            break;
    }
    return n;
}

struct threshold_t
{
    enum types { binary = CV_THRESH_BINARY
               , binary_inv = CV_THRESH_BINARY_INV
               , trunc = CV_THRESH_TRUNC
               //, trunc_inv = CV_THRESH_TRUNC | CV_THRESH_BINARY_INV // == CV_THRESH_TOZERO
               , tozero = CV_THRESH_TOZERO
               , tozero_inv = CV_THRESH_TOZERO_INV };

    static types from_string( const std::string& s )
    {
        if( s.empty() || s == "binary" ) { return binary; }
        if( s == "binary_inv" ) { return binary_inv; }
        if( s == "trunc" ) { return trunc; }
        if( s == "tozero" ) { return tozero; }
        if( s == "tozero_inv" ) { return tozero_inv; }
        COMMA_THROW( comma::exception, "expected threshold type, got: \"" << s << "\"" );
    }
};

static filters::value_type threshold_impl_( filters::value_type m, double threshold, double max_value, threshold_t::types type, bool otsu )
{
    filters::value_type n;
    n.first = m.first;
    cv::threshold( m.second, n.second, threshold, max_value, otsu ? type | CV_THRESH_OTSU : type );
    return n;
}

static filters::value_type transpose_impl_( filters::value_type m )
{
    filters::value_type n;
    n.first = m.first;
    cv::transpose( m.second, n.second );
    return n;
}

int single_channel_type( int t )
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
    n.second = cv::Mat( m.second.rows * m.second.channels(), m.second.cols, single_channel_type( m.second.type() ) ); // todo: check number of channels!
    std::vector< cv::Mat > channels;
    channels.reserve( m.second.channels() );
    for( unsigned int i = 0; i < static_cast< unsigned int >( m.second.channels() ); ++i )
    {
        channels.push_back( cv::Mat( n.second, cv::Rect( 0, i * m.second.rows, m.second.cols, m.second.rows ) ) );
    }
    cv::split( m.second, channels );
    return n;
}

class log_impl_ // quick and dirty; poor-man smart pointer, since boost::mutex is non-copyable
{            
    public:
        log_impl_() {}

        log_impl_( const std::string& filename ) : logger_( new logger( filename ) ) {}

        log_impl_( const std::string& directory, boost::posix_time::time_duration period, bool index ) : logger_( new logger( directory, period, index ) ) {}

        log_impl_( const std::string& directory, unsigned int size, bool index ) : logger_( new logger( directory, size, index ) ) {}

        filters::value_type operator()( filters::value_type m ) { return logger_->operator()( m ); }
        
        class logger
        {
            public:
                logger() : size_( 0 ), count_( 0 ) {}

                logger( const std::string& filename ) : ofstream_( new std::ofstream( &filename[0] ) ), serialization_( "t,rows,cols,type", comma::csv::format( "t,3ui" ) ), size_( 0 ), count_( 0 ) 
                { 
                    if( !ofstream_->is_open() ) { COMMA_THROW( comma::exception, "failed to open \"" << filename << "\"" ); } 
                }

                logger( const std::string& directory, boost::posix_time::time_duration period, bool index ) : directory_( directory ), serialization_( "t,rows,cols,type", comma::csv::format( "t,3ui" ) ), period_( period ), size_( 0 ), count_( 0 ), index_(index, directory) { }

                logger( const std::string& directory, unsigned int size, bool index ) : directory_( directory ), serialization_( "t,rows,cols,type", comma::csv::format( "t,3ui" ) ), size_( size ), count_( 0 ), index_(index, directory) { }

                ~logger() { if( ofstream_ ) { ofstream_->close(); } }

                filters::value_type operator()( filters::value_type m )
                {
                    if( m.second.empty() ) { return m; } // quick and dirty, end of stream
                    boost::mutex::scoped_lock lock( mutex_ ); // somehow, serial_in_order still may have more than one instance of filter run at a time
                    update_on_size_();
                    update_on_time_( m );
                    if( !ofstream_ )
                    {
                        std::string filename = directory_ + '/' + boost::posix_time::to_iso_string( m.first ) + ".bin";
                        ofstream_.reset( new std::ofstream( &filename[0] ) );
                        if( !ofstream_->is_open() ) { COMMA_THROW( comma::exception, "failed to open \"" << filename << "\"" ); }
                    }
                    serialization_.write( *ofstream_, m );
                    index_.write( m, serialization_.size( m ) );
                    return m;
                }
                
                struct indexer
                {
                    boost::posix_time::ptime t;
                    comma::uint16 file;
                    comma::uint64 offset;
                    boost::scoped_ptr< std::ofstream > filestream;
                    boost::scoped_ptr< comma::csv::binary_output_stream< indexer > > csv_stream;
                    
                    indexer() : file( 0 ), offset( 0 ) {}
                    
                    indexer( bool enabled, const std::string& directory ) : file ( 0 ), offset( 0 )
                    {
                        if( !enabled ) { return; }
                        std::string index_file = directory + "/index.bin";
                        filestream.reset( new std::ofstream( &index_file[0] ) );
                        if( !filestream->is_open() ) { COMMA_THROW( comma::exception, "failed to open \"" << index_file << "\"" ); }
                        csv_stream.reset( new comma::csv::binary_output_stream< indexer >( *filestream ) );
                    }
                    
                    ~indexer() { if ( filestream ) { filestream->close(); } }
                    
                    void increment_file() { ++file; offset = 0; }
                    
                    void write( const filters::value_type& m, std::size_t size )
                    {
                        t = m.first;
                        if( csv_stream ) 
                        { 
                            csv_stream->write( *this );
                            csv_stream->flush(); 
                        }
                        offset += size;
                    }
                };
                
            private:
                boost::mutex mutex_;
                std::string directory_;
                boost::scoped_ptr< std::ofstream > ofstream_;
                snark::cv_mat::serialization serialization_;
                boost::optional< boost::posix_time::time_duration > period_;
                boost::posix_time::ptime start_;
                unsigned int size_;
                unsigned int count_;
                indexer index_;

                void update_on_size_()
                {
                    if( size_ == 0 ) { return; }
                    if( count_ < size_ ) { ++count_; return; }
                    count_ = 1;
                    ofstream_->close();
                    ofstream_.reset();
                    index_.increment_file();
                }

                void update_on_time_( filters::value_type m )
                {
                    if( !period_ ) { return; }
                    if( start_.is_not_a_date_time() ) { start_ = m.first; return; }
                    if( ( m.first - start_ ) < *period_ ) { return; }
                    start_ = m.first;
                    ofstream_->close();
                    ofstream_.reset();
                    index_.increment_file();
                }
        };
        
    private:
        boost::shared_ptr< logger > logger_; // todo: watch performance
};

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

static filters::value_type cross_impl_( filters::value_type m, boost::optional< Eigen::Vector2i > xy ) // todo: move to draw
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

namespace drawing {

struct shape
{
    cv::Scalar color;
    int thickness;
    int line_type;
    int shift;
    shape() : thickness( 1 ), line_type( 8 ), shift( 0 ) {}
    shape( const cv::Scalar& color, int thickness = 1, int line_type = 8, int shift = 0 ) : color( color ), thickness( thickness ), line_type( line_type ), shift( shift ) {}
};

struct circle : public shape
{
    cv::Point center;
    int radius;
    circle() {}
    circle( const cv::Point& center, int radius, const cv::Scalar& color, int thickness = 1, int line_type = 8, int shift = 0 ) : shape( color, thickness, line_type, shift ), center( center ), radius( radius ) {}
    void draw( cv::Mat m ) const { cv::circle( m, center, radius, color, thickness, line_type, shift ); }
};

struct rectangle : public shape
{
    cv::Point upper_left;
    cv::Point lower_right;
    rectangle() {};
    rectangle( const cv::Point& upper_left, const cv::Point& lower_right, const cv::Scalar& color, int thickness = 1, int line_type = 8, int shift = 0 ) : shape( color, thickness, line_type, shift ), upper_left( upper_left ), lower_right( lower_right ) {}
    void draw( cv::Mat m ) const { cv::rectangle( m, upper_left, lower_right, color, thickness, line_type, shift ); }
};

} // namespace drawing {

static filters::value_type circle_impl_( filters::value_type m, const drawing::circle& circle ) { circle.draw( m.second ); return m; }

static filters::value_type rectangle_impl_( filters::value_type m, const drawing::rectangle& rectangle ) { rectangle.draw( m.second ); return m; }

static void encode_impl_check_type( const filters::value_type& m, const std::string& type )
{
    int channels = m.second.channels();
    int size = m.second.elemSize() / channels;
    int cv_type = m.second.type();
    if( !( channels == 1 || channels == 3 ) ) { COMMA_THROW( comma::exception, "expected image with 1 or 3 channel, got " << channels << " channels" ); }
    if( !( size == 1 || size == 2 ) ) { COMMA_THROW( comma::exception, "expected 8- or 16-bit image, got " << size*8 << "-bit image" ); }
    if( size == 2 && !( cv_type == CV_16UC1 || cv_type == CV_16UC3 ) ) {  COMMA_THROW( comma::exception, "expected 16-bit image with unsigned elements, got image of type " << type_as_string( cv_type ) ); }
    if( size == 2 && !( type == "tiff" || type == "tif" || type == "png" || type == "jp2" ) ) { COMMA_THROW( comma::exception, "cannot convert 16-bit image to type " << type << "; use tif or png instead" ); }
}

static filters::value_type encode_impl_( filters::value_type m, const std::string& type )
{
    if( is_empty( m ) ) { return m; }
    encode_impl_check_type( m, type );
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
    if( single_channel_type( m.second.type() ) != CV_8UC1 ) { std::cerr << "cv-cat: histogram: expected an unsigned char image type; got " << type_as_string( m.second.type() ) << std::endl; exit( 1 ); }
    typedef boost::array< comma::uint32, 256 > channel_t;
    std::vector< channel_t > channels( m.second.channels() );
    for( unsigned int i = 0; i < channels.size(); ++i ) { ::memset( ( char* )( &channels[i][0] ), 0, sizeof( comma::uint32 ) * 256 ); }
    for( int r = 0; r < m.second.rows; ++r )
    {
        const unsigned char* p = m.second.ptr< unsigned char >( r );
        for( int c = 0; c < m.second.cols; ++c ) { for( unsigned int i = 0; i < channels.size(); ++channels[i][*p], ++i, ++p ); }
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

template < typename T > static comma::csv::options make_csv_options_( bool binary ) // quick and dirty
{
    comma::csv::options csv;
    if( binary ) { csv.format( comma::csv::format::value< snark::timestamped< T > >() ); }
    return csv;
}

template < typename T > static T cv_read_( const std::string& filename = "", const std::string& path = "" )
{
    if( filename.empty() ) { return T(); }
    T t;
    if( !boost::filesystem::is_regular_file( filename ) ) { COMMA_THROW( comma::exception, "file not found: \"" << filename << "\"" ); }
    const std::vector< std::string > v = comma::split( filename, '.' );
    cv::FileStorage f( filename, cv::FileStorage::READ | ( v.size() > 1 || v.back() == "xml" ? cv::FileStorage::FORMAT_XML : cv::FileStorage::FORMAT_YAML ) );
    cv::FileNode n = f[path];
    t.read( n );
    return t;
}

static filters::value_type simple_blob_impl_( filters::value_type m, const cv::SimpleBlobDetector::Params& params, bool binary )
{
    static cv::SimpleBlobDetector detector( params ); // quick and dirty
    std::vector< cv::KeyPoint > key_points;
    detector.detect( m.second, key_points );
    static comma::csv::output_stream< snark::timestamped< cv::KeyPoint > > os( std::cout, make_csv_options_< snark::timestamped< cv::KeyPoint > >( binary ) );
    for( unsigned int i = 0; i < key_points.size(); ++i ) { os.write( snark::timestamped< cv::KeyPoint >( m.first, key_points[i] ) ); }
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
    encode_impl_check_type( m, type );
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

static filters::value_type invert_brightness_impl_( filters::value_type m )
{
    if( m.second.type() != CV_8UC3 ) { COMMA_THROW( comma::exception, "expected image type 3ub; got: " << type_as_string( m.second.type() ) ); }
    cv::Mat n;
    cv::cvtColor( m.second, n, CV_RGB2HSV );
    for( unsigned char* c = n.datastart + 2; c < n.dataend; *c = 255 - *c, c += 3 );
    cv::cvtColor( n, m.second, CV_HSV2RGB );
    return m;
}

static filters::value_type equalize_histogram_impl_(filters::value_type m)
{
    if( single_channel_type(m.second.type()) != CV_8UC1 ) { COMMA_THROW( comma::exception, "expected image type ub, 2ub, 3ub, 4ub; got: " << type_as_string( m.second.type() ) ); }
    int chs=m.second.channels();
    //split
    std::vector<cv::Mat> planes;
    for(int i=0;i<chs;i++)
        planes.push_back(cv::Mat(1,1,single_channel_type(m.second.type())));
    cv::split(m.second,planes);
    //equalize
    for(int i=0;i<chs;i++)
    {
        //cv::equalizeHist only supports 8-bit single channel
        cv::equalizeHist(planes[i],planes[i]);
    }
    //merge
    cv::merge(planes,m.second);
    return m;
}
static filters::value_type normalize_cv_impl_( filters::value_type m )
{
    //std::cerr<<"my type: "<<m.second.type()<<" ,CV_8UC3: "<<int(CV_8UC3)<<" ,CV_32FC3: "<<int(CV_32FC3)<<" ,CV_8UC1: "<<CV_8UC1<<std::endl;
    cv::normalize(m.second,m.second,1,0,cv::NORM_INF,CV_32FC(m.second.channels()));
    return m;
}

static cv::Scalar max_scalar(const cv::Scalar& s, const cv::Scalar& t)
{
    cv::Scalar out;
    for(int i=0;i<4;i++)
        out[i]=std::max(s[i],t[i]);
    return out;
}

struct filter_table_t
{
    struct filter_i
    {
        virtual cv::Mat normalize_max(const cv::Mat& v)=0;
        virtual cv::Mat normalize_sum(const cv::Mat& v)=0;
        virtual ~filter_i(){}
    };
    std::vector<filter_i*> filters_;
    filter_i& filter(int depth){return *filters_[depth];}
    static const int number_of_partitions=8;
    //typed filter
    template<int Depth>
    struct filter_t:public filter_i
    {
        typedef typename depth_traits< Depth >::value_t value_t;
        cv::Mat normalize_max(const cv::Mat& mat)
        {
            bool use_double=(single_channel_type(mat.type()) == CV_64FC1);
            int partition_size=mat.rows/number_of_partitions;
            //max
            cv::Scalar s(0,0,0,0);
            s=tbb::parallel_reduce(tbb::blocked_range<int>(0,mat.rows, partition_size), s, boost::bind(max_,_1,mat,_2), max_scalar);
            //invert
            s=cv::Scalar(1/(s[0]?s[0]:1),1/(s[1]?s[1]:1),1/(s[2]?s[2]:1),1/(s[3]?s[3]:1));
            //scale
            cv::Mat result(mat.rows,mat.cols,use_double?CV_64FC(mat.channels()):CV_32FC(mat.channels()));
            if(use_double)
                tbb::parallel_for(tbb::blocked_range<int>(0,mat.rows, partition_size), boost::bind(scale_<CV_64F>, _1, mat, s, boost::ref(result)));
            else
                tbb::parallel_for(tbb::blocked_range<int>(0,mat.rows, partition_size), boost::bind(scale_<CV_32F>, _1, mat, s, boost::ref(result)));
            return result;
        }
        cv::Mat normalize_sum(const cv::Mat& mat)
        {
            bool use_double=(single_channel_type(mat.type()) == CV_64FC1);
            cv::Mat result(mat.rows,mat.cols, use_double?CV_64FC(mat.channels()):CV_32FC(mat.channels()));
            int partition_size=mat.rows/number_of_partitions;
            if(use_double)
                tbb::parallel_for(tbb::blocked_range<int>(0,mat.rows, partition_size), boost::bind(normalize_sum_<CV_64F>, _1, mat, boost::ref(result)));
            else
                tbb::parallel_for(tbb::blocked_range<int>(0,mat.rows, partition_size), boost::bind(normalize_sum_<CV_32F>, _1, mat, boost::ref(result)));
            return result;
        }
        static cv::Scalar max_(const tbb::blocked_range<int>& r, const cv::Mat& v, cv::Scalar s)
        {
            int chs=v.channels();
            for(int i=r.begin();i<r.end();i++)
            {
                const value_t* row=v.ptr<value_t>(i);
                for(int j=0;j<v.cols;j++)
                    for(int k=0;k<chs;k++)
                        s[k]=std::max(s[k],double(*row++));
            }
            return s;
        }
        // Output_depth: CV_32FC or CV_64FC for float or double precision
        template<int Output_depth>
        static void scale_(const tbb::blocked_range<int>& r, const cv::Mat& v, const cv::Scalar& s, cv::Mat& out)
        {
            typedef typename depth_traits<Output_depth>::value_t output_t;
            int chs=v.channels();
            if(out.cols!=v.cols || out.rows!=v.rows || out.channels()!=chs)
            {
                COMMA_THROW(comma::exception, "scale: in and out Matrix mismatch");
            }
            if(single_channel_type(out.type())!=CV_MAKETYPE(Output_depth,1))
                COMMA_THROW(comma::exception, "scale: invalid output type: "<<type_as_string(out.type()) << " expected :"<<type_as_string(CV_MAKETYPE(Output_depth,1)));
            for(int i=r.begin();i<r.end();i++)
            {
                const value_t* in=v.ptr<value_t>(i);
                output_t* outp=out.ptr<output_t>(i);
                for(int j=0;j<v.cols;j++)
                    for(int k=0;k<chs;k++)
                        *outp++=output_t(*in++)*s[k];
            }
        }
        // Output_depth: CV_32FC or CV_64FC for float or double precision
        template<int Output_depth>
        static void normalize_sum_(const tbb::blocked_range<int>& r, const cv::Mat& v, cv::Mat& out)
        {
            typedef typename depth_traits<Output_depth>::value_t output_t;
            int chs=v.channels();
            if(out.cols!=v.cols || out.rows!=v.rows || out.channels()!=chs)
            {
                COMMA_THROW(comma::exception, "normalize_sum: in and out Matrix mismatch");
            }
            if(single_channel_type(out.type())!=CV_MAKETYPE(Output_depth,1))
                COMMA_THROW(comma::exception, "normalize_sum: invalid output type: "<<type_as_string(out.type()) << " expected :"<<type_as_string(CV_MAKETYPE(Output_depth,1)));
            for(int i=r.begin();i<r.end();i++)
            {
                const value_t* in=v.ptr<value_t>(i);
                output_t* outp=out.ptr<output_t>(i);
                for(int j=0;j<v.cols;j++)
                {
                    output_t sum=0;
                    for(int k=0;k<chs;k++)
                        sum+=output_t(in[k]);
                    for(int k=0;k<chs;k++)
                        *outp++=output_t(*in++)/(sum?sum:1);
                }
            }
        }
    };
    filter_table_t() : filters_(7)
    {
        filters_[CV_8U]=new filter_t<CV_8U>();
        filters_[CV_8S]=new filter_t<CV_8S>();
        filters_[CV_16U]=new filter_t<CV_16U>();
        filters_[CV_16S]=new filter_t<CV_16S>();
        filters_[CV_32S]=new filter_t<CV_32S>();
        filters_[CV_32F]=new filter_t<CV_32F>();
        filters_[CV_64F]=new filter_t<CV_64F>();
    }
    ~filter_table_t() { for(int i=0;i<=CV_64F;i++) { delete filters_[i]; } }
};
static filter_table_t filter_table;

static filters::value_type normalize_max_impl_( filters::value_type m )
{
    filter_table_t::filter_i& filter=filter_table.filter(m.second.depth());
    return filters::value_type(m.first, filter.normalize_max(m.second));
}
static filters::value_type normalize_sum_impl_( filters::value_type m )
{
    filter_table_t::filter_i& filter=filter_table.filter(m.second.depth());
    return filters::value_type(m.first, filter.normalize_sum(m.second));
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
            std::ifstream stream( &filename_[0] );
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

template< int DepthIn, int DepthOut >
static void ratio( const tbb::blocked_range< std::size_t >& r, const cv::Mat& m, const std::vector< double >& numerator, const std::vector< double >& denominator, cv::Mat& result )
{
    typedef typename depth_traits< DepthIn >::value_t value_in_t;
    typedef typename depth_traits< DepthOut >::value_t value_out_t;
    const unsigned int channels = m.channels();
    const unsigned int cols = m.cols * channels;
    static const value_out_t highest = std::numeric_limits< value_out_t >::max();
    static const value_out_t lowest = std::numeric_limits< value_out_t >::is_integer ? std::numeric_limits< value_out_t >::min() : -highest;
    for( unsigned int i = r.begin(); i < r.end(); ++i )
    {
        const value_in_t* in = m.ptr< value_in_t >(i);
        value_out_t* out = result.ptr< value_out_t >(i);
        for( unsigned int j = 0; j < cols; j += channels )
        {
            double n = numerator[0];
            double d = denominator[0];
            for( unsigned int k = 0; k < channels; ++k ) {
                n += *in * numerator[k + 1];
                d += *in++ * denominator[k + 1];
            }
            double value = ( d == 0 ? ( n == 0 ? 0 : highest ) : n / d );
            *out++ = value > highest ? highest : value < lowest ? lowest : value;
        }
    }
}

template< int DepthIn, int DepthOut >
static filters::value_type per_element_ratio( const filters::value_type m, const std::vector< double >& numerator, const std::vector< double > & denominator )
{
    cv::Mat result( m.second.size(), DepthOut );
    tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, m.second.rows ), boost::bind( &ratio< DepthIn, DepthOut >, _1, m.second, numerator, denominator, boost::ref( result ) ) );
    return filters::value_type( m.first, result );
}

template< int DepthIn >
static filters::value_type per_element_ratio_selector( const filters::value_type m, const std::vector< double >& numerator, const std::vector< double > & denominator, int otype, const std::string & opname )
{
    switch( otype )
    {
        case CV_8U : return per_element_ratio< DepthIn, CV_8U  >( m, numerator, denominator );
        case CV_8S : return per_element_ratio< DepthIn, CV_8S  >( m, numerator, denominator );
        case CV_16U: return per_element_ratio< DepthIn, CV_16U >( m, numerator, denominator );
        case CV_16S: return per_element_ratio< DepthIn, CV_16S >( m, numerator, denominator );
        case CV_32S: return per_element_ratio< DepthIn, CV_32S >( m, numerator, denominator );
        case CV_32F: return per_element_ratio< DepthIn, CV_32F >( m, numerator, denominator );
        case CV_64F: return per_element_ratio< DepthIn, CV_64F >( m, numerator, denominator );
    }
    COMMA_THROW( comma::exception, opname << ": unrecognised output image type " << otype );
}

static filters::value_type ratio_impl_( const filters::value_type m, const std::vector< double >& numerator, const std::vector< double >& denominator, const std::string & opname )
{
    if( numerator.size() != denominator.size() )
        { COMMA_THROW( comma::exception, opname << ": the number of numerator " << numerator.size() << " and denominator " << denominator.size() << " coefficients differs" ); }
    // the coefficients are always constant,r,g,b,a (some of the values can be zero); it is ok to have fewer channels than coefficients as long as all the unused coefficients are zero
    for ( size_t n = static_cast< size_t >( m.second.channels() ) + 1 ; n < numerator.size(); ++n ) {
        if ( numerator[n] != 0.0 || denominator[n] != 0.0 ) {
            const std::string & what = numerator[n] != 0.0 ? "numerator" : "denominator";
            COMMA_THROW( comma::exception, opname << ": have " << m.second.channels() << " channel(s) only, requested non-zero " << what << " coefficient for channel " << n - 1 );
        }
    }
    int otype = single_channel_type( m.second.type() );
    if ( otype != CV_64FC1 ) { otype = CV_32FC1; }
    switch( m.second.depth() )
    {
        case CV_8U : return per_element_ratio_selector< CV_8U  >( m, numerator, denominator, otype, opname );
        case CV_8S : return per_element_ratio_selector< CV_8S  >( m, numerator, denominator, otype, opname );
        case CV_16U: return per_element_ratio_selector< CV_16U >( m, numerator, denominator, otype, opname );
        case CV_16S: return per_element_ratio_selector< CV_16S >( m, numerator, denominator, otype, opname );
        case CV_32S: return per_element_ratio_selector< CV_32S >( m, numerator, denominator, otype, opname );
        case CV_32F: return per_element_ratio_selector< CV_32F >( m, numerator, denominator, otype, opname );
        case CV_64F: return per_element_ratio_selector< CV_64F >( m, numerator, denominator, otype, opname );
    }
    COMMA_THROW( comma::exception, opname << ": unrecognised input image type " << m.second.type() );
}

static double max_value(int depth)
{
    switch(depth)
    {
        case CV_8U : return depth_traits< CV_8U  >::max_value();
        case CV_8S : return depth_traits< CV_8S  >::max_value();
        case CV_16U: return depth_traits< CV_16U >::max_value();
        case CV_16S: return depth_traits< CV_16S >::max_value();
        case CV_32S: return depth_traits< CV_32S >::max_value();
        case CV_32F: return depth_traits< CV_32F >::max_value();
        case CV_64F: return depth_traits< CV_64F >::max_value();
        default: { COMMA_THROW(comma::exception, "invalid depth: "<<depth ); }
    }
}

static cv::Mat convert_and_scale(const cv::Mat& m, int depth)
{
    cv::Mat result;
    double scale=max_value(depth)/max_value(m.depth());
    m.convertTo(result, CV_MAKETYPE(depth, m.channels()), scale);
    return result;
}

struct overlay_impl_
{
    int x;
    int y;
    cv::Mat overlay;
    int alpha;
    overlay_impl_(const std::string& image_file, int a, int b) : x(a), y(b), alpha(0)
    {
//         comma::verbose<<"overlay_impl_: image_file "<<image_file<<std::endl;
//         comma::verbose<<"overlay_impl_: x,y "<<x<<","<<y<<std::endl;
        overlay=cv::imread(image_file,-1);
        if(overlay.data==NULL) { COMMA_THROW( comma::exception, "failed to load image file: "<<image_file); }
    }
    filters::value_type operator()( filters::value_type m )
    {
        cv::Mat& mat=m.second;
//         comma::verbose<<"mat rows,cols,type;channels,depth "<<mat.rows<<","<<mat.cols<<","<<type_as_string(mat.type())<<";"<<mat.channels()<<","<<mat.depth()<<std::endl;
//         comma::verbose<<"overlay rows,cols,type;channels,depth "<<overlay.rows<<","<<overlay.cols<<","<<type_as_string(overlay.type())<<";"<<overlay.channels()<<","<<overlay.depth()<<std::endl;
        if(mat.channels()!=overlay.channels()-1) { COMMA_THROW(comma::exception, "mat's channels ("<<mat.channels()<<") should be one less than overlay's channel: "<<overlay.channels()); }
        if(mat.depth() != overlay.depth())
        {
            comma::verbose<<"converting overlay from depth "<<overlay.depth()<<" to "<<mat.depth()<<std::endl;
            overlay=convert_and_scale(overlay, mat.depth());
        }
        cv::Mat result(mat.rows, mat.cols, CV_MAKETYPE(overlay.depth(), overlay.channels()-1));
        switch(mat.depth())
        {
            case CV_8U: process<CV_8U>(mat, overlay, x, y, result); break;
            case CV_8S: process<CV_8S>(mat, overlay, x, y, result); break;
            case CV_16U: process<CV_16U>(mat, overlay, x, y, result); break;
            case CV_16S: process<CV_16S>(mat, overlay, x, y, result); break;
            case CV_32S: process<CV_32S>(mat, overlay, x, y, result); break;
            case CV_32F: process<CV_32F>(mat, overlay, x, y, result); break;
            case CV_64F: process<CV_64F>(mat, overlay, x, y, result); break;
            default: { COMMA_THROW( comma::exception, "invalid depth: " << mat.depth()); }
        }
        return filters::value_type(m.first, result);
    }
    template<int Depth> static void process(const cv::Mat& mat, const cv::Mat& overlay, int x, int y, cv::Mat& result)
    {
        double max_a=depth_traits< Depth >::max_value();
//         comma::verbose<<"overrlay process<Depth="<<Depth<<">: x,y "<<x<<","<<y<<" max_a "<<max_a<<std::endl;
        if(mat.depth() != Depth) { COMMA_THROW( comma::exception, "mat depth ("<<mat.depth() <<")mismatch, expected: "<< Depth); }
        if(overlay.depth() != Depth) { COMMA_THROW( comma::exception, "overlay depth ("<<overlay.depth() <<")mismatch, expected: "<< Depth); }
        typedef typename depth_traits< Depth >::value_t value_t;
        int alpha;
        int ch=overlay.channels();
        switch(ch)
        {
            case 2: alpha=1; break;
            case 4: alpha=3; break;
            default: { COMMA_THROW( comma::exception, "overlay needs to have alpha channel; expected number of channels 2 or 4, got "<<ch ); }
        }
//         comma::verbose<<"alpha: "<<alpha<<" ;result rows,cols,channels "<< result.rows<<","<<result.cols<<","<<result.channels()<<std::endl;
        //assert result.rows==mat.rows && result.cols==mat.cols
        int channels=result.channels();
        int y2=y+overlay.rows;
        int x2=x+overlay.cols;
        for(int j=0;j<result.rows;j++)
        {
            const value_t* mat_ptr=mat.ptr<value_t>(j);
            const value_t* overlay_ptr=NULL;
            if(j>=y && j<y2) { overlay_ptr=overlay.ptr<value_t>(j-y); }
            value_t* data=result.ptr<value_t>(j);
            int index=0;
            for(int i=0;i<result.cols;i++)
            {
                if(overlay_ptr!=NULL && i>=x && i<x2)
                {
                    double a=static_cast<double>(overlay_ptr[alpha]) / max_a;
                    for(int c=0;c<channels;c++)
                    {
                        double m=static_cast<double>(*mat_ptr++);
                        double o=static_cast<double>(*overlay_ptr++);
                        data[index++]= o * a + m * (1-a);
                    }
                    //skip alpha
                    overlay_ptr++;
                }
                else
                {
                    //just copy
                    for(int c=0;c<channels;c++)
                        data[index++]=*mat_ptr++;
                }
            }
        }
    }
};

template < unsigned int Depth > struct gamma_traits {};

template <> struct gamma_traits< CV_8U >
{
    typedef unsigned char type;
    enum { is_signed = false };
    static const type min = 0;
    static const type max = 255;
};

template <> struct gamma_traits< CV_8S >
{
    typedef char type;
    enum { is_signed = true };
    static const type min = -128;
    static const type max = 127;
};

template < unsigned int Depth > static cv::Mat lut_matrix_gamma_( double gamma )
{
    double num_states = gamma_traits< Depth >::max - gamma_traits< Depth >::min; //std::numeric_limits< gamma_traits< Depth >::type >::max();
    cv::Mat lut_matrix( 1, num_states + 1, Depth );
    uchar * ptr = lut_matrix.ptr();
    double scale = std::abs( gamma_traits< Depth >::max );
    for( unsigned int i = 0, j = gamma_traits< Depth >::min; i <= num_states; i++, j++ )
    {
        ptr[i] = std::pow( j / scale, 1.0 / gamma ) * scale;
    }
    return lut_matrix;
}

template < unsigned int Depth >
static filters::value_type gamma_( const filters::value_type m, const double gamma )
{
    static double gamma_ = gamma;
    if( gamma_ != gamma ) { COMMA_THROW( comma::exception, "multiple filters with different gamma values: todo" ); }
    static cv::Mat lut_matrix = lut_matrix_gamma_<Depth>( gamma );
    cv::LUT( m.second, lut_matrix, m.second );
    return m;
}

static filters::value_type gamma_impl_(const filters::value_type m, const double gamma )
{
    switch( m.second.depth() )
    {
        case CV_8U: { return gamma_< CV_8U >( m, gamma ); break; }
        default: break;
    }
    COMMA_THROW(comma::exception, "gamma is unimplemented for types other than CV_8U, CV_8S");
}

static filters::value_type inrange_impl_( const filters::value_type m, const cv::Scalar& lower, const cv::Scalar& upper )
{
    filters::value_type n;
    n.first = m.first;
    n.second = cv::Mat( m.second.rows, m.second.cols, single_channel_type( m.second.type() ) );
    cv::inRange( m.second, lower, upper, n.second );
    return n;
}

struct load_impl_
{
    filters::value_type value;
    
    load_impl_( const std::string& filename )
    {
        const std::vector< std::string >& v = comma::split( filename, '.' );
        if( v.back() == "bin" ) // quick and dirty
        {
            std::ifstream ifs( &filename[0] );
            if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open \"" << filename << "\"" ); }
            serialization s( "t,rows,cols,type", comma::csv::format( "t,3ui" ) ); // quick and dirty
            value = s.read( ifs );
            ifs.close();
        }
        else
        {
            value.second = cv::imread( filename, -1 );
        }
        if( value.second.data == NULL ) { COMMA_THROW( comma::exception, "failed to load image from file \""<< filename << "\"" ); }
    }
    
    filters::value_type operator()( filters::value_type ) { return value; }
};

static filters::value_type remove_mean_impl_(const filters::value_type m, const cv::Size kernel_size, const double ratio )
{
    filters::value_type n;
    n.first = m.first;
    cv::GaussianBlur(m.second, n.second, kernel_size, 0, 0);
    n.second = m.second - ratio * n.second;
    return n;
}

static boost::function< filter::input_type( filter::input_type ) > make_filter_functor( const std::vector< std::string >& e )
{
    if( e[0] == "convert-color" || e[0] == "convert_color" )
    {
        if( e.size() == 1 ) { COMMA_THROW( comma::exception, "convert-color: please specify conversion" ); }
        return boost::bind( &cvt_color_impl_, _1, cvt_color_type_from_string( e[1] ) );
    }
    if( e[0] == "count" ) { return count_impl_(); }
    if( e[0] == "crop" )
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
                COMMA_THROW( comma::exception, "expected crop=[x,y,]width,height, got \"" << comma::join( e, '=' ) << "\"" );
        }
        return boost::bind( &crop_impl_, _1, x, y, w, h );
    }
    if( e[0] == "crop-cols" )
    {
        if( e.size() < 2 ) { COMMA_THROW( comma::exception, "crop-cols: specify at least one column to extract, e.g. crop-cols=1,10" ); }
        std::vector< std::string > stripes = comma::split( e[1], '|' );
        std::vector< stripe_t > cols;
        for ( size_t s = 0; s < stripes.size(); ++s )
        {
            std::vector< std::string > column = comma::split( stripes[s], ',' );
            if ( column.size() > 2 ) { COMMA_THROW( comma::exception, "crop-cols: expected position,[width]; got " << column.size() << " parameters '" << stripes[s] << "'" ); }
            unsigned int x = boost::lexical_cast< unsigned int >( column[0] );
            unsigned int w = ( column.size() == 2 ? boost::lexical_cast< unsigned int >( column[1] ) : 1 );
            cols.push_back( std::make_pair( x, w ) );
        }
        return boost::bind( &crop_cols_impl_, _1, cols );
    }
    if( e[0] == "crop-rows" )
    {
        if( e.size() < 2 ) { COMMA_THROW( comma::exception, "crop-rows: specify at least one row to extract, e.g. crop-rows=1,10" ); }
        std::vector< std::string > stripes = comma::split( e[1], '|' );
        std::vector< stripe_t > rows;
        for ( size_t s = 0; s < stripes.size(); ++s )
        {
            std::vector< std::string > rowblock = comma::split( stripes[s], ',' );
            if ( rowblock.size() > 2 ) { COMMA_THROW( comma::exception, "crop-rows: expected position,[height]; got " << rowblock.size() << " parameters '" << stripes[s] << "'" ); }
            unsigned int y = boost::lexical_cast< unsigned int >( rowblock[0] );
            unsigned int h = ( rowblock.size() == 2 ? boost::lexical_cast< unsigned int >( rowblock[1] ) : 1 );
            rows.push_back( std::make_pair( y, h ) );
        }
        return boost::bind( &crop_rows_impl_, _1, rows );
    }
    if( e[0] == "bands-to-cols" || e[0] == "bands-to-rows" )
    {
        // rhs looks like "12,23,50,30,100,1,method:average,output-depth:d"
        // the ','-separated entries shall be either:
        // - integers always coming in pairs, or
        // - colon-separated words with a known keyword on the left and one of the known enumeration names on the right
        const bool bands_to_cols = e[0] == "bands-to-cols";
        const std::string & op_name = bands_to_cols ? "bands-to-cols" : "bands-to-rows";
        if( e.size() < 2 ) { COMMA_THROW( comma::exception, op_name << ": specify at least one band to extract, e.g. " << op_name << "=1,10" ); }
        std::vector< std::string > stripes = comma::split( e[1], ',' );
        std::vector< stripe_t > bands;
        // iterate over pair of integers until all taken; then iterate over "name:value" pairs
        size_t s = 0;
        while ( s < stripes.size() )
        {
            if ( stripes[s].empty() ) { COMMA_THROW( comma::exception, op_name << ": empty comma-separated field in '" << e[1] << "'" ); }
            unsigned int y;
            try {
                y = boost::lexical_cast< int >( stripes[s] );
            } catch ( boost::bad_lexical_cast & ) {
                break; // possibly a keyword, not an error
            }
            if ( ! (++s < stripes.size() ) ) { COMMA_THROW( comma::exception, op_name << ": expected <int, int> pairs, got a single int in '" << e[1] << "'" ); }
            unsigned int h;
            try {
                h = boost::lexical_cast< int >( stripes[s] );
            } catch ( boost::bad_lexical_cast & ) {
                COMMA_THROW( comma::exception, op_name << ": expected <position>,<size> integer pairs, got " << e[1] );
            }
            bands.push_back( std::make_pair( y, h ) );
            ++s;
        }
        // the rest of the string shall be comma-separated name:value pairs
        int cv_reduce_method = bands_method_default;
        int cv_reduce_dtype = -1;
        while ( s < stripes.size() )
        {
            if ( stripes[s].empty() ) { COMMA_THROW( comma::exception, op_name << ": empty comma-separated field in '" << e[1] << "'" ); }
            std::vector< std::string > setting = comma::split( stripes[s], ':' );
            if ( setting.size() != 2 ) { COMMA_THROW( comma::exception, op_name << ": expected keyword:value; got " << setting.size() << " parameter '" << stripes[s] << "'" ); }
            if ( setting[0] == "method" )
            {
                static std::map< std::string, int > methods = boost::assign::map_list_of ( "average", CV_REDUCE_AVG ) ( "sum", CV_REDUCE_SUM ) ( "min", CV_REDUCE_MIN ) ( "max", CV_REDUCE_MAX );
                std::map< std::string, int >::const_iterator found = methods.find( setting[1] );
                if ( found == methods.end() ) { COMMA_THROW( comma::exception, op_name << ": the method is not one of [average,sum,min,max]" ); }
                cv_reduce_method = found->second;
            }
            else if ( setting[0] == "output-depth" )
            {
                // the permitted list is very restrictive and explicit
                static std::map< std::string, int > depths = boost::assign::map_list_of ( "CV_32S", CV_32S ) ( "i", CV_32S ) ( "CV_32F", CV_32F ) ( "f", CV_32F ) ( "CV_64F", CV_64F ) ( "d", CV_64F );
                std::map< std::string, int >::const_iterator found = depths.find( setting[1] );
                if ( found == depths.end() ) { COMMA_THROW( comma::exception, op_name << ": the output-depth '" << setting[1] << "' is not one of [i,f,d] or [CV_32S,CV_32F,CV_64F]" ); }
                cv_reduce_dtype = found->second;
            }
            else
            {
                COMMA_THROW( comma::exception, op_name << ": the keyword '" << setting[0] << "' is not one of [method,output-depth]" );
            }
            ++s;
        }
        if ( bands.empty() ) { COMMA_THROW( comma::exception, op_name << ": specify at least one band" ); }
        return boost::bind( &bands_to_cols_impl_, _1, bands_to_cols, bands, cv_reduce_method, cv_reduce_dtype );
    }
    if( e[0] == "crop-tile" )
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
        return boost::bind( &crop_tile_impl_, _1, number_of_tile_cols, number_of_tile_rows, tiles, vertical );
    }
    if( e[0] == "cols-to-channels" || e[0] == "rows-to-channels" )
    {
        // rhs looks like "cols-to-channels=1,4,5[|pad:value|repeat:step]"
        // the '|'-separated entries shall be either:
        // - a comma-separated lists of integers, or
        // - a single integer, or
        // - colon-separated words with a known keyword on the left and one of the known enumeration names on the right
        const bool cols_to_channels = e[0] == "cols-to-channels";
        const std::string & op_name = cols_to_channels ? "cols-to-channels" : "rows-to-channels";
        const std::string & op_what = cols_to_channels ? "column" : "row";
        if( e.size() < 2 ) { COMMA_THROW( comma::exception, op_name << ": specify at least one column or column list to extract, e.g. " << op_name << "=1,10" ); }
        std::vector< std::string > inputs = comma::split( e[1], '|' );
        std::vector< unsigned int > values;
        double padding = 0.0;
        unsigned int repeat = 0;
        for ( size_t s = 0; s < inputs.size(); ++s )
        {
            if ( inputs[s].find( ":" ) != std::string::npos )
            {
                std::vector< std::string > setting = comma::split( inputs[s], ':' );
                if ( setting.size() != 2 ) { COMMA_THROW( comma::exception, op_name << ": expected keyword:value; got " << setting.size() << " parameters '" << inputs[s] << "'" ); }
                if ( setting[0] == "pad" )
                {
                    padding = boost::lexical_cast< double >( setting[1] );
                }
                else if ( setting[0] == "repeat" )
                {
                    repeat = boost::lexical_cast< unsigned int >( setting[1] );
                }
                else
                {
                    COMMA_THROW( comma::exception, op_name << ": the keyword '" << setting[0] << "' is not one of [pad,repeat]" );
                }
            }
            else
            {
                std::vector< std::string > vstrings = comma::split( inputs[s], ',' );
                if ( vstrings.size() > 4 ) { COMMA_THROW( comma::exception, op_name << ": can store up to 4 " << op_what << "s into channels, got " << vstrings.size() << " inputs '" << inputs[s] << "'" ); }
                values.reserve( vstrings.size() );
                for ( size_t i = 0; i < vstrings.size(); ++i ) { values.push_back( boost::lexical_cast< unsigned int >( vstrings[i] ) ); }
            }
        }
        if ( values.empty() ) { COMMA_THROW( comma::exception, op_name << ": specify at least one " << op_what << " to store as channel" ); }
        if ( values.size() > 4 ) { COMMA_THROW( comma::exception, op_name << ": can have at most 4 output channels" ); }
        return boost::bind( &cols_to_channels_impl_, _1, cols_to_channels, values, padding, repeat );
    }
    if( e[0] == "channels-to-cols" || e[0] == "channels-to-rows" )
    {
        const bool channels_to_cols = e[0] == "channels-to-cols";
        return boost::bind( &channels_to_cols_impl_, _1, channels_to_cols );
    }
    if( e[0] == "swap-channels" )
    {
        COMMA_THROW( comma::exception, "NYI" );
        // use cv::reshape or cv::mixChannels
    }
    if( e[0] == "cross" )
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
        return boost::bind( &cross_impl_, _1, center );
    }
    if( e[0] == "circle" ) // todo: quick and dirty, implement using traits
    {
        boost::array< int, 9 > p = {{ 0, 0, 0, 0, 0, 0, 1, 8, 0 }};
        const std::vector< std::string > v = comma::split( e[1], ',' );
        for( unsigned int i = 0; i < v.size(); ++i ) { if( !v[i].empty() ) { p[i] = boost::lexical_cast< int >( v[i] ); } }
        return boost::bind( &circle_impl_, _1, drawing::circle( cv::Point( p[0], p[1] ), p[2], cv::Scalar( p[5], p[4], p[3] ), p[6], p[7], p[8] ) );
    }
    if( e[0] == "rectangle" || e[0] == "box" ) // todo: quick and dirty, implement using traits
    {
        boost::array< int, 10 > p = {{ 0, 0, 0, 0, 0, 0, 0, 1, 8, 0 }};
        const std::vector< std::string > v = comma::split( e[1], ',' );
        for( unsigned int i = 0; i < v.size(); ++i ) { if( !v[i].empty() ) { p[i] = boost::lexical_cast< int >( v[i] ); } }
        return boost::bind( &rectangle_impl_, _1, drawing::rectangle( cv::Point( p[0], p[1] ), cv::Point( p[2], p[3] ), cv::Scalar( p[6], p[5], p[4] ), p[7], p[8], p[9] ) );
    }
    if( e[0] == "gamma" ) { return boost::bind( &gamma_impl_, _1, boost::lexical_cast< double >( e[1] ) ); }
    if( e[0] == "remove-mean")
    {
        std::vector< std::string > s = comma::split( e[1], ',' );
        if( s.size() != 2 ) { COMMA_THROW( comma::exception, "remove-mean expected 2 parameters" ); }
        unsigned int neighbourhood_size = boost::lexical_cast< unsigned int >( s[0] );
        cv::Size kernel_size(neighbourhood_size, neighbourhood_size);
        double ratio = boost::lexical_cast< double >( s[1] );
        return boost::bind( &remove_mean_impl_, _1, kernel_size, ratio );
    }
    if( e[0] == "fft" )
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
        return boost::bind( &fft_impl_, _1, direct, complex, magnitude, log_scale, normalize );
    }
    if( e[0] == "flip" ) { return boost::bind( &flip_impl_, _1, 0 ); }
    if( e[0] == "flop" ) { return boost::bind( &flip_impl_, _1, 1 ); }
    if( e[0] == "magnitude" ) { return boost::bind( &magnitude_impl_, _1 ); }
    if( e[0] == "text" )
    {
        if( e.size() <= 1 ) { COMMA_THROW( comma::exception, "text: expected text value" ); }
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
            else { COMMA_THROW( comma::exception, "expected colour of text in \"" << comma::join( e, '=' ) << "\", got '" << w[3] << "'" ); }
        }
        return boost::bind( &text_impl_, _1, w[0], p, s );
    }
    if( e[0] == "convert-to" || e[0] == "convert_to" )
    {
        if( e.size() <= 1 ) { COMMA_THROW( comma::exception, "convert-to: expected options, got none" ); }
        const std::vector< std::string >& w = comma::split( e[1], ',' );
        boost::unordered_map< std::string, int >::const_iterator it = types_.find( w[0] );
        if( it == types_.end() ) { COMMA_THROW( comma::exception, "convert-to: expected target type, got \"" << w[0] << "\"" ); }
        double scale = w.size() > 1 ? boost::lexical_cast< double >( w[1] ) : 1.0;
        double offset = w.size() > 2 ? boost::lexical_cast< double >( w[2] ) : 0.0;
        return boost::bind( &convert_to_impl_, _1, it->second, scale, offset );
    }
    if( e[0] == "resize" )
    {
        unsigned int width = 0;
        unsigned int height = 0;
        double w = 0;
        double h = 0;
        const std::vector< std::string >& r = comma::split( e[1], ',' );
        int interpolation = cv::INTER_LINEAR;
        unsigned int size = r.size();
        if( r.size() > 1 && r.back()[0] >= 'a' && r.back()[0] <= 'z' )
        {
            if( r.back() == "nearest" ) { interpolation = cv::INTER_NEAREST; }
            else if( r.back() == "linear" ) { interpolation = cv::INTER_LINEAR; }
            else if( r.back() == "area" ) { interpolation = cv::INTER_AREA; }
            else if( r.back() == "cubic" ) { interpolation = cv::INTER_CUBIC; }
            else if( r.back() == "lanczos4" ) { interpolation = cv::INTER_LANCZOS4; }
            else { COMMA_THROW( comma::exception, "resize: expected interpolation type, got: \"" << e[1] << "\"" ); }
            --size;
        }
        else if( r.size() == 3 ) { interpolation = boost::lexical_cast< int >( r[3] ); }
        switch( size )
        {
            case 1:
                w = h = boost::lexical_cast< double >( r[0] );
                break;
            case 2:
            case 3:
                try { width = boost::lexical_cast< unsigned int >( r[0] ); }
                catch ( ... ) { w = boost::lexical_cast< double >( r[0] ); }
                try { height = boost::lexical_cast< unsigned int >( r[1] ); }
                catch ( ... ) { h = boost::lexical_cast< double >( r[1] ); }
                break;
            default:
                COMMA_THROW( comma::exception, "expected resize=<width>,<height>, got: \"" << e[1] << "\"" );
        }
        return boost::bind( &resize_impl_, _1, width, height, w, h, interpolation );
    }
    if( e[0] == "mask" )
    {
        if( e.size() == 1 ) { COMMA_THROW( comma::exception, "mask: please specify mask filters" ); }
        if( e.size() > 2 ) { COMMA_THROW( comma::exception, "mask: expected 1 parameter; got: " << comma::join( e, '=' ) ); }
        std::string filter_string = e[1];
        const std::vector< std::string > w = comma::split( filter_string, '|' ); // quick and dirty, running out of delimiters
        boost::function< filter::input_type( filter::input_type ) > g = make_filter_functor( comma::split( w[0], ':' ) );
        for( unsigned int k = 1; k < w.size(); ++k ) { g = boost::bind( make_filter_functor( comma::split( w[k], ':' ) ), boost::bind( g, _1 ) ); }
        return boost::bind( &mask_impl_, _1, g );
    }
    else if( e[0] == "timestamp" ) { return &timestamp_impl_; }
    else if( e[0] == "transpose" ) { return &transpose_impl_; }
    else if( e[0] == "split" ) { return &split_impl_; }
    else if( e[0] == "merge" )
    {
        unsigned int default_number_of_channels = 3;
        unsigned int nchannels = e.size() == 1 ? default_number_of_channels : boost::lexical_cast< unsigned int >( e[1] );
        if ( nchannels == 0 ) { COMMA_THROW( comma::exception, "expected positive number of channels in merge filter, got " << nchannels ); }
        return boost::bind( &merge_impl_, _1, nchannels );
    }
    if( e[0] == "undistort" ) { return undistort_impl_( e[1] ); }
    if( e[0] == "invert" )
    {
        if( e.size() == 1 ) { return &invert_impl_; }
        else if( e[1] == "brightness" ) { return &invert_brightness_impl_; } // quick and dirty, a secret option
    }
    if(e[0]=="normalize")
    {
        if(e[1]=="max") { return &normalize_max_impl_; }
        else if(e[1]=="sum") { return &normalize_sum_impl_; }
        else if(e[1]=="all") { return &normalize_cv_impl_; }
        else { COMMA_THROW( comma::exception, "expected max or sum option for normalize, got" << e[1] ); }
    }
    if( e[0]=="equalize-histogram" ) { return &equalize_histogram_impl_; }
    if( e[0] == "brightness" || e[0] == "scale" )
    {
        const std::vector< std::string >& s = comma::split( e[1], ',' );
        double scale = boost::lexical_cast< double >( s[0] );
        double offset = s.size() == 1 ? 0.0 : boost::lexical_cast< double >( s[1] );
        return boost::bind( &brightness_impl_, _1, scale, offset );
    }
    if( e[0] == "color-map" )
    {
        if( e.size() != 2 ) { COMMA_THROW( comma::exception, "expected colour-map=<type>; got: \"" << e[1] << "\"" ); }
        int type;
        if( e[1] == "autumn" ) { type = cv::COLORMAP_AUTUMN; }
        else if( e[1] == "bone" ) { type = cv::COLORMAP_BONE; }
        else if( e[1] == "jet" ) { type = cv::COLORMAP_JET; }
        else if( e[1] == "winter" ) { type = cv::COLORMAP_WINTER; }
        else if( e[1] == "rainbow" ) { type = cv::COLORMAP_RAINBOW; }
        else if( e[1] == "ocean" ) { type = cv::COLORMAP_OCEAN; }
        else if( e[1] == "summer" ) { type = cv::COLORMAP_SUMMER; }
        else if( e[1] == "spring" ) { type = cv::COLORMAP_SPRING; }
        else if( e[1] == "cool" ) { type = cv::COLORMAP_COOL; }
        else if( e[1] == "hsv" ) { type = cv::COLORMAP_HSV; }
        else if( e[1] == "pink" ) { type = cv::COLORMAP_PINK; }
        else if( e[1] == "hot" ) { type = cv::COLORMAP_HOT; }
        else { COMMA_THROW( comma::exception, "expected colour-map type; got: \"" << e[1] << "\"" ); }
        return boost::bind( &colour_map_impl_, _1, type );
    }
    if( e[0] == "blur" )
    {
        const std::vector< std::string >& s = comma::split( e[1], ',' );
        if( s.size() < 2 ) { COMMA_THROW( comma::exception, "expected blur=<blur_type>,<blur_type_parameters>" ); }
        
        blur_t params;
        params.blur_type = blur_t::from_string(s[0]);
        if (s[0] == "box")
        {
            params.kernel_size.height = params.kernel_size.width = boost::lexical_cast< int >( s[1] );
        }
        else if (s[0] == "gaussian")
        {
            if (s.size() != 3) { COMMA_THROW( comma::exception, "expected blur=gaussian,kernel_size,std" ); }
            params.kernel_size.width = params.kernel_size.height = boost::lexical_cast< int >( s[1] );
            params.std.x = params.std.y = boost::lexical_cast< double >( s[2] );
        }
        else if (s[0] == "median")
        {
            if (s.size() != 2) { COMMA_THROW( comma::exception, "blur=median,kernel_size" ); }
            params.neighbourhood_size = boost::lexical_cast< int >( s[1] );
        }
        else if (s[0] == "bilateral")
        {
            if (s.size() != 4) // nsize,std_space,std_colour
            { COMMA_THROW( comma::exception, "blur=bilateral expected 3 parameters" ); }
            params.neighbourhood_size = boost::lexical_cast< int >( s[1] );
            params.sigma_space = boost::lexical_cast< double >( s[2] );
            params.sigma_colour = boost::lexical_cast< double >( s[3] ); 
        }
        else if (s[0] == "adaptive-bilateral")
        {
            if (s.size() != 4) { COMMA_THROW( comma::exception, "expected blur=adaptive-bilateral,kernel_size,std_space,std_colour_max" ); }
            params.kernel_size.width = params.kernel_size.height = boost::lexical_cast< int >( s[1] );
            params.sigma_space = boost::lexical_cast< double >( s[2] );
            params.sigma_colour = boost::lexical_cast< double >( s[3] ); // max sigma color
        }
        else { COMMA_THROW( comma::exception, "invalid blur type" ); }
        return boost::bind( &blur_impl_, _1, params );
    }
    if( e[0] == "load" )
    {
        if( e.size() < 2 ) { COMMA_THROW( comma::exception, "please specify filename load=<filename>" ); }
        return load_impl_( e[1] );
    }
    if( e[0] == "map" )
    {
        if( e.size() < 2 ) { COMMA_THROW( comma::exception, "expected file name with the map, e.g. map=f.csv" ); }
        std::stringstream s; s << e[1]; for( std::size_t i = 2; i < e.size(); ++i ) { s << "=" << e[i]; }
        std::string map_filter_options = s.str();
        std::vector< std::string > items = comma::split( map_filter_options, '&' );
        bool permissive = std::find( items.begin()+1, items.end(), "permissive" ) != items.end();
        return map_impl_( map_filter_options, permissive );
    }
    if( e[0] == "inrange" )
    {
        const std::vector< std::string >& s = comma::split( e[1], ',' );
        if( s.size() < 2 || s.size() % 2 != 0 ) { COMMA_THROW( comma::exception, "inrange: expected <upper>,<lower> got: \"" << comma::join( e, '=' ) << "\"" ); }
        cv::Scalar lower = scalar_from_strings( &s[0], s.size() / 2 );
        cv::Scalar upper = scalar_from_strings( &s[ s.size() / 2 ], s.size() / 2 );
        return boost::bind( &inrange_impl_, _1, lower, upper );
    }
    if( e[0] == "threshold" )
    {
        const std::vector< std::string >& s = comma::split( e[1], ',' );
        if( s[0].empty() ) { COMMA_THROW( comma::exception, "threshold: expected <threshold|otsu>[,<maxval>[,<type>]] got: \"" << comma::join( e, '=' ) << "\"" ); }
        bool otsu = s[0] == "otsu";
        double threshold = otsu ? 0 : boost::lexical_cast< double >( s[0] );
        double maxval = s.size() < 2 ? 255 : boost::lexical_cast< double >( s[1] );
        threshold_t::types type = threshold_t::from_string( s.size() < 3 ? "" : s[2] );
        return boost::bind( &threshold_impl_, _1, threshold, maxval, type, otsu );
    }
    if( e[0] == "linear-combination" || e[0] == "ratio" )
    {
        typedef std::string::const_iterator iterator_type;
        ratios::rules< iterator_type > rules;
        ratios::parser< iterator_type, ratios::ratio > parser( rules.ratio_ );
        ratios::ratio r;
        iterator_type begin = e[1].begin();
        iterator_type end = e[1].end();
        bool status = phrase_parse( begin, end, parser, boost::spirit::ascii::space, r );
        if ( !status || ( begin != end ) ) { COMMA_THROW( comma::exception, e[0] << ": expected a " << e[0] << " expression, got: \"" << comma::join( e, '=' ) << "\"" ); }
        if ( e[0] == "linear-combination" && !r.denominator.unity() ) { COMMA_THROW( comma::exception, e[0] << ": expected a linear combination expression, got a ratio" ); }
        std::vector< double > numerator( r.numerator.terms.size() );
        for( size_t j = 0; j < r.numerator.terms.size(); ++j ) { numerator[j] = r.numerator.terms[j].value; }
        std::vector< double > denominator( r.denominator.terms.size() );
        for( size_t j = 0; j < r.denominator.terms.size(); ++j ) { denominator[j] = r.denominator.terms[j].value; }
        return boost::bind( &ratio_impl_, _1, numerator, denominator, e[0] );
    }
    if( e[0] == "overlay" )
    {
        if( e.size() != 2 ) { COMMA_THROW( comma::exception, "expected file name (and optional x,y) with the overlay, e.g. overlay=a.svg" ); }
        std::vector< std::string > s = comma::split( e[1], ',' );
        if(s.size()!=1 && s.size()!=3) { COMMA_THROW( comma::exception, "expected one or three parameters (file[,x,y]); found " << s.size() ); }
        int x=0, y=0;
        if(s.size()>1)
        {
            x=boost::lexical_cast<int>(s[1]);
            y=boost::lexical_cast<int>(s[2]);
        }
        return overlay_impl_( s[0], x, y );
    }
    boost::function< cv_mat::filters::value_type( cv_mat::filters::value_type ) > functor = imaging::vegetation::filters::make_functor( e );
    if( functor ) { return functor; }
    COMMA_THROW( comma::exception, "expected filter, got: \"" << comma::join( e, '=' ) << "\"" );
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
        if( e[0] == "accumulate" )
        {
            unsigned int how_many = boost::lexical_cast< unsigned int >( e[1] );
            if ( how_many == 0 ) { COMMA_THROW( comma::exception, "expected positive number of images to accumulate in accumulate filter, got " << how_many ); }
            f.push_back( filter( accumulate_impl_( how_many ), false ) );
        }
        else if( e[0] == "bayer" ) // kept for backwards-compatibility, use convert-color=BayerBG,BGR etc..
        {
            if( modified ) { COMMA_THROW( comma::exception, "cannot covert from bayer after transforms: " << name ); }
            unsigned int which = boost::lexical_cast< unsigned int >( e[1] ) + 45u; // HACK, bayer as unsigned int, but I don't find enum { BG2RGB, GB2BGR ... } more usefull
            f.push_back( filter( boost::bind( &cvt_color_impl_, _1, which ) ) );
        }
        else if( e[0] == "unpack12" )
        {
            if( modified ) { COMMA_THROW( comma::exception, "cannot covert from 12 bit packed after transforms: " << name ); }
            if(e.size()!=1) { COMMA_THROW( comma::exception, "unexpected arguement: "<<e[1]); }
            f.push_back( filter( boost::bind( &unpack12_impl_, _1 ) ) );
        }
        else if( e[0] == "log" ) // todo: rotate log by size: expose to user
        {
            boost::optional < double > period;
            boost::optional < comma::uint32 > size;
            bool index = false;
            if( e.size() <= 1 ) { COMMA_THROW( comma::exception, "please specify log=<filename> or log=<directory>[,<options>]" ); }
            const std::vector< std::string >& w = comma::split( e[1], ',' );
            
            std::string file = w[0];
            for ( std::size_t option = 1; option < w.size(); ++option )
            {
                const std::vector< std::string >& u = comma::split( w[option], ':' );
                if( u[0] == "period" )
                {
                    if (size) { COMMA_THROW( comma::exception, "log: expected \"period\" or \"size\"; got both" ); }
                    if( u.size() <= 1 ) { COMMA_THROW( comma::exception, "log: please specify period:<seconds>" ); }
                    period = boost::lexical_cast< double >( u[1] );
                }
                else if( u[0] == "size" )
                {
                    if (period) { COMMA_THROW( comma::exception, "log: expected \"period\" or \"size\"; got both" ); }
                    if( u.size() <= 1 ) { COMMA_THROW( comma::exception, "log: please specify size:<number of frames>" ); }
                    size = boost::lexical_cast< comma::uint32 >( u[1] );
                }
                else if( u[0] == "index" )
                {
                    index = true;
                }
                else
                {
                    COMMA_THROW( comma::exception, "log: expected \"index\", \"period\" or \"size\"; got: \"" << u[0] << "\"" );
                }
            }
            if (period) 
            {   
                unsigned int seconds = static_cast< unsigned int >( *period );
                f.push_back( filter( log_impl_( file, boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( ( *period - seconds ) * 1000000 ), index ), false ) ); 
            }
            else if ( size )
            { 
                f.push_back( filter( log_impl_( file, *size, index), false ) );
            } 
            else
            { 
                if( index ) { COMMA_THROW( comma::exception, "log: index should be specified with directory and period or size, not with filename" );  }
                f.push_back( filter( log_impl_( file ), false ) );
            }
        }
        else if( e[0] == "max" ) // todo: remove this filter; not thread-safe, should be run with --threads=1
        {
            f.push_back( filter( max_impl_( boost::lexical_cast< unsigned int >( e[1] ), true ), false ) );
        }
        else if( e[0] == "min" ) // todo: remove this filter; not thread-safe, should be run with --threads=1
        {
            f.push_back( filter( max_impl_( boost::lexical_cast< unsigned int >( e[1] ), false ), false ) );
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
            if( i < v.size() - 1 )
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
        else if( e[0] == "simple-blob" )
        {
            if( i < v.size() - 1 ) { COMMA_THROW( comma::exception, "expected 'simple-blob' as the last filter, got \"" << how << "\"" ); }
            std::vector< std::string > s;
            if( e.size() > 1 ) { s = comma::split( e[1], ',' ); }
            bool binary = false;
            std::string config;
            std::string path;
            for( unsigned int i = 0; i < s.size(); ++i )
            {
                if( s[i] == "output-binary" ) { binary = true; }
                if( s[i] == "output-fields" ) { std::cout << comma::join( comma::csv::names< snark::timestamped< cv::KeyPoint > >( false ), ',' ) << std::endl; exit( 0 ); }
                if( s[i] == "output-format" ) { std::cout << comma::csv::format::value< snark::timestamped< cv::KeyPoint > >() << std::endl; exit( 0 ); }
                if( s[i] == "output-default-params" || s[i] == "default-params" )
                {
                    cv::FileStorage fs( "dummy", cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_XML );
                    cv::SimpleBlobDetector::Params().write( fs );
                    std::cout << fs.releaseAndGetString() << std::endl;
                    exit( 0 ); // hyper quick and dirty
                }
                else
                {
                    const std::vector< std::string >& t = comma::split( s[i], ':' );
                    config = t[0];
                    if( t.size() > 1 ) { path = s[1]; }
                }
            }
            f.push_back( filter( boost::bind( &simple_blob_impl_, _1, cv_read_< cv::SimpleBlobDetector::Params >( config, path ), binary ) ) );
            f.push_back( filter( NULL ) ); // quick and dirty
        }
        else if( e[0] == "null" )
        {
            if( i < v.size() - 1 ) { COMMA_THROW( comma::exception, "expected 'null' as the last filter, got \"" << how << "\"" ); }
            if( i == 0 ) { COMMA_THROW( comma::exception, "'null' as the only filter is not supported; use cv-cat > /dev/null, if you need" ); }
            f.push_back( filter( NULL ) );
        }
        else if ( e[0] == "head" )
        {
            if( i < v.size() - 1 )
            {
                std::string next_filter = comma::split( v[i+1], '=' )[0];
                if( next_filter != "null" && next_filter != "encode" ) { COMMA_THROW( comma::exception, "cannot have a filter after head unless next filter is null or encode" ); }
            }
            unsigned int n = e.size() < 2 ? 1 : boost::lexical_cast< unsigned int >( e[1] );
            f.push_back( filter( boost::bind( &head_impl_, _1, n ), false ) );
        }
        else
        {
            f.push_back( filter( make_filter_functor( e ) ) );
        }
        modified = e[0] != "view" && e[0] != "thumb" && e[0] != "split" && e[0] !="unpack12";
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
    oss << "        bayer=<mode>: convert from bayer, <mode>=1-4 (see also convert-color)" << std::endl;
    oss << "        blur=<type>,<parameters>: apply a blur to the image (positive and odd kernel sizes)" << std::endl;
    oss << "            blur=box,<kernel_size> " << std::endl;
    oss << "            blur=median,<kernel_size>" << std::endl;
    oss << "            blur=gaussian,<kernel_size>,<std_size>. Set either std or kernel size to 0 to calculate based on other parameter." << std::endl;
    oss << "            blur=bilateral,<neighbourhood_size>,<sigma_space>,<sigma_colour>; preserves edges" << std::endl;
    oss << "            blur=adaptive-bilateral: <kernel_size>,<sigma_space>,<sigma_colour_max>; preserve edges, automatically calculate sigma_colour (and cap to sigma_colour_max)" << std::endl;
    oss << "        brightness,scale=<scale>[,<offset>]: output=(scale*input)+offset; default offset=0" << std::endl;
    oss << "        color-map=<type>: take image, apply colour map; see cv::applyColorMap for detail" << std::endl;
    oss << "            <type>: autumn, bone, jet, winter, rainbow, ocean, summer, spring, cool, hsv, pink, hot" << std::endl;
    oss << "        convert-to,convert_to=<type>[,<scale>[,<offset>]]: convert to given type; should be the same number of channels; see opencv convertTo for details; values will not overflow" << std::endl;
    oss << "        convert-color,convert_color=<from>,<to>: convert from colour space to new colour space (BGR, RGB, Lab, XYZ, Bayer**, GRAY); eg: BGR,GRAY or CV_BGR2GRAY" << std::endl;
    oss << "        count: write frame number on images" << std::endl;
    oss << "        crop=[<x>,<y>],<width>,<height>: crop the portion of the image starting at x,y with size width x height" << std::endl;
    oss << "        crop-tile=<ncols>,<nrows>,<i>,<j>,...[&horizontal]: divide the image into a grid of tiles (ncols-by-nrows), and output an image made of the croped tiles defined by i,j (count from zero)" << std::endl;
    oss << "            <horizontal>: if present, tiles will be stacked horizontally (by default, vertical stacking is used)" << std::endl;
    oss << "            example: \"crop-tile=2,5,1,0,1,4&horizontal\"" << std::endl;
    oss << "            deprecated: old syntax <i>,<j>,<ncols>,<nrows> is used for one tile if i < ncols and j < ncols" << std::endl;
    oss << "        encode=<format>: encode images to the specified format. <format>: jpg|ppm|png|tiff..., make sure to use --no-header" << std::endl;
    oss << "        equalize-histogram: todo: equalize each channel by its histogram" << std::endl;
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
    oss << "        inrange=<lower>,<upper>: a band filter on r,g,b or greyscale image; for rgb: <lower>::=<r>,<g>,<b>; <upper>::=<r>,<g>,<b>; see cv::inRange() for detail" << std::endl;
    oss << "        invert: invert image (to negative)" << std::endl;
    oss << "        load=<filename>: load image from file instead of taking an image on stdin; the main meaningful use would be in association with mask filter" << std::endl;
    oss << "                         supported file types by filename extension:" << std::endl;
    oss << "                             - .bin: file is in cv-cat binary format: <t>,<rows>,<cols>,<type>,<image data>" << std::endl;
    oss << "                             - otherwise whatever cv::imread supports" << std::endl;
    oss << "        log=<options>: write images to files" << std::endl;
    oss << "            log=<filename>: write images to a single file" << std::endl;
    oss << "            log=<dirname>,size:<number of frames>: write images to files in a given directory, each file (except possibly the last one) containing <number of frames> frames" << std::endl;
    oss << "            log=<dirname>,period:<seconds>: write images to files in a given directory, each file containing frames for a given period of time" << std::endl;
    oss << "                                            e.g. for log=tmp,period:1.5 each file will contain 1.5 seconds worth of images" << std::endl;
    oss << "            log=<options>,index: write index file, describing file number and offset of each frame" << std::endl;
    oss << "        magnitude: calculate magnitude for a 2-channel image; see cv::magnitude() for details" << std::endl;
    oss << "        mask=<mask>: apply mask to image (see cv::copyTo for details)" << std::endl;
    oss << "                     <mask>: any sequence of cv-cat filters that outputs a single-channel image of the same dimensions as the images on stdin" << std::endl;
    oss << "                             the separator between the filters is '|'" << std::endl;
    oss << "                             the equal sign for a filter is ':'" << std::endl;
    oss << "                     examples" << std::endl;
    oss << "                         apply a constant mask from a file" << std::endl;
    oss << "                             cat images.bin | cv-cat 'mask=load:mask.bin' > masked.bin" << std::endl;
    oss << "                             cat images.bin | cv-cat 'mask=load:mask.png' > masked.bin" << std::endl;
    oss << "                         extract pixels brighter than 100" << std::endl;
    oss << "                             cat images.bin | cv-cat 'mask=convert-color:BGR,GRAY|threshold=otsu,100' > masked.bin" << std::endl;
    oss << "        map=<map file>[&<csv options>][&permissive]: map integer values to floating point values read from the map file" << std::endl;
    oss << "             <csv options>: usual csv options for map file, but &-separated (running out of separator characters)" << std::endl;
    oss << "                  fields: key,value; default: value" << std::endl;
    oss << "                  default: read a single column of floating point values (with the row counter starting from zero used as key)" << std::endl;
    oss << "             <permissive>: if present, integer values in the input are simply copied to the output unless they are in the map" << std::endl;
    oss << "                  default: filter fails with an error message if it encounters an integer value which is not in the map" << std::endl;
    oss << "             example: \"map=map.bin&fields=,key,value&binary=2ui,d\"" << std::endl;
    oss << "        merge=<n>: split an image into n horizontal bands of equal height and merge them into an n-channel image (the number of rows must be a multiple of n)" << std::endl;
    oss << "        normalize=<how>: normalize image and scale to 0 to 1 float (or double if input is CV_64F)" << std::endl;
    oss << "            normalize=max: normalize each pixel channel by its max value" << std::endl;
    oss << "            normalize=sum: normalize each pixel channel by the sum of all channels" << std::endl;
    oss << "            normalize=all: normalize each pixel by max of all channels (see cv::normalize with NORM_INF)" << std::endl;
    oss << "        null: same as linux /dev/null (since windows does not have it)" << std::endl;
    oss << "        overlay=<image_file>[,x,y]: overlay image_file on top of current stream at optional x,y location; overlay image should have alpha channel" << std::endl;
    oss << "        resize=<factor>[,<interpolation>]; resize=<width>,<height>[,<interpolation>]" << std::endl;
    oss << "            <interpolation>: nearest, linear, area, cubic, lanczos4; default: linear" << std::endl;
    oss << "                             in format <width>,<height>,<interpolation> corresponding numeric values can be used: " << cv::INTER_NEAREST << ", " << cv::INTER_LINEAR << ", " << cv::INTER_AREA << ", " << cv::INTER_CUBIC << ", " << cv::INTER_LANCZOS4 << std::endl;
    oss << "            examples" << std::endl;
    oss << "                resize=0.5,1024 : 50% of width; heigth 1024 pixels" << std::endl;
    oss << "                resize=512,1024,cubic : resize to 512x1024 pixels, cubic interpolation" << std::endl;
    oss << "                resize=0.2,0.4 : resize to 20% of width and 40% of height" << std::endl;
    oss << "                resize=0.5 : resize proportionally to 50%" << std::endl;
    oss << "                resize=0.5,1024 : 50% of width; heigth 1024 pixels" << std::endl;
    oss << "            note: if no decimal dot '.', size is in pixels; if decimal dot present, size as a fraction" << std::endl;
    oss << "                  i.e. 5 means 5 pixels; 5.0 means 5 times" << std::endl;
    oss << "        remove-mean=<kernel_size>,<ratio>: simple high-pass filter removing <ratio> times the mean component on <kernel_size> scale" << std::endl;
    oss << "        split: split n-channel image into a nx1 grey-scale image" << std::endl;
    oss << "        text=<text>[,x,y][,colour]: print text; default x,y: 10,10; default colour: yellow" << std::endl;
    oss << "        threshold=<threshold|otsu>[,<maxval>[,<type>]]: threshold image; same semantics as cv::threshold()" << std::endl;
    oss << "            <threshold|otsu>: threshold value; if 'otsu' then the optimum threshold value using the Otsu's algorithm is used (only for 8-bit images)" << std::endl;
    oss << "            <maxval>: maximum value to use with the binary and binary_inv thresholding types (default:255)" << std::endl;
    oss << "            <type>: binary, binary_inv, trunc, tozero, tozero_inv (default:binary)" << std::endl;
    oss << "        thumb[=<cols>[,<wait-interval>]]: view resized image; a convenience for debugging and filter pipeline monitoring" << std::endl;
    oss << "                                          <cols>: image width in pixels; default: 100" << std::endl;
    oss << "                                          <wait-interval>: a hack for now; milliseconds to wait for image display and key press; default: 1" << std::endl;
    oss << "        timestamp: write timestamp on images" << std::endl;
    oss << "        transpose: transpose the image (swap rows and columns)" << std::endl;
    oss << "        undistort=<undistort map file>: undistort" << std::endl;
    oss << "        unpack12: convert from 12-bit packed (2 pixels in 3 bytes) to 16UC1; use before other filters" << std::endl;
    oss << "        view[=<wait-interval>]: view image; press <space> to save image (timestamp or system time as filename); <esc>: to close" << std::endl;
    oss << "                                <wait-interval>: a hack for now; milliseconds to wait for image display and key press; default 1" << std::endl;
    oss << std::endl;
    oss << "    operations on subsets of columns, rows, or channels:" << std::endl;
    oss << "        bands-to-cols=x,w[,x,w][,method:<method-name>,output-depth:<depth>]; take a number of columns (bands) from the input, process together by method," << std::endl;
    oss << "            write into the output, one band (a range of columns) reduced into one column; supported methods: average (default), sum, min, max; the sum method" << std::endl;
    oss << "            requires the explicit output-depth parameter (one of i,f,d or CV_32S, CV_32F, CV_64F) if the input has low depth" << std::endl;
    oss << "            examples: \"bands-to-cols=12,23,50,30,45,60,100,1,method:average\"; output an image of 4 columns containing the average" << std::endl;
    oss << "                          of columns 12-34 (i.e., 12 + 23 - 1), 50-79, 45-104 (overlap is OK), and 100 (width is 1) from the original image" << std::endl;
    oss << "                      \"bands-to-cols=12,23,50,30,45,60,100,1,method:sum,output-depth:d\"; same bands but output an image of 4 columns containing the sum" << std::endl;
    oss << "                          of the columns data from the original image; use CV_64F depth (double) as the output format" << std::endl;
    oss << "        bands-to-rows=x,w[,x,w][,method:<method-name>,output-depth:<depth>]; same as bands-to-cols but operate on rows of input instead of columns" << std::endl;
    oss << std::endl;
    oss << "        channels-to-cols; opposite to cols-to-channels; unwrap all channels as columns" << std::endl;
    oss << "            example: \"channels-to-cols\" over a 3-channel image: RGB channels of column 0 become columns 0 (single-channel), 1, and 2, RGB channels" << std::endl;
    oss << "            of column 1 become columns 3,4,5, and so on" << std::endl;
    oss << "        channels-to-rows; same as channels-to-cols but operates over rows" << std::endl;
    oss << std::endl;
    oss << "        cols-to-channels=1,4,5[|pad:value|repeat:step]; opposite to channels-to-cols; stores the listed columns as channels in the output file" << std::endl;
    oss << "            input shall be a single-channel stream; up to 4 channels are supported; if 1, 3, or 4 columns are specified, the output would have 1, 3, or 4 channels respectively" << std::endl;
    oss << "            in case of 2 columns, a third empty (zero) channel is added; use the \"pad:value\" option to specify the fill value other then zero" << std::endl;
    oss << "            the repeat option applies the transformation periodically, first for the specified columns, then for columns incremented by one step, and so on; see the examples" << std::endl;
    oss << "            examples: \"cols-to-channels=6,4|pad:128\"; put column 6 into the R channel, column 4 into the G channel, and fill the B channel with 128" << std::endl;
    oss << "                      \"cols-to-channels=0,1,2|repeat:3\"; store columns 0,1,2 as RGB channels of column 0 of the output file, then columns 3,4,5 as RGB" << std::endl;
    oss << "                      channels of column 1 of the output file, etc.; conversion stops when all input column indices exceed the image width" << std::endl;
    oss << "                      if one of the input columns exceed the image width, the respective output channel is filled with zeros (or the padding value)" << std::endl;
    oss << std::endl;
    oss << "        crop-cols=<x>[,<w>[|<x>[,<w>,...: output an image consisting of (multiple) columns starting at x with width w" << std::endl;
    oss << "            examples: \"crop-cols=2,10|12,10\"; output an image of width 20 taking 2 width-10 columns starting at 2 and 12" << std::endl;
    oss << "                      \"crop-cols=2|12,10\"; default width is 1, the output image has width 11" << std::endl;
    oss << "        crop-rows=<y>[,<h>[|<y>[,<h>,...: output an image consisting of (multiple) row blocks starting at y with height h" << std::endl;
    oss << "            examples: \"crop-rows=5,10|25,5\"; output an image of height 15 taking 2 row blocks starting at 5 and 25 and with heights 10 and 5, respectively" << std::endl;
    oss << "                      \"crop-rows=5|25|15\"; default block height is 1, block starts can be out of order" << std::endl;
    oss << std::endl;
    oss << "        rows-to-channels=1,4,5[|pad:value|repeat:step]; same as cols-to-channels but operates on rows" << std::endl;
    oss << std::endl;
    oss << "        swap-channels=2,1,0,3; re-order channels; arguments shall be integers from 0 to the total number of input channels" << std::endl;
    oss << "            NYI - for now a placeholder only, possibly can be achieved by other operations" << std::endl;
    oss << "            the number of arguments shall be the same as the number of input channels" << std::endl;
    oss << "            example: \"swap-channels=2,1,0\"; revert the order of RGB channels with R becoming B and B becoming R; G is mapped onto itself" << std::endl;
    oss << std::endl;
    oss << "    operations combining the data of multiple channels:" << std::endl;
    oss << "        ratio=(<a1>r + <a2>g + ... + <ac>)/(<b1>r + <b2>g + ... + <bc>): output grey-scale image that is a ratio of linear combinations of input channels" << std::endl;
    oss << "            with given coefficients and offsets; see below for examples, use '--help filters::ratio' for the detailed explanation of the syntax and examples" << std::endl;
    oss << "            the naming convention does not depend on the actual image channels: 'r' in the ratio expression is always interpreted as channel[0]," << std::endl;
    oss << "            'g' as channel[1], etc.; in particular, grey-scaled images have a single channel that shall be referred to as 'r', e.g., ratio=r / ( r + 1 )" << std::endl;
    oss << "            examples: \"ratio=( r + g + b ) / ( 1 + a )\"; output a grey-scale image equal to the sum of the first 3 channels" << std::endl;
    oss << "                          divided by the offset 4th channel" << std::endl;
    oss << "                      \"ratio=( r - b ) / ( r + b )\"; output normalized difference of channels 'r' and 'g'" << std::endl;
    oss << "        linear-combination=<a1>r + <a2>g + ... + <ac>: output grey-scale image that is linear combination of input channels with given coefficients and optional offset" << std::endl;
    oss << "            example: \"linear-combination=-r+2g-b\", highlights the green channel" << std::endl;
    oss << "            naming conventions are the same as for the ratio operation; use '--help filters::linear-combination' for more examples and a detailed syntax explanation" << std::endl;
    oss << "        output of the ratio and linear-combination operations has floating point (CV_32F) precision unless the input is already in doubles (if so, precision is unchanged)" << std::endl;
    oss << std::endl;
    oss << "    basic drawing on images" << std::endl;
    oss << "        cross[=<x>,<y>]: draw cross-hair at x,y; default: at image center" << std::endl;
    oss << "        circle=<x>,<y>,<radius>[,<r>,<g>,<b>,<thickness>,<line_type>,<shift>]: draw circle; see cv::circle for details on parameters and defaults" << std::endl;
    oss << "        rectangle,box=<x>,<y>,<x>,<y>[,<r>,<g>,<b>,<thickness>,<line_type>,<shift>]: draw rectangle; see cv::rectangle for details on parameters and defaults" << std::endl;
    oss << std::endl;
    oss << "    cv::Mat image operations:" << std::endl;
    oss << "        histogram: calculate image histogram and output in binary format: t,3ui,256ui for ub images; for 3ub images as b,g,r: t,3ui,256ui,256ui,256ui, etc" << std::endl;
    oss << "        simple-blob[=<parameters>]: wraps cv::SimpleBlobDetector, outputs as csv key points timestamped by image timestamp" << std::endl;
    oss << "            <parameters>" << std::endl;
    oss << "                output-binary: output key points as binary" << std::endl;
    oss << "                output-fields: print output fields on stdout and exit" << std::endl;
    oss << "                output-format: print binary output format on stdout and exit" << std::endl;
    oss << "                output-default-params,default-params: print default simple blob detector parameters to stdout and exit" << std::endl;
    oss << "                <filename>[:<path>]: simple blob detector params config file and optionally cv file node path in it" << std::endl;
    oss << "                                     e.g: my-params.txt:blob" << std::endl;
    oss << "                                     if not present, defaults will be used" << std::endl;
    oss << std::endl;
    oss << snark::imaging::vegetation::filters::usage() << std::endl;
    return oss.str();
}

const std::string& filters::usage( const std::string & operation )
{
    if ( operation.empty() )
    {
        static const std::string s = usage_impl_();
        return s;
    }
    else
    {
        if ( operation == "ratio" || operation == "linear-combination" )
        {
            static const std::string s = snark::cv_mat::ratios::ratio::describe_syntax();
            return s;
        }
        else
        {
            static std::string s = "filters: no specific help is available for the '" + operation + "' operation";
            return s;
        }
    }
}

} } // namespace snark{ namespace cv_mat {

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

template <> struct traits< snark::cv_mat::log_impl_::logger::indexer >
{
    template < typename K, typename V > static void visit( const K&, const snark::cv_mat::log_impl_::logger::indexer& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "file", t.file );
        v.apply( "offset", t.offset );
    }

    template < typename K, typename V > static void visit( const K&, snark::cv_mat::log_impl_::logger::indexer& t, V& v )
    {
        v.apply( "t", t.t );
        v.apply( "file", t.file );
        v.apply( "offset", t.offset );
    }
};

} } // namespace comma { namespace visiting {
