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
#include <boost/filesystem/operations.hpp>
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
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "../../timing/timestamped.h"
#include "../../timing/traits.h"
#include "filters.h"
#include "../cv_mat/serialization.h"
#include "../cv_mat/traits.h"
#include "../vegetation/filters.h"

namespace snark { namespace imaging { namespace vegetation {

template < typename T > static void set_ndvi_pixel_( cv::Mat& red, const cv::Mat& nir )
{
    T* r = &red.at< T >( 0, 0 );
    const T* n = &nir.at< T >( 0, 0 );
    const T* end = r + red.cols * red.rows;
    for( ; r != end; ++r, ++n ) // slightly faster perhaps?
    {
        T s = *r + *n;
        *r = s == 0 ? 0: ( ( *n - *r ) / s );
    }
//     for( int i = 0; i < red.cols; ++i ) // watch performance
//     {
//         for( int j = 0; j < red.rows; ++j )
//         {
//             T& r = red.at< T >( i, j );
//             const T& n = nir.at< T >( i, j );
//             T s = r + n;
//             r = s == 0 ? 0: ( ( n - r ) / s );
//         }
//     }
}
    
static cv_mat::filters::value_type ndvi_impl_( cv_mat::filters::value_type m ) // too quick, too dirty?
{
    if( m.second.channels() != 4 ) { std::cerr << "cv vegetation filters: expected 4 channels, got " << m.second.channels() << std::endl; return cv_mat::filters::value_type(); }
    int type;
    switch( m.second.type() )
    {
        case CV_32FC4: type = CV_32FC3; break;
        case CV_8SC4: //type = CV_8SC3; break;
        case CV_8UC4: //type = CV_8UC3; break;
        default: std::cerr << "cv vegetation filters: expected type CV_32FC4; got: " << m.second.type() << std::endl; return cv_mat::filters::value_type();
    }
    cv::Mat split( m.second.rows * m.second.channels(), m.second.cols, cv_mat::single_channel_type( m.second.type() ) ); // todo: check number of channels!
    std::vector< cv::Mat > channels;
    channels.reserve( m.second.channels() );    
    for( unsigned int i = 0; i < static_cast< unsigned int >( m.second.channels() ); ++i )
    {
        channels.push_back( cv::Mat( split, cv::Rect( 0, i * m.second.rows, m.second.cols, m.second.rows ) ) );
    }
    cv::split( m.second, channels ); // watch performance, do we really need to split?
    switch( channels[0].type() )
    {
        case CV_32FC1: set_ndvi_pixel_< float >( channels[0], channels[3] ); break;
        case CV_8SC1: //set_ndvi_pixel_< char >( channels[0], channels[3] ); break;
        case CV_8UC1: //set_ndvi_pixel_< char >( channels[0], channels[3] ); break;
        default: break; // never here
    }
    cv_mat::filters::value_type n;
    n.first = m.first;
    n.second = cv::Mat( m.second.rows, m.second.cols, type );
    //channels.resize( 3 );
    cv::merge( channels, n.second );
    return n;
}

boost::optional< cv_mat::filter > filters::make( const std::string& what )
{
    std::vector< std::string > v = comma::split( what, '=' );
    if( v[0] == "ndvi" )
    { 
        return cv_mat::filter( boost::bind( &ndvi_impl_, _1 ) );
    }
    return boost::none;
}

static std::string usage_impl_()
{
    std::ostringstream oss;
    oss << "    cv::Mat vegetation-specific filters" << std::endl;
    oss << "        ndvi: todo" << std::endl;
    return oss.str();
}

const std::string& filters::usage()
{
    static const std::string s = usage_impl_();
    return s;
}

} } } // namespace snark { namespace imaging { namespace vegetation {
