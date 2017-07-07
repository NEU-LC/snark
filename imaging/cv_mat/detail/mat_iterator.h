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

#pragma once

#include <vector>
#include <map>
#include <boost/bind.hpp>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <opencv2/core/core.hpp>
#include <boost/function.hpp>

namespace snark{ namespace cv_mat {  namespace impl {
    
typedef boost::function< double( double , double , double ) > apply_function;
    
template< int DepthIn >
static void iterate_pixels( const tbb::blocked_range< std::size_t >& r, const cv::Mat& m, cv::Mat& result, const apply_function& fn, comma::uint64 count )
{
    typedef typename depth_traits< DepthIn >::value_t value_in_t;
    typedef double value_out_t;
    const unsigned int channels = m.channels();
    const unsigned int cols = m.cols * channels;
    for( unsigned int i = r.begin(); i < r.end(); ++i )
    {
        const value_in_t* in = m.ptr< value_in_t >(i);
        auto* ret = result.ptr< value_out_t >(i);
        for( unsigned int j = 0; j < cols; ++j ) 
        {
            *ret = fn( *in, *ret, count);
            ++ret;
            ++in;
        }
    }
}

template< typename H, int DepthIn >
static void divide_by_rows( const cv::Mat& m, cv::Mat& result, const apply_function& fn, comma::uint64 count )
{
    tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, m.rows ), 
                       boost::bind( &iterate_pixels< DepthIn >, _1, m, boost::ref( result ), fn, count ) );
}

template< typename H >
static void iterate_by_input_type( const cv::Mat& m, cv::Mat& result, const apply_function& fn, comma::uint64 count)
{
    int otype = m.depth(); // This will work?
    switch( otype )
    {
        case CV_8U : divide_by_rows< H, CV_8U  >( m, result, fn, count ); break;
        case CV_8S : divide_by_rows< H, CV_8S  >( m, result, fn, count ); break;
        case CV_16U: divide_by_rows< H, CV_16U >( m, result, fn, count ); break;
        case CV_16S: divide_by_rows< H, CV_16S >( m, result, fn, count ); break;
        case CV_32S: divide_by_rows< H, CV_32S >( m, result, fn, count ); break;
        case CV_32F: divide_by_rows< H, CV_32F >( m, result, fn, count ); break;
        case CV_64F: divide_by_rows< H, CV_64F >( m, result, fn, count ); break;
        default: COMMA_THROW( comma::exception, "accumulated: unrecognised output image type " << otype );
    }
}

} } } // namespace snark{ namespace cv_mat {  namespace impl {