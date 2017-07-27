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

#include "accumulated.h"

#include <comma/base/exception.h>
#include <vector>
#include <functional>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/bind.hpp>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include "../depth_traits.h"

// todo:
// - remove unused includes
// - move mat_iterator to accumulated.cpp
// - replace bind directly with lambda functions to pass to mat_iterator
// - sliding_average: try to make it non-template; try to avoid bind

namespace snark{ namespace cv_mat {  namespace accumulated {
    
namespace impl {
    
// Row and Col tells you which pixel to access or was accessed to get input nd result pixel
// The returned value will also be set into this pixel in 'result' cv::Mat 
// 'count' is the image number being accessed
typedef std::function< float( float input_value, float result_value, comma::uint32 count, unsigned int row, unsigned int col ) > apply_function;
    
template< int DepthIn >
static void iterate_pixels( const tbb::blocked_range< std::size_t >& r, const cv::Mat& m, cv::Mat& result, const apply_function& fn, comma::uint64 count )
{
    typedef typename depth_traits< DepthIn >::value_t value_in_t;
    typedef float value_out_t;
    const unsigned int channels = m.channels();
    const unsigned int cols = m.cols * channels;
    for( unsigned int i = r.begin(); i < r.end(); ++i )
    {
        const value_in_t* in = m.ptr< value_in_t >(i);
        auto* ret = result.ptr< value_out_t >(i);
        for( unsigned int j = 0; j < cols; ++j ) 
        {
            *ret = fn( *in, *ret, count, i, j);
            ++ret;
            ++in;
        }
    }
}

template< typename H, int DepthIn >
static void divide_by_rows( const cv::Mat& m, cv::Mat& result, const apply_function& fn, comma::uint64 count )
{
    tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, m.rows ), 
                       boost::bind( &iterate_pixels< DepthIn >, _1, m, boost::ref( result ), boost::ref(fn), count ) );
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

} // namespace impl {

template < typename H >
ema< H >::ema( float alpha, comma::uint32 spin_up_size ) : count_(0), alpha_(alpha), spin_up_(spin_up_size)
{
    if( spin_up_ == 0 ) { COMMA_THROW(comma::exception, "accumulated=ema: error please specify spin up value greater than 0"); }
    if( alpha_ <= 0 || alpha_ >= 1.0 ) { COMMA_THROW(comma::exception, "accumulated: please specify ema alpha in the range 0 < alpha < 1.0, got " << alpha_ ); }
}

template < typename H >
typename average< H >::value_type average< H >::operator()( const typename average< H >::value_type& n )
{
    ++count_;
    // This filter is not run in parallel, no locking required
    if( result_.empty() ) { result_ = cv::Mat::zeros( n.second.rows, n.second.cols, CV_MAKETYPE(CV_32F, n.second.channels()) ); }
    
    impl::iterate_by_input_type< H >(n.second, result_, [](float in, float avg, comma::uint64 count, comma::uint32 row, comma::uint32 col) { return avg + (in - avg)/count; } , count_);
    
    cv::Mat output; // copy as result_ will be changed next iteration
    result_.convertTo(output, n.second.type());
    return value_type(n.first, output); 
}

template < typename H >
typename ema< H >::value_type ema< H >::operator()( const typename ema< H >::value_type& n )
{
    ++count_;
    // This filter is not run in parallel, no locking required
    if( result_.empty() ) { result_ = cv::Mat::zeros( n.second.rows, n.second.cols, CV_MAKETYPE(CV_32F, n.second.channels()) ); }
    
    auto ema_functor = [this](float in, float avg, comma::uint64 count, comma::uint32 row, comma::uint32 col) {
        // Spin up initial value for EMA
        if( count <= spin_up_ ) { return (avg + (in - avg)/count); } else {  return avg + (in - avg) * alpha_; }
    };
    impl::iterate_by_input_type< H >(n.second, result_, ema_functor, count_);
    
    cv::Mat output; // copy as result_ will be changed next iteration
    result_.convertTo(output, n.second.type());
    return value_type(n.first, output); 
}

template < int DepthIn >
static float sliding_average( float in, float avg, comma::uint64 count,
                              comma::uint32 row, comma::uint32 col,
                              const std::deque< cv::Mat >& window, comma::uint32 size)
{
    typedef typename depth_traits< DepthIn >::value_t value_in_t;
    // Haven't reach the window size yet
    if( window.size() < size ) { return (avg + (in - avg)/count); }
    else
    {
        const auto* back_of_window = window.front().ptr< value_in_t >(row);
        value_in_t val = *(back_of_window + col);
        return avg + (in - val)/float(size);
    }
}

template < typename H >
moving_average< H >::moving_average( comma::uint32 size ) : count_(0), size_(size) {}

template < typename H >
typename moving_average< H >::value_type moving_average< H >::operator()( const typename moving_average< H >::value_type& n )
{
    ++count_;
    // This filter is not run in parallel, no locking required
    if( count_ == 1 ) { n.second.convertTo( result_, CV_MAKETYPE(CV_32F, n.second.channels()) ); }
    else 
    { 
        int depth = n.second.depth();
        auto& window = window_;
        auto size = size_;
        impl::iterate_by_input_type< H >(n.second, result_, 
            [&window, size, depth](float new_value, float avg, comma::uint64 count, comma::uint32 row, comma::uint32 col) -> float 
            {
                switch( depth )
                {
                    case CV_8U : return sliding_average< CV_8U > ( new_value, avg, count, row, col, window, size );
                    case CV_8S : return sliding_average< CV_8S > ( new_value, avg, count, row, col, window, size );
                    case CV_16U: return sliding_average< CV_16U >( new_value, avg, count, row, col, window, size );
                    case CV_16S: return sliding_average< CV_16S >( new_value, avg, count, row, col, window, size );
                    case CV_32S: return sliding_average< CV_32S >( new_value, avg, count, row, col, window, size );
                    case CV_32F: return sliding_average< CV_32F >( new_value, avg, count, row, col, window, size );
                    default: return     sliding_average< CV_64F >( new_value, avg, count, row, col, window, size );
                }
            },
            count_); 
    }
    
    // update sliding window
    if( window_.size() >= size_ ) { window_.pop_front(); }
    window_.push_back( cv::Mat() );
    n.second.copyTo( window_.back() );      // have to copy, next filter will change it
    
    cv::Mat output; // copy as result_ will be changed next iteration
    result_.convertTo(output, n.second.type());
    return value_type(n.first, output); 
}

} } }  // namespace snark { namespace cv_mat { namespace accumulated {

template class snark::cv_mat::accumulated::average< boost::posix_time::ptime >;
template class snark::cv_mat::accumulated::average< std::vector< char > >;
template class snark::cv_mat::accumulated::ema< boost::posix_time::ptime >;
template class snark::cv_mat::accumulated::ema< std::vector< char > >;
template class snark::cv_mat::accumulated::moving_average< boost::posix_time::ptime >;
template class snark::cv_mat::accumulated::moving_average< std::vector< char > >;