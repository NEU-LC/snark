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
#include <iostream>
#include <vector>
#include <map>
#include <boost/bind.hpp>
#include <boost/thread/pthread/pthread_mutex_scoped_lock.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include "../depth_traits.h"
#include "mat_iterator.h"

namespace snark{ namespace cv_mat {  namespace impl {

static float accumulated_average( float in, float avg, comma::uint64 count, comma::uint32 row, comma::uint32 col)
{
    return (avg + (in - avg)/count);
}

// exponential moving average
static float accumulated_ema( float in, float avg, comma::uint64 count, comma::uint32 row, comma::uint32 col, 
                        comma::uint32 window_size, double multiplier)
{
    // Do Simple Moving Average until the count is greater than window size
    if( count <= window_size ) { return (avg + (in - avg)/count); } else {  return avg + (in - avg) * multiplier; }
}

template < typename H >
accumulated< H >::accumulated( boost::optional< comma::uint32 > window_size ) 
    : count_(0), type_(accumulated_type::average)
{
    if( window_size ) 
    {
        if( *window_size < 2 ) { COMMA_THROW(comma::exception, "accumulated: window size for Exponential Moving Average must be >= 2, got " << *window_size ) }
        type_ = accumulated_type::exponential_moving_average;
        double multiplier = 2.0 / ( *window_size + 1 );    
        
        average_ema_ = boost::bind( &accumulated_ema, _1, _2, _3, _4, _5, *window_size, multiplier);
    }
}

template < typename H >
typename accumulated< H >::value_type accumulated< H >::operator()( const typename accumulated< H >::value_type& n )
{
    ++count_;
    // This filter is not run in parallel, no locking required
    if( result_.empty() ) { result_ = cv::Mat::zeros( n.second.rows, n.second.cols, CV_MAKETYPE(CV_32F, n.second.channels()) ); }
    
    switch (type_)  // call to parallel by row ranges
    {
        case accumulated_type::average: iterate_by_input_type< H >(n.second, result_, &accumulated_average, count_); break;
        case accumulated_type::exponential_moving_average: iterate_by_input_type< H >(n.second, result_, average_ema_, count_); break;
    }
    
    cv::Mat output; // copy as result_ will be changed next iteration
    result_.convertTo(output, n.second.type());
    return value_type(n.first, output); 
}

template < typename H >
sliding_window< H >::sliding_window( comma::uint32 size ) : count_(0), size_(size) {}

template < typename H >
typename sliding_window< H >::value_type sliding_window< H >::operator()( const typename sliding_window< H >::value_type& n )
{
    ++count_;
    // This filter is not run in parallel, no locking required
    if( result_.empty() ) { result_ = cv::Mat::zeros( n.second.rows, n.second.cols, CV_MAKETYPE(CV_32F, n.second.channels()) ); }
    
    if( count_ == 1 ) { n.second.convertTo( result_, result_.type() ); }
    else if( window_.size() < size_ ) // window size is not reached
    { 
        // only when input type is 32F
        // Formula is below, only works if input data type is also floats 
        if( n.second.depth() == result_.depth() ) { result_ += (n.second - result_)/float(count_); }
        else 
        {
            cv::Mat temp;
            cv::subtract( n.second, result_, temp, cv::noArray(), result_.type() );
            result_ += temp/float(count_);
        }
    }
    else 
    {
        // remove data from front of window, add in data from this image
        // Formula is below, only works if input data type is also floats 
        if( n.second.depth() == result_.depth() ) { result_ += ( (n.second - window_.front()) / float(size_)); }
        else 
        {
            cv::Mat temp;
            cv::subtract( n.second, window_.front(), temp, cv::noArray(), result_.type() );
            result_ += temp / float(size_);
        }
    }
    
    // update sliding window
    if( window_.size() >= size_ ) { window_.pop_front(); }
    window_.push_back( cv::Mat() );
    n.second.copyTo( window_.back() );      // have to copy, next filter will change it
    
    cv::Mat output; // copy as result_ will be changed next iteration
    result_.convertTo(output, n.second.type());
    return value_type(n.first, output); 
}

} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::impl::accumulated< boost::posix_time::ptime >;
template class snark::cv_mat::impl::accumulated< std::vector< char > >;
template class snark::cv_mat::impl::sliding_window< boost::posix_time::ptime >;
template class snark::cv_mat::impl::sliding_window< std::vector< char > >;