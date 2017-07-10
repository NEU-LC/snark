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
    
// accumulated_type accumulated_type_to_str(const std::__cxx11::string& s)
// {
//     if( s == "average" ) { return accumulated_type::average; }
//     else if( s == "max" ) { return accumulated_type::max; }
//     else if( s == "min" ) { return accumulated_type::min; }
//     else { COMMA_THROW( comma::exception, "accumulated=" << s << ", unknown accumulated operation"); }
// }

static float accumulated_average( float in, float avg, comma::uint64 count, comma::uint32 row, comma::uint32 col)
{
    return (avg + (in - avg)/count);
}

// exponential moving average
template < typename H >
float accumulated_impl_< H >::accumulated_ema_( float in, float avg, comma::uint64 count, comma::uint32 row, comma::uint32 col)
{
    // Do Simple Moving Average until the count is greater than window size
    if( count_ <= *window_size_ ) { return (avg + (in - avg)/count_); } else {  return avg + (in - avg) * multiplier_; }
}

template < typename H >
accumulated_impl_< H >::accumulated_impl_( boost::optional< comma::uint32 > size, bool output_float_image ) 
    : count_(0), window_size_(size), type_(accumulated_type::average), output_float_(output_float_image)
{
    if( window_size_ ) 
    {
        if( *window_size_ < 2 ) { COMMA_THROW(comma::exception, "accumulated: window size for Exponential Moving Average must be >= 2, got " << *window_size_ ); }
        type_ = accumulated_type::exponential_moving_average;
        multiplier_ = 2.0 / ( *window_size_ + 1 );    
        
        average_ema_ = boost::bind( &accumulated_impl_< H >::accumulated_ema_, this, _1, _2, _3, _4, _5);
    }
}

template < typename H >
typename accumulated_impl_< H >::value_type accumulated_impl_< H >::operator()( const typename accumulated_impl_< H >::value_type& n )
{
    ++count_;
    // This filter is not run in parallel, no locking required
    if( result_.size() == cv::Size(0,0) ) { result_ = cv::Mat::zeros( n.second.rows, n.second.cols, CV_MAKETYPE(CV_32F, n.second.channels()) ); }
    
    switch (type_)
    {
        case accumulated_type::average: iterate_by_input_type< H >(n.second, result_, &accumulated_average, count_); break;
        case accumulated_type::exponential_moving_average: iterate_by_input_type< H >(n.second, result_, average_ema_, count_); break;
    }
    
    if( output_float_ || n.second.depth() == CV_32FC1 ){ return value_type(n.first, result_); }
    else 
    {
        cv::Mat converted;
        result_.convertTo(converted, n.second.type());
        return value_type( n.first, converted );
    }
}

} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::impl::accumulated_impl_< boost::posix_time::ptime >;
template class snark::cv_mat::impl::accumulated_impl_< std::vector< char > >;