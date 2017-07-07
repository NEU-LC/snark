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
    
accumulated_type accumulated_type_to_str(const std::__cxx11::string& s)
{
    if( s == "average" ) { return accumulated_type::average; }
    else if( s == "max" ) { return accumulated_type::max; }
    else if( s == "min" ) { return accumulated_type::min; }
    else { COMMA_THROW( comma::exception, "accumulated=" << s << ", unknown accumulated operation"); }
}

static double accumulated_average( double in, double avg, comma::uint64 count)
{
    return (avg + (in - avg)/count);
}

static double accumulated_max( double in, double max, comma::uint64 count)
{
    return std::max(in, max);
}
static double accumulated_min( double in, double min, comma::uint64 count)
{
    return std::min(in, min);
}
    
template < typename H >
typename average_impl_< H >::value_type average_impl_< H >::operator()( const typename average_impl_< H >::value_type& n )
{
    return n;
}

template < typename H >
typename accumulated_impl_< H >::value_type accumulated_impl_< H >::operator()( const typename accumulated_impl_< H >::value_type& n )
{
    ++count_;
    if( result_.size() == cv::Size(0,0) ) 
    {  // This filter is not run in parallel, no locking required
        result_.create( n.second.rows, n.second.cols, CV_MAKETYPE(CV_64F, n.second.channels()) );
        
        switch (type_)
        {
            case accumulated_type::max: result_.setTo(std::numeric_limits< double >::min()); break;
            case accumulated_type::min: result_.setTo(std::numeric_limits< double >::max()); break;
            default: break;
        }
    }
    
    switch (type_)
    {
        case accumulated_type::average: iterate_by_input_type< H >(n.second, result_, &accumulated_average, count_); break;
        case accumulated_type::max: iterate_by_input_type< H >(n.second, result_, &accumulated_max, count_); break;
        case accumulated_type::min: iterate_by_input_type< H >(n.second, result_, &accumulated_min, count_); break;
    }
    
    cv::Mat result;
    result_.convertTo(result, n.second.type());
    return value_type( n.first, result );
}

} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::impl::average_impl_< boost::posix_time::ptime >;
template class snark::cv_mat::impl::average_impl_< std::vector< char > >;
template class snark::cv_mat::impl::accumulated_impl_< boost::posix_time::ptime >;
template class snark::cv_mat::impl::accumulated_impl_< std::vector< char > >;