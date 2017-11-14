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

#include "arithmetic.h"

#include <string>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include "utils.h"

namespace snark{ namespace cv_mat { namespace impl {
    
template < typename H >
typename arithmetic< H >::operation arithmetic< H >::str_to_operation(const std::string& s)
{
    if( s == "multiply" ) { return operation::multiply; }
    else if( s == "divide" ) { return operation::divide; }
    else if( s == "subtract" ) { return operation::subtract; }
    else if( s == "add" ) { return operation::add; }
    else if( s == "absdiff" ) { return operation::absdiff; }
    else { COMMA_THROW(comma::exception, "unknown arithmetic operation: \"" << s << "\", expected: multiply, add, subtract or divide" ); }
}
template < typename H >
typename std::string arithmetic< H >::operation_to_str(arithmetic< H >::operation op)
{
    switch(op)
    {
        case operation::multiply: return "multiply";
        case operation::divide:   return "divide";
        case operation::subtract: return "subtract";
        case operation::add:      return "add";
        case operation::absdiff:  return "absdiff";
    }
    
    COMMA_THROW(comma::exception, "arithmetic: unknown operation: " << int(op));
}

template < typename H >
arithmetic< H >::arithmetic( operation op ) : operation_(op) {}

template < typename H >
typename arithmetic< H >::value_type arithmetic< H >::operator()( value_type m, boost::function< value_type( value_type ) >& operand ) // have to pass mask by value, since filter functors may change on call
{
    const cv::Mat & rhs = operand( m ).second;
    if ( rhs.channels() != m.second.channels() ) { COMMA_THROW( comma::exception,  operation_to_str(operation_) << ": channel mis-match, input image channels: " << m.second.channels() << ", the operand channels: " << rhs.channels() ); }
    return apply_(m, rhs);
}

template < typename H >
typename arithmetic< H >::value_type arithmetic< H >::apply_(const value_type& m, const cv::Mat& operand )
{
    value_type n( m.first, cv::Mat(m.second.rows, m.second.cols, m.second.type()) );
    switch(operation_)
    {
        case operation::multiply: cv::multiply(m.second, operand, n.second, 1.0, m.second.type() ); break;
        case operation::divide:   cv::divide(m.second, operand, n.second, 1.0, m.second.type() ); break;
        case operation::subtract: cv::subtract(m.second, operand, n.second, cv::noArray(), m.second.type() ); break;
        case operation::add:      cv::add(m.second, operand, n.second, cv::noArray(), m.second.type() );    break;
        case operation::absdiff:  cv::absdiff(m.second, operand, n.second); break;
    }
    
    return n;
}
    
} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::impl::arithmetic< boost::posix_time::ptime >;
template class snark::cv_mat::impl::arithmetic< std::vector< char > >;