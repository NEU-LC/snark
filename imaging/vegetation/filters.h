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

#include <boost/optional.hpp>
#include <comma/string/split.h>
#include "../cv_mat/filters.h"

namespace snark { namespace imaging { namespace vegetation {

/// filter pipeline helpers
namespace impl {
    
template < typename H=boost::posix_time::ptime >
struct filters
{
    typedef cv_mat::operation< cv::Mat, H > filter;
    typedef typename cv_mat::impl::filters< H >::value_type value_type;
    /// take name-value string, return filter
    static boost::optional< filter > make( const std::string& what );
    
    /// take name-value string, return functor
    static boost::function< value_type( value_type ) > make_functor( const std::string& v, char equal_sign = '=' ) { return make_functor( comma::split( v, equal_sign ) ); }
    
    /// take name-value vector, return functor
    static boost::function< value_type( value_type ) > make_functor( const std::vector< std::string >& e );
    
    /// return usage for all filters
    static const std::string& usage();
};

} // namespace impl {

typedef impl::filters< > filters;
    
} } }  // namespace snark { namespace imaging { namespace vegetation {
