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

#include <string>
#include <deque>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <comma/base/types.h>

namespace snark{ namespace cv_mat { namespace impl {
    
enum class accumulated_type { average, max, min };

accumulated_type accumulated_type_to_str( const std::string& s );

template < typename H >
struct accumulated_impl_ {
    typedef std::pair< H, cv::Mat > value_type;
    accumulated_type type_;
    comma::uint64 count_;
    cv::Mat result_;
    
    accumulated_impl_( accumulated_type type ) : type_(type), count_(0) {}

    value_type operator()( const value_type& n );
};

template < typename H >
struct sliding_window_impl_ {
    typedef std::pair< H, cv::Mat > value_type;
    accumulated_type type_;
    comma::uint64 count_;
    cv::Mat result_;
    comma::uint32 size_;  // sliding window size
    std::deque< cv::Mat > window_;
    
    sliding_window_impl_( accumulated_type type, comma::uint32 size ) : type_(type), count_(0), size_(size) {}

    value_type operator()( const value_type& n );
};

} } }  // namespace snark { namespace cv_mat { namespace impl {
