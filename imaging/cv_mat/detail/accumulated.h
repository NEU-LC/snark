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
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <comma/base/types.h>

namespace snark{ namespace cv_mat { namespace accumulated {
    
typedef boost::function< float( float input_value, float result_value, comma::uint64 count, unsigned int row, unsigned int col ) > apply_function;

template < typename H >
class average {
public:
    typedef std::pair< H, cv::Mat > value_type;
    
    value_type operator()( const value_type& n );
private:
    comma::uint64 count_;   // How many input images so far
    cv::Mat result_;        // This is a float depth image
};

template < typename H >
class ema {
public:
    typedef std::pair< H, cv::Mat > value_type;
    
    // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
    ema( float alpha, comma::uint32 spin_up_size=1 );
    value_type operator()( const value_type& n );
private:
    comma::uint64 count_;   // How many input images so far
    cv::Mat result_;        // This is a float depth image
    apply_function average_ema_;
};

template < typename H >
struct sliding_window {
    typedef std::pair< H, cv::Mat > value_type;
    comma::uint64 count_;
    cv::Mat result_;
    comma::uint32 size_;  // sliding window size
    std::deque< cv::Mat > window_;
    
    apply_function average_;
    
    sliding_window( comma::uint32 size );
    
    value_type operator()( const value_type& n );
};

} } }  // namespace snark { namespace cv_mat { namespace accumulated {
