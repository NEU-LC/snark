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

#include <functional>
#include <map>
#include <string>
#include <vector>
#include <comma/base/types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark{ namespace cv_mat { namespace morphology {
    
const std::map< std::string, int >& operations();

struct parameters
{
    parameters( const std::vector< std::string >& e );
    
    cv::Mat kernel;
    comma::uint32 iterations;
};


template < typename H >
typename std::pair< H, cv::Mat > morphology( const typename std::pair< H, cv::Mat > m, int op, const cv::Mat & element, comma::uint32 iterations )
{
    typename std::pair< H, cv::Mat > result( m.first, cv::Mat() );
    cv::morphologyEx( m.second, result.second, op, element, cv::Point(-1,-1), iterations );
    return result;
}

template < typename H >
class skeleton
{
    public:
        typedef std::pair< H, cv::Mat > value_type;
        value_type value;

        // Set load as "bin" to load cv-cat format, else load using cv::imread
        skeleton( const parameters& param );

        value_type operator()( value_type );
        
    private:
        cv::Mat kernel_;
        int iterations_;
};

template < typename H >
class advance
{
    public:
        typedef std::pair< H, cv::Mat > value_type;

        advance( const parameters& param, bool advance = true, int background = 0 );

        value_type operator()( value_type );
        
        static advance make( const std::vector< std::string >& options );
        
    private:
        int background_;
        std::vector< std::pair< float, cv::Point > > offsets_; // quick and dirty
        std::function< void( unsigned char*, const unsigned char* ) > set_;
        std::function< bool( unsigned char*, const unsigned char*, const unsigned char*, const unsigned char*, unsigned int ) > visit_neighbour_;
};

template < typename H >
class meet
{
    public:
        typedef std::pair< H, cv::Mat > value_type;

        meet( const parameters& param );

        value_type operator()( value_type );
        
    private:
        unsigned int iterations_;
        std::vector< std::pair< float, cv::Point > > offsets_;
};

} } }  // namespace snark { namespace cv_mat { namespace impl {
