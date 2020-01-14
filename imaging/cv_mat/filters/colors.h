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

#include <opencv2/core/core.hpp>
#if CV_MAJOR_VERSION > 2
#include <opencv2/xphoto/white_balance.hpp>
#else // CV_MAJOR_VERSION > 2
#include <comma/base/exception.h>
#endif // CV_MAJOR_VERSION > 2

namespace snark{ namespace cv_mat { namespace impl {

#if CV_MAJOR_VERSION > 2
    template < typename H >
    class balance_white
    {
        public:
            balance_white();
            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        private:
            cv::Ptr< cv::xphoto::SimpleWB > wb_;
    };
#else // CV_MAJOR_VERSION > 2
    template < typename H >
    struct balance_white
    {
        balance_white() { COMMA_THROW( comma::exception, "balance-white not implemented for opencv version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << " that you have" ); }
        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m ) { COMMA_THROW( comma::exception, "balance-white not implemented for opencv version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << " that you have" ); }
    };
#endif // CV_MAJOR_VERSION > 2

} } }  // namespace snark { namespace cv_mat { namespace impl {
