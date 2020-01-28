// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

/// @author vsevolod vlaskine

#pragma once

#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/base/types.h>

namespace snark { namespace cv_mat { namespace filters { namespace partitions {

template < typename H >
class partition {
    public:
        explicit partition(boost::optional< cv::Scalar > do_not_visit_value = boost::optional< cv::Scalar >(),
                           comma::int32 none = -1, bool merge = false, bool keep_id = false,
                           comma::int32 start_from = 0, unsigned int min_partition_size = 0, unsigned int degrees = 8);

        std::pair< H, cv::Mat > operator()(std::pair< H, cv::Mat > m);

        typedef boost::function< std::pair< H, cv::Mat >(std::pair< H, cv::Mat >) > functor_t;

        /// return partition functor and boolean flag that indicates whether functor is safely re-entrant in multithread context
        /// functor is re-entrant, if keep_id is set to false
        /// @todo? protect with mutex instead?
        static std::pair< functor_t, bool > make(const std::string &options);

        static std::string usage(unsigned int indent = 0);

    private:
        boost::optional< cv::Scalar > do_not_visit_value_;
        comma::int32 none_;
        bool merge_;
        bool keep_id_;
        comma::int32 start_from_;
        comma::int32 id_;
        unsigned int min_partition_size_;
        unsigned int degrees_;
};

template < typename H >
class reduce {
    public:
        explicit reduce(unsigned int colours = 6, unsigned int channel = 0, comma::int32 background = -1,
                        bool merge = false) : colours_(colours), channel_(channel), background_(background),
                                              merge_(merge) {};

        std::pair< H, cv::Mat > operator()(std::pair< H, cv::Mat > m);

        typedef boost::function< std::pair< H, cv::Mat >(std::pair< H, cv::Mat >) > functor_t;

        static std::pair< functor_t, bool > make(const std::string &options);

        static std::string usage(unsigned int indent = 0);

    private:
        unsigned int colours_;
        unsigned int channel_;
        comma::int32 background_;
        bool merge_;

        template < typename T, int I >
        cv::Mat process_(cv::Mat m, int type);
};

}}}}  // namespace snark { namespace cv_mat { namespace impl { namespace partitions {
