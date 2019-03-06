// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
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
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include "../serialization.h"

namespace snark { namespace cv_mat { namespace impl {
    
template < typename H >
class file
{
    public:
        typedef boost::function< boost::posix_time::ptime( const H& ) > get_timestamp_functor;

        file( const get_timestamp_functor& get_timestamp, const std::string& type, bool no_header, const boost::optional< int >& quality, bool do_index, const std::vector< std::string >& filenames = std::vector< std::string >() );

        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        
    private:
        const get_timestamp_functor get_timestamp_;
        std::string type_;
        boost::optional< int > quality_;
        bool do_index_;
        snark::cv_mat::serialization serialization_;
        boost::posix_time::ptime previous_timestamp_;
        unsigned int index_;
        std::vector< std::string > filenames_;
        unsigned int count_;
        std::string make_filename_( const boost::posix_time::ptime& t );
};

} } } // namespace snark { namespace cv_mat { namespace impl {
