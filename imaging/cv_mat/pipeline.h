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


#ifndef SNARK_IMAGING_APPLICATIONS_PIPELINE_H_
#define SNARK_IMAGING_APPLICATIONS_PIPELINE_H_

#ifdef WIN32
#include <winsock2.h>
#endif

#include <comma/sync/synchronized.h>
#include "../../tbb/bursty_reader.h"
#include "bursty_pipeline.h"
#include "filters.h"
#include "serialization.h"

namespace snark {

namespace tbb {

template<>
struct bursty_reader_traits< std::pair< boost::posix_time::ptime, cv::Mat > >
{
    static bool valid( const std::pair< boost::posix_time::ptime, cv::Mat >& p ) { return ( !p.second.empty() ); }
};

} // namespace tbb {

namespace imaging { namespace applications {

/// base class for video processing, capture images in a serarate thread, apply filters, serialize to stdout
class pipeline
{
    public:
        typedef std::pair< boost::posix_time::ptime, cv::Mat > pair;
        
        pipeline( cv_mat::serialization& output
                , const std::string& filters
                , tbb::bursty_reader< pair >& reader
                , unsigned int number_of_threads = 0 );
        
        pipeline( cv_mat::serialization& output
                , const std::vector< cv_mat::filter >& filters
                , tbb::bursty_reader< pair >& reader
                , unsigned int number_of_threads = 0 );

        void run();

    protected:
        void write_( pair p );
        void null_( pair p );
        void setup_pipeline_();

        comma::synchronized< cv_mat::serialization* > m_output;
        ::tbb::filter_t< pair, void > m_filter;
        std::vector< cv_mat::filter > m_filters;
        tbb::bursty_reader< pair >& m_reader;
        tbb::bursty_pipeline< pair > m_pipeline;
    };

} } }

#endif // SNARK_IMAGING_APPLICATIONS_PIPELINE_H_
