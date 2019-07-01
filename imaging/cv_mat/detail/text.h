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

#pragma once

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>

namespace snark { namespace cv_mat { namespace impl {

struct text_input
{
    cv::Point origin;
    cv::Scalar colour;
    std::string text;
    
    text_input() {}
    text_input( const std::string& text, const cv::Point& origin, const cv::Scalar& colour ): origin( origin ), colour( colour ), text( text ) {}
};
    
template < typename H >
class text
{
    public:
        text( const text_input& t
            , const comma::csv::options& csv
            , const std::vector< std::pair< unsigned int, unsigned int > >& ranges = std::vector< std::pair< unsigned int, unsigned int > >() );

        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        
        typedef boost::function< std::pair< H, cv::Mat >( std::pair< H, cv::Mat > ) > functor_t;
        
        /// return functor and boolean flag that indicates whether functor is safely re-entrant in multithread context
        /// functor is re-entrant, if keep_id is set to false
        static std::pair< functor_t, bool > make( const std::string& options );
        
        static std::string usage( unsigned int indent = 0 );
        
    private:
        text_input caption_;
        boost::shared_ptr< comma::io::istream > is_; // quick and dirty, ashamed of myself
        boost::shared_ptr< comma::csv::input_stream< text_input > > istream_; // quick and dirty, ashamed of myself
        std::vector< std::pair< unsigned int, unsigned int > > ranges_; 
        unsigned int range_index_;
        unsigned int count_;
};

} } } // namespace snark { namespace cv_mat { namespace impl {
