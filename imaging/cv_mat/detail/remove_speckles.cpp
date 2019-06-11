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

/// @author vsevolod vlaskine

#include <memory>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include "remove_speckles.h"

namespace snark { namespace cv_mat { namespace impl {

template < typename H >
std::pair< H, cv::Mat > remove_speckles( std::pair< H, cv::Mat > m, cv::Size2i s )
{
    int wback = s.width - 1;
    int hback = s.height - 1;
    int last_j = 0; // poor man's optimisation
    for( int j = 0; j < m.second.rows - s.height; ++j ) // todo: super quick and dirty, use tbb and better operations on cv::mat
    {
        int last_i = 0;
        for( int i = 0; i < m.second.cols - s.width; ++i )
        {
            cv::Mat r( m.second, cv::Rect( i, j, s.width, s.height ) );
            auto c = r.ptr( 0, 0 );
            bool same = true;
            for( int k = 0; k < s.width && same; ++k ) { same = std::memcmp( c, r.ptr( 0, k ), r.elemSize() ) == 0 && std::memcmp( c, r.ptr( hback, k ), r.elemSize() ) == 0; }
            for( int k = 1; k < hback && same; ++k ) { same = std::memcmp( c, r.ptr( k, 0 ), r.elemSize() ) == 0 && std::memcmp( c, r.ptr( k, wback ), r.elemSize() ) == 0; }
            if( !same ) { continue; }
            int from = last_j == j && i - last_i < wback ? wback - ( i - last_i ) : 1;
            for( int k = 1; k < hback; ++k )
            {
                for( int l = from; l < wback; ++l )
                {
                    std::memcpy( r.ptr( k, l ), c, r.elemSize() );
                }
            }
            last_i = i;
            last_j = j; 
        }
    }
    return m;
}

template std::pair< boost::posix_time::ptime, cv::Mat > remove_speckles< boost::posix_time::ptime >( std::pair< boost::posix_time::ptime, cv::Mat >, cv::Size2i );
template std::pair< std::vector< char >, cv::Mat > remove_speckles< std::vector< char > >( std::pair< std::vector< char >, cv::Mat >, cv::Size2i );

} } }  // namespace snark { namespace cv_mat { namespace impl {
