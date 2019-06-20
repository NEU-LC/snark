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

#include <memory>
#include <vector>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include "hard_edge.h"
#include "utils.h"

#include <iostream>

namespace snark { namespace cv_mat { namespace impl {

template < typename H >
std::pair< H, cv::Mat > hard_edge< H >::handle( std::pair< H, cv::Mat > m, float background )
{
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    m.second.copyTo( n.second );
    std::vector< unsigned char > background_pixel( m.second.elemSize() );
    for( unsigned int i = 0; i < m.second.elemSize(); i += m.second.elemSize1() ) { set_channel( &background_pixel[i], background, m.second.depth() ); }
    auto is_background = [&]( int i, int j ) -> bool { return std::memcmp( m.second.ptr( i, j ), &background_pixel[0], m.second.elemSize() ) == 0; };
    auto same = [&]( int i0, int j0, int i1, int j1 ) -> bool
    {
        if( i1 < 0 || i1 >= m.second.rows || j1 < 0 || j1 >= m.second.cols ) { return true; }
        return std::memcmp( m.second.ptr( i0, j0 ), m.second.ptr( i1, j1 ), m.second.elemSize() ) == 0;
    };
    auto reset = [&]( int i, int j ) { std::memcpy( n.second.ptr( i, j ), &background_pixel[0], m.second.elemSize() ); };
    tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, m.second.rows, m.second.rows / 8 ), [&]( const tbb::blocked_range< std::size_t >& r )
    {
        for( unsigned int i = r.begin(); i < r.end(); ++i )
        {
            for( int j = 0; j < m.second.cols; ++ j )
            {
                if( is_background( i, j ) ) { continue; }
                if(    same( i, j, i - 1, j - 1 )
                    && same( i, j, i - 1, j     )
                    && same( i, j, i - 1, j + 1 )
                    && same( i, j, i    , j - 1 )
                    && same( i, j, i    , j + 1 )
                    && same( i, j, i + 1, j - 1 )
                    && same( i, j, i + 1, j     )
                    && same( i, j, i + 1, j + 1 ) )
                {
                    reset( i, j );
                }
            }
        }
    } );
    return n;
}

template < typename H >
std::pair< typename hard_edge< H >::functor_t, bool > hard_edge< H >::make( const std::string& options )
{
    return std::make_pair( boost::bind( &hard_edge< H >::handle, _1, options.empty() ? 0. : boost::lexical_cast< float >( options ) ), true );
}

template < typename H >
std::string hard_edge< H >::usage( unsigned int indent )
{
    return "hard-edge[=<background>]: find hard edge (currently only single-channel images supported); set all non-edge pixels to <background>";
}

template struct hard_edge< boost::posix_time::ptime >;
template struct hard_edge< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {
