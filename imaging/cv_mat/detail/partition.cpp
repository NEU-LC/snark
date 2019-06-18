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

#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/functional/hash.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include "partition.h"

namespace snark { namespace cv_mat { namespace impl {

template < typename S, typename T = S >
struct pair_hash
{
    std::size_t operator()( const std::pair< T, T >& pair ) const
    {
        std::size_t seed = 0;
        boost::hash_combine( seed, pair.first );
        boost::hash_combine( seed, pair.second );
        return seed;
    }
};

template < typename H >
std::pair< H, cv::Mat > partition( std::pair< H, cv::Mat > m, boost::optional< cv::Scalar > do_not_visit_value, comma::int32 none, bool merge )
{
    if( m.second.channels() > 1 ) { COMMA_THROW( comma::exception, "partition: currently support only single-channel images; got " << m.second.channels() << " channels" ); }
    if( merge && m.second.type() != CV_32SC1 ) { COMMA_THROW( comma::exception, "partition: asked to merge, expected image of type i (CV_32SC1), got: " << m.second.type() ); }
    cv::Mat partitions( m.second.rows, m.second.cols, CV_32SC1, cv::Scalar( none ) );
    comma::int32 id = 0;
    std::unordered_set< std::pair< int, int >, pair_hash< int > > neighbours;
    auto insert_neighbour = [&]( int i, int j, int io, int jo )
    {
        int ni = i + io;
        if( ni < 0 || ni >= m.second.rows ) { return; }
        int nj = j + jo;
        if( nj < 0 || nj >= m.second.cols ) { return; }
        if( partitions.template at< comma::int32 >( ni, nj ) != none ) { return; }
        if( do_not_visit_value && std::memcmp( m.second.ptr( ni, nj ), &( *do_not_visit_value ), m.second.elemSize() ) == 0 ) { return; }
        if( std::memcmp( m.second.ptr( ni, nj ), m.second.ptr( i, j ), m.second.elemSize() ) != 0 ) { return; }
        neighbours.insert( std::make_pair( ni, nj ) );
    };
    for( int i = 0; i < m.second.rows; ++i )
    {
        for( int j = 0; j < m.second.cols; ++j )
        {
            if( partitions.template at< comma::int32 >( i, j ) != none ) { continue; }
            if( do_not_visit_value && std::memcmp( m.second.ptr( i, j ), &( *do_not_visit_value ), m.second.elemSize() ) == 0 ) { continue; }
            std::unordered_set< std::pair< int, int >, pair_hash< int > > neighbours; // quick and dirty; performance should suck
            neighbours.insert( std::make_pair( i, j ) );
            while( !neighbours.empty() )
            {
                partitions.template at< comma::int32 >( neighbours.begin()->first, neighbours.begin()->second ) = id;
                neighbours.erase( neighbours.begin() );
                insert_neighbour( i, j, -1, -1 );
                insert_neighbour( i, j, -1,  0 );
                insert_neighbour( i, j, -1,  1 );
                insert_neighbour( i, j,  0, -1 );
                insert_neighbour( i, j,  0,  1 );
                insert_neighbour( i, j,  1, -1 );
                insert_neighbour( i, j,  1,  0 );
                insert_neighbour( i, j,  1,  1 );
            }
            ++id;
        }
    }
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    if( merge )
    {
        std::vector< cv::Mat > channels( 2 );
        channels[0] = m.second;
        channels[1] = partitions;
        cv::merge( channels, n.second );
    }
    else
    {
        n.second = partitions;
    }
    return n;
}

template std::pair< boost::posix_time::ptime, cv::Mat > partition< boost::posix_time::ptime >( std::pair< boost::posix_time::ptime, cv::Mat >, boost::optional< cv::Scalar > do_not_visit_value, comma::int32 none, bool merge );
template std::pair< std::vector< char >, cv::Mat > partition< std::vector< char > >( std::pair< std::vector< char >, cv::Mat >, boost::optional< cv::Scalar > do_not_visit_value, comma::int32 none, bool merge );

} } }  // namespace snark { namespace cv_mat { namespace impl {
