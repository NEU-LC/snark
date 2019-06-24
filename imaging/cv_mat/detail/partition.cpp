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
#include <sstream>
#include <unordered_set>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/string/split.h>
#include "partition.h"

#include <iostream>

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
partition< H >::partition( boost::optional< cv::Scalar > do_not_visit_value
                         , comma::int32 none
                         , bool merge
                         , bool keep_id
                         , comma::int32 start_from
                         , unsigned int min_partition_size )
    : do_not_visit_value_( do_not_visit_value )
    , none_( none )
    , merge_( merge )
    , keep_id_( keep_id )
    , start_from_( start_from )
    , id_( start_from )
    , min_partition_size_( min_partition_size )
{
}

template < typename H >
std::pair< H, cv::Mat > partition< H >::operator()( std::pair< H, cv::Mat > m )
{
    if( m.second.channels() > 1 ) { COMMA_THROW( comma::exception, "partition: currently support only single-channel images; got " << m.second.channels() << " channels" ); }
    if( merge_ && m.second.type() != CV_32SC1 ) { COMMA_THROW( comma::exception, "partition: asked to merge, expected image of type i (CV_32SC1), got: " << m.second.type() ); }
    cv::Mat partitions( m.second.rows, m.second.cols, CV_32SC1, cv::Scalar( none_ ) );
    if( !keep_id_ ) { id_ = start_from_; }
    std::unordered_set< std::pair< int, int >, pair_hash< int > > neighbours;
    auto insert_neighbour = [&]( int i, int j, int io, int jo, int none )
    {
        int ni = i + io;
        if( ni < 0 || ni >= m.second.rows ) { return; }
        int nj = j + jo;
        if( nj < 0 || nj >= m.second.cols ) { return; }
        if( partitions.template at< comma::int32 >( ni, nj ) != none ) { return; }
        if( do_not_visit_value_ && std::memcmp( m.second.ptr( ni, nj ), &( *do_not_visit_value_ ), m.second.elemSize() ) == 0 ) { return; }
        if( std::memcmp( m.second.ptr( ni, nj ), m.second.ptr( i, j ), m.second.elemSize() ) != 0 ) { return; }
        neighbours.insert( std::make_pair( ni, nj ) );
    };
    auto visit_partition_at = [&]( int i, int j, int id, int none ) -> unsigned int
    {
        if( partitions.template at< comma::int32 >( i, j ) != none ) { return 0; }
        if( do_not_visit_value_ && std::memcmp( m.second.ptr( i, j ), &( *do_not_visit_value_ ), m.second.elemSize() ) == 0 ) { return 0; }
        unsigned int size = 0;
        neighbours.insert( std::make_pair( i, j ) );
        while( !neighbours.empty() )
        {
            int ci = neighbours.begin()->first;
            int cj = neighbours.begin()->second;
            partitions.template at< comma::int32 >( ci, cj ) = id;
            ++size;
            neighbours.erase( neighbours.begin() );
            insert_neighbour( ci, cj, -1, -1, none );
            insert_neighbour( ci, cj, -1,  0, none );
            insert_neighbour( ci, cj, -1,  1, none );
            insert_neighbour( ci, cj,  0, -1, none );
            insert_neighbour( ci, cj,  0,  1, none );
            insert_neighbour( ci, cj,  1, -1, none );
            insert_neighbour( ci, cj,  1,  0, none );
            insert_neighbour( ci, cj,  1,  1, none );
        }
        return size;
    };
    std::vector< std::pair< int, int > > discarded; // todo? use deque?
    for( int i = 0; i < m.second.rows; ++i )
    {
        for( int j = 0; j < m.second.cols; ++j )
        {
            unsigned int size = visit_partition_at( i, j, id_, none_ );
            if( size == 0 ) { continue; }
            if( size < min_partition_size_ ) { discarded.push_back( std::make_pair( i, j ) ); } // todo: quick and dirty, watch performance
            else { ++id_; }
        }
    }
    for( const auto& p: discarded ) { visit_partition_at( p.first, p.second, none_, partitions.template at< comma::int32 >( p.first, p.second ) ); }
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    if( merge_ )
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

template < typename H >
std::pair< typename partition< H >::functor_t, bool > partition< H >::make( const std::string& options )
{
    boost::optional< cv::Scalar > do_not_visit;
    comma::int32 none = -1;
    bool merge = false;
    bool keep_id = false;
    comma::int32 start_from = 0;
    unsigned int min_partition_size = 0;
    if( !options.empty() )
    {
        std::vector< std::string > s = comma::split( options, ',' );
        for( unsigned int i = 0; i < s.size(); ++i ) // todo: quick and dirty, use visiting
        {
            if( s[i] == "merge" ) { merge = true; }
            else if( s[i] == "keep-id" ) { keep_id = true; }
            else if( s[i].substr( 0, 5 ) == "none:" ) { start_from = boost::lexical_cast< comma::int32 >( s[i].substr( 5 ) ); }
            else if( s[i].substr( 0, 13 ) == "do-not-visit:" ) { do_not_visit = boost::lexical_cast< comma::int32 >( s[i].substr( 13 ) ); }
            else if( s[i].substr( 0, 11 ) == "start-from:" ) { start_from = boost::lexical_cast< comma::int32 >( s[i].substr( 11 ) ); }
            else if( s[i].substr( 0, 9 ) == "min-size:" ) { min_partition_size = boost::lexical_cast< comma::int32 >( s[i].substr( 9 ) ); }
            else { COMMA_THROW( comma::exception, "partition: expected an option, got: '" << s[i] << "'" ); }
        }
    }
    if( start_from <= none ) { COMMA_THROW( comma::exception, "partition: expected start-from > none, got: start-from: " << start_from << " none: " << none ); }
    return std::make_pair( partition< H >( do_not_visit, none, merge, keep_id, start_from, min_partition_size ), !keep_id );
}

template < typename H >
typename std::string partition< H >::usage( unsigned int indent )
{
    std::string offset( indent, ' ' );
    std::ostringstream oss;
    oss << offset << "partition=<options>" << std::endl;
    oss << offset << "    <options>" << std::endl;
    oss << offset << "        do-not-visit:<value>: value that does not represent any class; e.g. 0: do not" << std::endl;
    oss << offset << "                              partition black pixels" << std::endl;
    oss << offset << "        keep-id: keep incrementing id from image to image; otherwise, for each image, " << std::endl;
    oss << offset << "                 start id with the value of start-with option" << std::endl;
    oss << offset << "        merge: if present and image is of type i (32-bit int), output two-channel image:" << std::endl;
    oss << offset << "               first channel: original image, second: partition ids" << std::endl;
    oss << offset << "        min-size:<pixels>: min partition size to keep; set output pixels of smaller partitions to none id" << std::endl;
    oss << offset << "        none:<id>: id that does not represent any class in output; default: -1" << std::endl;
    oss << offset << "        start-with:<id>: start id numbers from <id>; default: 0" << std::endl;
    return oss.str();
}

template class partition< boost::posix_time::ptime >;
template class partition< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {