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

#ifndef SNARK_POINT_CLOUD_VOTED_TRACKING_H_
#define SNARK_POINT_CLOUD_VOTED_TRACKING_H_

#include <stdlib.h>
#include <algorithm>
#include <map>
#include <boost/optional.hpp>

namespace snark { 
    
/// A common problem:
///
///     We have two subsequent velodyne scans partitioned.
///     Now we want to colour them. But the partitioning algorithms often
///     don't guarantee that the partition ids will be the same:
///
///     A tree might have partition id = 5 in partition #1, and 6 in partition #2,
///     although it is the same tree.
///
///     Therefore, in GUI, the tree will first be, say, blue and then suddenly
///     turn pink.
///
///  The following algorithm solves this problem for static and slow-moving
///  objects by simple voting.
///
///  It works in one pass (i.e. ~O(N)) by the price of additional memory
///  (as we need to store the previous id - e.g. previous colour).
///
///  See unit test for usage example
template < typename It, typename Id, typename F >
Id voted_tracking( It begin, It end, F GetPreviousId, Id vacantId );

namespace impl {

template < typename P > inline bool comparePairs( const P& i, const P& j ) { return i.second < j.second; }
    
} // namespace impl {

template < typename It, typename Id, typename F >
inline Id voted_tracking( It begin, It end, F getPreviousId, Id vacantId )
{
    typedef std::map< Id, std::size_t > Map;
    typedef std::pair< Id, std::size_t > Pair;
    Map ids;
    for( It it = begin; it != end; ++it )
    {
        boost::optional< Id > previous = getPreviousId( it );
        if( !previous ) { continue; }
        std::pair< typename Map::iterator, bool > p = ids.insert( Pair( *previous, 0 ) );
        if( !p.second ) { ++p.first->second; }
    }
    return ids.empty() ? vacantId
                       : std::max_element( ids.begin(), ids.end(), impl::comparePairs< Pair > )->first;
}
    
} // namespace snark { 

#endif // #ifndef SNARK_POINT_CLOUD_VOTED_TRACKING_H_
