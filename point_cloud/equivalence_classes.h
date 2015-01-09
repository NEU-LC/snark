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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the University of Sydney.
// 4. Neither the name of the University of Sydney nor the
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

#ifndef SNARK_PERCEPTION_EQUIVALENCECLASSES_HEADER_GUARD_
#define SNARK_PERCEPTION_EQUIVALENCECLASSES_HEADER_GUARD_

#include <cmath>
#include <list>
#include <map>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <comma/base/types.h>

namespace snark {

/// partition elements of container
template < typename It, typename N, typename Tr >
inline std::map< comma::uint32, std::list< It > > equivalence_classes( const It& begin, const It& end, comma::uint32 minId )
{
    typedef std::list< It > partition_type;
    typedef std::map< comma::uint32, partition_type > partitions_type;
    partitions_type partitions;
    comma::uint32 maxId = minId;
    for( It it = begin; it != end; ++it )
    {
        if( Tr::skip( *it ) ) { continue; }
        if( !Tr::visited( *it ) )
        {
            for( typename N::iterator nit = N::begin( it ); nit != N::end( it ); ++nit )
            {
                if( Tr::skip( *nit ) ) { continue; }
                if( !Tr::visited( *nit ) || !Tr::same( *it, *nit ) ) { continue; }
                Tr::set_visited( *it, true );
                Tr::set_id( *it, Tr::id( *nit ) );
            }
            if( !Tr::visited( *it ) )
            {
                Tr::set_visited( *it, true );
                Tr::set_id( *it, maxId++ );
            }
        }
        comma::uint32 id = Tr::id( *it );
        partition_type& p( partitions[id] );
        p.push_back( it );
        for( typename N::iterator nit = N::begin( it ); nit != N::end( it ); ++nit )
        {
            if( Tr::skip( *nit ) ) { continue; }
            if( !Tr::visited( *nit ) || !Tr::same( *it, *nit ) || id == Tr::id( *nit ) ) { continue; }
            typename partitions_type::iterator old = partitions.find( Tr::id( *nit ) );
            for( typename partition_type::iterator i = old->second.begin(); i != old->second.end(); ++i ) // todo: below is quite suboptimal, improve!
            {
                Tr::set_id( *It( *i ), id ); // quick and dirty to wave constness // Tr::set_id( **i, id );
            }
            p.splice( p.end(), old->second, old->second.begin(), old->second.end() );
            partitions.erase( old );
        }
    }
    return partitions;
}

} // namespace snark

#endif // SNARK_PERCEPTION_EQUIVALENCECLASSES_HEADER_GUARD_
