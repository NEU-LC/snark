// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

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
