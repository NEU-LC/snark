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
