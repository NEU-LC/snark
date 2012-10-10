#ifndef SNARK_PERCEPTION_ALGORITHMS_ALGORITHMS_VOTEDTRACKING_HEADER_GUARD_
#define SNARK_PERCEPTION_ALGORITHMS_ALGORITHMS_VOTEDTRACKING_HEADER_GUARD_

#include <stdlib.h>
#include <algorithm>
#include <map>
#include <boost/optional.hpp>

namespace snark { namespace Robotics {
    
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
Id votedTracking( It begin, It end, F GetPreviousId, Id vacantId );

namespace impl {

template < typename P > inline bool comparePairs( const P& i, const P& j ) { return i.second < j.second; }
    
} // namespace impl {

template < typename It, typename Id, typename F >
inline Id votedTracking( It begin, It end, F getPreviousId, Id vacantId )
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
    
} } // namespace snark { namespace Robotics {

#endif // #ifndef SNARK_PERCEPTION_ALGORITHMS_ALGORITHMS_VOTEDTRACKING_HEADER_GUARD_
