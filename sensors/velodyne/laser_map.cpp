#include <map>
#include <snark/sensors/velodyne/laser_map.h>

namespace snark {  namespace velodyne {

laser_map::laser_map( const snark::velodyne::db& db )
{
    std::map< double, unsigned int > elevation;
    for( unsigned int i = 0; i < 64; ++i )
    {
        elevation[ -db.lasers[i].correction_angles.vertical.value ] = i;
    }
    unsigned int i = 0;
    for( std::map< double, unsigned int >::const_iterator it = elevation.begin(); it != elevation.end() ; ++it, ++i )
    {
        indices_[ it->second ] = i;
        ids_[i] = it->second;
    }
}

unsigned int laser_map::id_to_index( unsigned int i ) const
{
    return indices_[i];
}

unsigned int laser_map::operator[]( unsigned int i ) const
{
    return id_to_index( i );
}

unsigned int laser_map::index_to_id( unsigned int i ) const
{
    return ids_[i];
}

} } // namespace snark {  namespace velodyne {
