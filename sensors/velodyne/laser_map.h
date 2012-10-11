#include <boost/array.hpp>
#include <snark/sensors/velodyne/db.h>

namespace snark {  namespace velodyne {

/// orders lasers by elevation
class laser_map
{
    public:
        /// constructor
        laser_map( const snark::velodyne::db& db );

        /// take laser id, return index by elevation
        unsigned int id_to_index( unsigned int i ) const;

        /// same as id_to_index
        unsigned int operator[]( unsigned int i ) const;

        /// take index by elevation, return laser id
        unsigned int index_to_id( unsigned int i ) const;
        
    private:
        boost::array< unsigned int, 64 > indices_;
        boost::array< unsigned int, 64 > ids_;
};
    
} } // namespace snark {  namespace velodyne {
    