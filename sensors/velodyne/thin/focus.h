#ifndef SNARK_SENSORS_VELODYNE_THIN_FOCUS
#define SNARK_SENSORS_VELODYNE_THIN_FOCUS

#include <map>
#include <boost/shared_ptr.hpp>
#include "./region.h"

namespace snark {  namespace velodyne { namespace thin {

/// focus on particular region in the velodyne scan
class focus
{
    public:
        focus( double rate = 1.0, double ratio = 1.0 );
        template < typename Random >
        bool has( double range, double bearing, double elevation, Random& random ) const;
        double rate_in_focus() const;
        double rate_out_of_focus() const;
        double coverage() const;
        void insert( std::size_t id, region* r );
        void erase( std::size_t id );
        
    private:
        typedef std::map< std::size_t, boost::shared_ptr< region > > Map;
        double m_rate;
        double m_ratio;
        Map m_regions;
        double m_rate_in_focus;
        double m_rate_out_of_focus;
        void update();
};

template < typename Random >
bool focus::has( double range, double bearing, double elevation, Random& random ) const
{
    double r = random();
    for( typename Map::const_iterator it = m_regions.begin(); it != m_regions.end(); ++it )
    {
        if( it->second->has( range, bearing, elevation ) ) { return r < m_rate_in_focus; }
    }
    return r < m_rate_out_of_focus;
}

} } } // namespace snark {  namespace velodyne { namespace thin {

#endif // #ifndev SNARK_SENSORS_VELODYNE_THIN_FOCUS

