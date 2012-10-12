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

