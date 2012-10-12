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
    
