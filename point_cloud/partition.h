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

#ifndef SNARK_POINTCLOUD_PARTITION_H_
#define SNARK_POINTCLOUD_PARTITION_H_

#include <boost/optional.hpp>
#include <Eigen/Core>
#include <comma/base/types.h>
#include <snark/math/interval.h>
#include <snark/point_cloud/voxel_grid.h>

namespace snark {

class partition
{
    public:
        typedef snark::math::interval< double, 3 > extents_type;
        partition( const extents_type& extents
                 , const Eigen::Vector3d& resolution
                 , std::size_t min_points_per_voxel = 1 );

        ~partition();
        
        const boost::optional< comma::uint32 >& insert( const Eigen::Vector3d& point );

        void commit( std::size_t min_voxels_per_partition = 1
                   , std::size_t min_points_per_partition = 1
                   , comma::uint32 min_id = 0 );

    private:
        class impl_;
        impl_* pimpl_;
};

} // namespace snark {

#endif // SNARK_POINTCLOUD_PARTITION_H_
