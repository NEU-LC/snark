// Copyright (c) 2011 The University of Sydney

#pragma once

#include <Eigen/Core>
#include <comma/base/types.h>
#include "../math/interval.h"

namespace snark {

class partition
{
    public:
        typedef snark::math::closed_interval< double, 3 > extents_type;

        partition( const extents_type& extents, const Eigen::Vector3d& resolution, std::size_t min_points_per_voxel = 1 );

        ~partition();

        const boost::optional< comma::uint32 >& insert( const Eigen::Vector3d& point );

        /// @param min_density is number of points in partition / number of voxels in partition
        /// @todo define better signature for commit()
        void commit( std::size_t min_voxels_per_partition = 1
                   , std::size_t min_points_per_partition = 1
                   , comma::uint32 min_id = 0
                   , double min_density = 0
                   , unsigned int keep_largest = 0 );

    private:
        class impl_;
        impl_* pimpl_;
};

} // namespace snark {
