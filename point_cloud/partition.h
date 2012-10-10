#ifndef SNARK_POINTCLOUD_PARTITION_H_
#define SNARK_POINTCLOUD_PARTITION_H_

#include <boost/optional.hpp>
#include <Eigen/Core>
#include <comma/base/types.h>
#include <snark/point_cloud/voxel_grid.h>

namespace snark {

class partition
{
    public:
        typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > extents_type;
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
