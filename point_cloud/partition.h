// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef SNARK_POINTCLOUD_PARTITION_H_
#define SNARK_POINTCLOUD_PARTITION_H_

#include <Eigen/Core>
#include <comma/base/types.h>
#include "../math/interval.h"
#include "voxel_grid.h"

namespace snark {

class partition
{
    public:
        typedef snark::math::closed_interval< double, 3 > extents_type;

        partition( const extents_type& extents
                 , const Eigen::Vector3d& resolution
                 , std::size_t min_points_per_voxel = 1 );

        ~partition();

        const boost::optional< comma::uint32 >& insert( const Eigen::Vector3d& point );

        void commit();

        /// @param min_density is number of points in partition / number of voxels in partition
        /// @todo define better signature for commit()
        void commit( std::size_t min_voxels_per_partition
                   , std::size_t min_points_per_partition
                   , comma::uint32 min_id = 0
                   , double min_density = 0 );

    private:
        class impl_;
        impl_* pimpl_;
};

} // namespace snark {

#endif // SNARK_POINTCLOUD_PARTITION_H_
