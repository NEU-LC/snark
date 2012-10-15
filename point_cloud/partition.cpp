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

#include <cmath>
#include <snark/point_cloud/equivalence_classes.h>
#include <snark/point_cloud/partition.h>
#include <snark/point_cloud/voxel_grid.h>

namespace snark {

class partition::impl_
{
    public:
        impl_( const partition::extents_type& extents
             , const Eigen::Vector3d& resolution
             , std::size_t min_points_per_voxel = 1 )
            : voxels_( expanded_( extents, resolution ), resolution )
            , min_points_per_voxel_( min_points_per_voxel )
        {
        }
        
        const boost::optional< comma::uint32 >& insert( const Eigen::Vector3d& point )
        {
            voxel_* voxel = voxels_.touch_at( point );
            if( voxel == NULL ) { return none_; }
            ++voxel->count;
            return voxel->id;
        }

        void commit( std::size_t min_voxels_per_partition = 1
                   , std::size_t min_points_per_partition = 1
                   , comma::uint32 min_id = 0 )
        {
            std::size_t pc = 0;
            std::size_t vc = 0;
            for( voxels_type_::iterator it = voxels_.begin(); it != voxels_.end(); ++it )
            {
                if( it->count < min_points_per_voxel_ ) { it->count = 0; }
                ++vc;
                pc += it->count;
            }
            typedef std::list< voxels_type_::iterator > Set;
            typedef std::map< comma::uint32, Set > partitions;
            typedef voxels_type_::iterator It;
            typedef voxels_type_::neighbourhood_iterator Nit;
            const partitions& parts = snark::equivalence_classes< It, Nit, Methods_ >( voxels_.begin(), voxels_.end(), min_id );
            bool check_points_per_partitions = min_points_per_partition > min_voxels_per_partition * min_points_per_voxel_;
            for( partitions::const_iterator it = parts.begin(); it != parts.end(); ++it )
            {
                bool remove = false;
                if( it->second.size() < min_voxels_per_partition )
                {
                    remove = true;
                }
                else if( check_points_per_partitions ) // watch performance
                {
                    std::size_t size = 0;
                    for( Set::const_iterator j = it->second.begin(); j != it->second.end(); size += ( *j++ )->count );
                    if( size < min_points_per_partition ) { remove = true; }
                }
                if( remove )
                {
                    for( Set::const_iterator j = it->second.begin(); j != it->second.end(); ( *j++ )->id.reset() );
                }
            }
        }

    private:
        struct voxel_ // quick and dirty
        {
            mutable boost::optional< comma::uint32 > id; // quick and dirty
            std::size_t count;
            bool visited;

            voxel_() : id( 0 ), count( 0 ), visited( false ) {}
        };

        struct Methods_
        {
            static bool skip( const voxel_& e ) { return e.count == 0; }
            static bool same( const voxel_& lhs, const voxel_& rhs ) { return true; }
            static bool visited( const voxel_& e ) { return e.visited; }
            static void set_visited( voxel_& e, bool v ) { e.visited = v; }
            static comma::uint32 id( const voxel_& e ) { return *e.id; }
            static void set_id( voxel_& e, comma::uint32 id ) { e.id = id; }
        };

        typedef voxel_grid< voxel_ > voxels_type_;
        voxels_type_ voxels_;
        boost::optional< comma::uint32 > none_;
        std::size_t min_points_per_voxel_;
        
        partition::extents_type expanded_( const partition::extents_type& extents, const Eigen::Vector3d& resolution )
        {
            Eigen::Vector3d floor = extents.min() - resolution / 2;
            Eigen::Vector3d ceil = extents.max() + resolution / 2;
            return partition::extents_type( Eigen::Vector3d( std::floor( floor.x() ), std::floor( floor.y() ), std::floor( floor.z() ) )
                                          , Eigen::Vector3d( std::ceil( ceil.x() ), std::ceil( ceil.y() ), std::ceil( ceil.z() ) ) );
        }
};

partition::partition( const partition::extents_type& extents
                    , const Eigen::Vector3d& resolution
                    , std::size_t min_points_per_voxel )
: pimpl_( new impl_( extents, resolution, min_points_per_voxel ) )
{
}

partition::~partition() { delete pimpl_; }
    
const boost::optional< comma::uint32 >& partition::insert( const Eigen::Vector3d& point )
{
    return pimpl_->insert( point );
}

void partition::commit( std::size_t min_voxels_per_partition
                      , std::size_t min_points_per_partition
                      , comma::uint32 min_id )
{
    pimpl_->commit( min_voxels_per_partition, min_points_per_partition, min_id );
}

} // namespace snark {
