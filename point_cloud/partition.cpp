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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

/// @author vsevolod vlaskine

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

        void commit( std::size_t min_voxels_per_partition
                   , std::size_t min_points_per_partition
                   , comma::uint32 min_id
                   , double min_density )
        {
            for( voxels_type_::iterator it = voxels_.begin(); it != voxels_.end(); ++it )
            { 
                if( it->count < min_points_per_voxel_ )
                { 
                    it->count = 0;
                    it->id.reset();
                }
            }
            typedef std::list< voxels_type_::iterator > Set;
            typedef std::map< comma::uint32, Set > partitions;
            typedef voxels_type_::iterator It;
            typedef voxels_type_::neighbourhood_iterator Nit;
            const partitions& parts = snark::equivalence_classes< It, Nit, Methods_ >( voxels_.begin(), voxels_.end(), min_id );
            bool check_points_per_partitions = min_density > 0 || ( min_points_per_partition > min_voxels_per_partition * min_points_per_voxel_ );
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
                    remove = size < min_points_per_partition || ( double( size ) / it->second.size() ) < min_density;
                }
                if( remove ) { for( Set::const_iterator j = it->second.begin(); j != it->second.end(); ( *j++ )->id.reset() ); }
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

void partition::commit()
{
    pimpl_->commit( 1, 1, 0, 0 );
}

void partition::commit( std::size_t min_voxels_per_partition
                      , std::size_t min_points_per_partition
                      , comma::uint32 min_id
                      , double min_density )
{
    pimpl_->commit( min_voxels_per_partition, min_points_per_partition, min_id, min_density );
}

} // namespace snark {
