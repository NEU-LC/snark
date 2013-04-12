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


#ifndef SNARK_PERCEPTION_VOXELGRID_HEADER_GUARD
#define SNARK_PERCEPTION_VOXELGRID_HEADER_GUARD

#include <cmath>
#include <map>
#include <boost/none_t.hpp>
#include <Eigen/Core>
#include <boost/optional.hpp>
#include <snark/math/interval.h>
#include <snark/point_cloud/impl/pin_screen.h>

namespace snark {

/// voxel grid
/// @todo this class is mostly copy-pasted
///       just to allow refactoring elsewhere
///       refactor this class further, if needed
template < typename V = boost::none_t, typename P = Eigen::Vector3d >
class voxel_grid : public pin_screen< V >
{
    public:
        typedef V voxel_type;
        typedef P point_type;
        typedef typename pin_screen< V >::index_type index_type;
        typedef typename pin_screen< V >::size_type size_type;
        typedef typename pin_screen< V >::column_type column_type;
        typedef snark::math::closed_interval< typename P::Scalar, P::RowsAtCompileTime > interval_type;
        
        /// constructor
        /// @note if adjusted, the actual extents will be
        ///       extents - resolution / 2, extents + resolution / 2
        /// @todo get rid of "adjusted", it's just ugly
        voxel_grid( const interval_type& extents
                  , const point_type& resolution
                  , bool adjusted = false );

        /// return extents
        const interval_type& extents() const;

        /// return resolution
        const point_type& resolution() const;

        /// return index of the voxel covering a given datapoint
        index_type index_of( const point_type& p ) const;

        /// return true, if voxel grid covers a datapoint
        bool covers( const point_type& p ) const;

        /// create voxel that would cover given point and return index, if point is not out of bound
        voxel_type* touch_at( const point_type& d );

        /// erase voxel covering given point
        void erase_at( const point_type& p );

        /// return voxel origin
        point_type origin( const index_type& i ) const;

        /// return voxel origin
        point_type origin_at( const point_type& p ) const;

        /// return column of voxels
        const column_type* column( const point_type& p ) const;
        //column_type* column( const point_type& p ); // no non-const class in pin_screen for now

        using typename pin_screen< voxel_type >::iterator;
        using typename pin_screen< voxel_type >::const_iterator;
        using typename pin_screen< voxel_type >::neighbourhood_iterator;
        using pin_screen< voxel_type >::column;

    private:
        interval_type extents_;
        point_type resolution_;
};

namespace detail {

template < typename P >
inline static math::closed_interval< typename P::Scalar, P::RowsAtCompileTime > extents( const math::closed_interval< typename P::Scalar, P::RowsAtCompileTime >& e, const P& r, bool adjusted = true )
{
    return adjusted ? math::closed_interval< typename P::Scalar, P::RowsAtCompileTime >( e.min() - r / 2, e.max() + r / 2 ) : e;
}

template < typename P >
inline static Eigen::Matrix< std::size_t, 1, 2 > size( const math::closed_interval< typename P::Scalar, P::RowsAtCompileTime >& e, const P& r, bool adjusted = true )
{
    math::closed_interval< typename P::Scalar, P::RowsAtCompileTime > f = extents( e, r, adjusted );
    P diff = f.max() - f.min();
    return Eigen::Matrix< std::size_t, 1, 2 >( std::ceil( diff.x() / r.x() ), std::ceil( diff.y() / r.y() ) );
}

} // namespace detail {

template < typename V, typename P >
inline voxel_grid< V, P >::voxel_grid( const typename voxel_grid< V, P >::interval_type& extents
                                , const typename voxel_grid< V, P >::point_type& resolution
                                , bool adjusted )
    : pin_screen< V >( detail::size( extents, resolution, adjusted ) )
    , extents_( detail::extents( extents, resolution, adjusted ) )
    , resolution_( resolution )
{
}

template < typename V, typename P >
inline const typename voxel_grid< V, P >::interval_type& voxel_grid< V, P >::extents() const { return extents_; }

template < typename V, typename P >
inline const P& voxel_grid< V, P >::resolution() const { return resolution_; }

template < typename V, typename P >
inline typename voxel_grid< V, P >::index_type voxel_grid< V, P >::index_of( const P& p ) const
{
    return index_type( std::floor( ( p.x() - extents_.min().x() ) / resolution_.x() )
                     , std::floor( ( p.y() - extents_.min().y() ) / resolution_.y() )
                     , std::floor( ( p.z() - extents_.min().z() ) / resolution_.z() ) );
}

template < typename V, typename P >
inline bool voxel_grid< V, P >::covers( const P& p ) const
{
    return extents_.contains( p );
}

template < typename V, typename P >
inline V* voxel_grid< V, P >::touch_at( const P& p )
{
    if( !covers( p ) ) { return NULL; }
    const index_type& i = index_of( p );
    return &pin_screen< V >::touch( i );
}

template < typename V, typename P >
inline void voxel_grid< V, P >::erase_at( const P& point )
{
    if( !covers( point ) ) { return; }
    pin_screen< V >::erase( index_of( point ) );
}

template < typename V, typename P >
inline P voxel_grid< V, P >::origin( const index_type& i ) const
{
    P p( resolution_[0] * i[0], resolution_[1] * i[1], resolution_[2] * i[2] );
    return extents_.min() + p;
}

template < typename V, typename P >
inline P voxel_grid< V, P >::origin_at( const point_type& p ) const
{
    return origin( index_of( p ) );
}

template < typename V, typename P >
const typename voxel_grid< V, P >::column_type* voxel_grid< V, P >::column( const point_type& p ) const
{
    if( !covers( p ) ) { return NULL; }
    index_type index = index_of( p );
    return &this->pin_screen< V >::column( index.x(), index.y() );
}

// template < typename V, typename P >
// typename voxel_grid< V, P >::column_type* voxel_grid< V, P >::column( const point_type& p )
// {
//     if( !covers( p ) ) { return NULL; }
//     index i = index_of( p );
//     return &( this->column( i[0], i[1] ) );
// }

} // namespace snark {

#endif // SNARK_PERCEPTION_VOXELGRID_HEADER_GUARD
