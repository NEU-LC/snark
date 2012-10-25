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

#include <snark/point_cloud/voxel_map.h>
#include <gtest/gtest.h>

namespace snark { namespace Robotics {

typedef voxel_map< int, 3 > map_type;

TEST( voxel_map, index )
{
    {
        map_type m( map_type::point_type( 1, 1, 1 ) );
        {
            map_type::index_type i = {{ 0, 0, 0 }};
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0, 0, 0 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0.001, 0.001, 0.001 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0.999, 0.999, 0.999 ) ) );
        }
        {
            map_type::index_type i = {{ 1, 1, 1 }};
            EXPECT_EQ( i, m.index_of( map_type::point_type( 1.0, 1.0, 1.0 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( 1.001, 1.001, 1.001 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( 1.999, 1.999, 1.999 ) ) );
        }
        {
            map_type::index_type i = {{ -1, -1, -1 }};
            EXPECT_EQ( i, m.index_of( map_type::point_type( -1.0, -1.0, -1.0 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( -0.999, -0.999, -0.999 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( -0.001, -0.001, -0.001 ) ) );
        }
    }
    {
        map_type m( map_type::point_type( 0.3, 0.3, 0.3 ) );
        {
            map_type::index_type i = {{ 0, 0, 0 }};
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0, 0, 0 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0.001, 0.001, 0.001 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0.299, 0.299, 0.299 ) ) );
        }        
        {
            map_type::index_type i = {{ 1, 1, 1 }};
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0.3, 0.3, 0.3 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0.3001, 0.3001, 0.3001 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( 0.3999, 0.3999, 0.3999 ) ) );
        }
        {
            map_type::index_type i = {{ -1, -1, -1 }};
            EXPECT_EQ( i, m.index_of( map_type::point_type( -0.3, -0.3, -0.3 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( -0.299, -0.299, -0.299 ) ) );
            EXPECT_EQ( i, m.index_of( map_type::point_type( -0.001, -0.001, -0.001 ) ) );
        }        
    }
}

TEST( voxel_map, operations )
{
    map_type m( map_type::point_type( 1, 1, 1 ) );
    {
        EXPECT_TRUE( ( m.find( map_type::point_type( 1, 1, 1 ) ) == m.end() ) );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( 1, 1, 1 ) ) != m.end() ) );
        EXPECT_EQ( 1, m.size() );
        EXPECT_TRUE( ( m.find( map_type::point_type( 1, 1, 1 ) ) != m.end() ) );
        EXPECT_TRUE( ( m.find( map_type::point_type( 1, 1, 1 ) ) == m.find( map_type::point_type( 1.1, 1.1, 1.1 ) ) ) );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( 1, 1, 1 ) ) != m.end() ) );
        EXPECT_EQ( 1, m.size() );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( 1.1, 1.1, 1.1 ) ) != m.end() ) );
        EXPECT_EQ( 1, m.size() );
    }
    {
        EXPECT_TRUE( ( m.find( map_type::point_type( -1, -1, -1 ) ) == m.end() ) );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( -1, -1, -1 ) ) != m.end() ) );
        EXPECT_EQ( 2, m.size() );
        EXPECT_TRUE( ( m.find( map_type::point_type( -1, -1, -1 ) ) != m.end() ) );
        EXPECT_TRUE( ( m.find( map_type::point_type( -1, -1, -1 ) ) == m.find( map_type::point_type( -0.1, -0.1, -0.1 ) ) ) );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( -1, -1, -1 ) ) != m.end() ) );
        EXPECT_EQ( 2, m.size() );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( -0.1, -0.1, -0.1 ) ) != m.end() ) );
        EXPECT_EQ( 2, m.size() );
    }
    {
        EXPECT_TRUE( ( m.find( map_type::point_type( 0, 0, 0 ) ) == m.end() ) );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( 0, 0, 0 ) ) != m.end() ) );
        EXPECT_EQ( 3, m.size() );
        EXPECT_TRUE( ( m.find( map_type::point_type( 0, 0, 0 ) ) != m.end() ) );
        EXPECT_TRUE( ( m.find( map_type::point_type( 0, 0, 0 ) ) == m.find( map_type::point_type( 0.1, 0.1, 0.1 ) ) ) );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( 0, 0, 0 ) ) != m.end() ) );
        EXPECT_EQ( 3, m.size() );
        EXPECT_TRUE( ( m.touch_at( map_type::point_type( 0.1, 0.1, 0.1 ) ) != m.end() ) );
        EXPECT_EQ( 3, m.size() );
    }    
}

TEST( voxel_map, test )
{
    map_type m( map_type::point_type( 1, 1, 1 ) );
    EXPECT_TRUE( m.empty() );
}

TEST( voxel_map, neighbourhood )
{
    map_type m( map_type::point_type( 1, 1, 1 ) );
    {
        EXPECT_TRUE( ( m.find( map_type::point_type( 1, 1, 1 ) ) == m.end() ) );
        {
            EXPECT_TRUE( ( m.touch_at( map_type::point_type( 1, 1, 1 ) ) != m.end() ) );
            EXPECT_EQ( 1, m.size() );
            m.touch_at( map_type::point_type( 1, 1, 1 ) )->second = 111;
            EXPECT_EQ( 111, m.find( map_type::point_type( 1, 1, 1 ) )->second );
            map_type::index_type index = {{ 1, 1, 1 }};
            EXPECT_EQ( 111, m.base_type::find( index )->second );
        }
        {
            EXPECT_TRUE( ( m.touch_at( map_type::point_type( 2, 2, 2 ) ) != m.end() ) );
            EXPECT_EQ( 2, m.size() );
            m.touch_at( map_type::point_type( 2, 2, 2 ) )->second = 222;
            EXPECT_EQ( 222, m.find( map_type::point_type( 2, 2, 2 ) )->second );
            map_type::index_type index = {{ 2, 2, 2 }};
            EXPECT_EQ( 222, m.base_type::find( index )->second );
        }
        {
            map_type::index_type index = {{ -1, 0, 0 }};
            EXPECT_TRUE( m.base_type::find( index ) == m.end() );
        }
        {
            map_type::index_type index = {{ 0, 0, 0 }};
            EXPECT_TRUE( m.base_type::find( index ) == m.end() );
        }
        {
            map_type::index_type index = {{ 2, 2, 3 }};
            EXPECT_TRUE( m.base_type::find( index ) == m.end() );
        }
    }
}

} }

