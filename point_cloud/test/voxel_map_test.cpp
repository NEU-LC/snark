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

