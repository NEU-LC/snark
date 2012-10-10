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


} }

