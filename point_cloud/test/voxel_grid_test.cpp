#include <stdlib.h>
#include <vector>
#include <gtest/gtest.h>
#include <snark/point_cloud/voxel_grid.h>

namespace snark { namespace Robotics { namespace Test {

typedef Eigen::Vector3d point;
typedef Eigen::Matrix< std::size_t, 1, 3 > index_type;
typedef std::pair< point, point > extents_type;
    
TEST( voxel_grid, construction )
{
    extents_type e( point( 0, 0, 0 ), point( 10, 10, 10 ) );
    point resolution( 0.2, 0.2, 0.2 );
    snark::voxel_grid< int > g( e, resolution );
    snark::voxel_grid< int > h( g );
    h = g;
}

TEST( voxel_grid, test )
{
    extents_type e( point( 0, 0, 0 ), point( 10, 10, 5 ) );
    snark::voxel_grid< int > grid( e, point( 0.2, 0.2, 0.2 ) );
    
    EXPECT_TRUE( !grid.covers( point( 10.001, 1, 1 ) ) );
    EXPECT_TRUE( !grid.covers( point( 1, 1, 5.001 ) ) );
    EXPECT_TRUE( !grid.covers( point( -0.001, 1, 1 ) ) );
    EXPECT_TRUE( !grid.covers( point( 1, -0.001, 1 ) ) );
    EXPECT_TRUE( !grid.covers( point( 1, 1, -0.001 ) ) );
    
    EXPECT_TRUE( grid.covers( point( 10, 1, 1 ) ) );
    EXPECT_TRUE( grid.covers( point( 1, 1, 5 ) ) );
    EXPECT_TRUE( grid.covers( point( 0, 1, 1 ) ) );
    EXPECT_TRUE( grid.covers( point( 1, 0, 1 ) ) );
    EXPECT_TRUE( grid.covers( point( 1, 1, 0 ) ) );
    EXPECT_TRUE( grid.covers( point( 1, 1, 1 ) )  );

    snark::voxel_grid< int >::index_type index = grid.index_of( point( 1, 1, 1 ) );
    point origin = grid.origin( index );
    EXPECT_EQ( origin[0] , ( 0.2 * double( index[0] ) + 0.0 ) );
    EXPECT_EQ( origin[1] , ( 0.2 * double( index[1] ) + 0.0 ) );
    EXPECT_EQ( origin[2] , ( 0.2 * double( index[2] ) + 0.0 ) );

    for( unsigned int i = 0; i < grid.size()[0]; ++i )
    {
        point p( i, i, i );
        //if( !e.has( p ) ) { EXPECT_TRUE( !grid.covers( p ) ); continue; }
        //EXPECT_TRUE( grid.covers( p ) );
        if( !grid.covers( p ) ) { continue; }
        int* v = grid.touch_at( p );
        EXPECT_TRUE( v );
        *v = 1234;
        index_type ind = grid.index_of( p );
        EXPECT_EQ( *v, grid( ind ) );
        EXPECT_EQ( *v, grid( ind[0], ind[1], ind[2] ) );
        point q = p + point( 0.001, 0.001, 0.001 );
        if( grid.covers( q ) )
        {
            int* w = grid.touch_at( q );
            EXPECT_TRUE( w );
            //EXPECT_EQ( w, v ); todo: invalid test, since the elements may get reallocated; fix... sometime...
            ++( *w );
            grid.erase_at( q );
            EXPECT_TRUE( !grid.exists( grid.index_of( q ) ) );
        }
    }
}

} } } // namespace snark { namespace Robotics { namespace Test {

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

