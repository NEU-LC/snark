#include "../great_circle.h"

#include <gtest/gtest.h>

#include <iostream>
#include <vector>

using namespace snark::spherical;

namespace {

static const double degree = M_PI / 180;

coordinates coordinates_( double latitude, double longitude ) { return coordinates( latitude * degree, longitude * degree ); }

inline coordinates deg2coordinates( double lat, double lon )
{
    return coordinates::from_degrees(lat, lon);
}

// convert a vector of points in degrees to a vector of points in radians
void deg2coordinates( const std::vector< std::pair< double, double > > & points, std::vector< coordinates > & coords )
{
    for ( std::vector< std::pair< double, double > >::const_iterator i = points.begin(); i != points.end(); ++i )
    {
        coords.push_back( coordinates::from_degrees( i->first, i->second ) );
    }
}


} // namespace anonymous

TEST(geometry, may_intersect_01)
{
    {
        bool force = true;
        great_circle::arc arc1( coordinates_( 50.2727777778,133.6175 ), coordinates_( 13.8841666667,-73.3333333333 ) );
        great_circle::arc arc2( coordinates_( 64.4083528,-80.0000139 ), coordinates_( 54.7666806,-108.4166917 ) );
        EXPECT_TRUE( arc1.may_intersect( arc2 ) );
        EXPECT_TRUE( arc1.intersection_with( arc2, force ) );
    }
}

TEST(geometry, may_intersect_02)
{
    {
        // spin a pair of intersecting arcs in longitude: shall always report intersection regardless of longitude shift
        bool force = true;
        great_circle::arc arc1_base( coordinates_( -10.0,10.0 ), coordinates_( 10.0,15.0 ) );
        great_circle::arc arc2_base( coordinates_( 5.0,0.0 ), coordinates_( -5.0,20.0 ) );
        EXPECT_TRUE( arc1_base.may_intersect( arc2_base ) );
        EXPECT_TRUE( arc1_base.intersection_with( arc2_base, force ) );
        for ( size_t shift = 0 ; shift < 400; ++shift )
        {
            coordinates offset( coordinates_( 0, shift ) );
            great_circle::arc arc1( arc1_base.begin_coordinates() + offset, arc1_base.end_coordinates() + offset );
            great_circle::arc arc2( arc2_base.begin_coordinates() + offset, arc2_base.end_coordinates() + offset );
            EXPECT_TRUE( arc1.may_intersect( arc2 ) );
            EXPECT_TRUE( arc1.intersection_with( arc2, force ) );
        }
    }
}

TEST(geometry, may_intersect_03)
{
    {
        // spin a pair of non-intersecting arcs in longitude: shall never report intersection regardless of longitude shift
        bool force = true;
        great_circle::arc arc1_base( coordinates_( -10.0,10.0 ), coordinates_( 10.0,15.0 ) );
        great_circle::arc arc2_base( coordinates_( 5.0,17.0 ), coordinates_( -5.0,37.0 ) );
        EXPECT_FALSE( arc1_base.may_intersect( arc2_base ) );
        EXPECT_FALSE( arc1_base.intersection_with( arc2_base, force ) );
        for ( size_t shift = 0 ; shift < 400; ++shift )
        {
            coordinates offset( coordinates_( 0, shift ) );
            great_circle::arc arc1( arc1_base.begin_coordinates() + offset, arc1_base.end_coordinates() + offset );
            great_circle::arc arc2( arc2_base.begin_coordinates() + offset, arc2_base.end_coordinates() + offset );
            EXPECT_FALSE( arc1.may_intersect( arc2 ) );
            EXPECT_FALSE( arc1.intersection_with( arc2, force ) );
        }
    }
}

TEST(geometry, may_intersect_04)
{
    {
        // spin a pair of intersecting arcs in latitude: shall always report intersection regardless of latitude shift
        bool force = true;
        great_circle::arc arc1_base( coordinates_( -10.0,10.0 ), coordinates_( 10.0,15.0 ) );
        great_circle::arc arc2_base( coordinates_( 5.0,0.0 ), coordinates_( -5.0,20.0 ) );
        EXPECT_TRUE( arc1_base.may_intersect( arc2_base ) );
        EXPECT_TRUE( arc1_base.intersection_with( arc2_base, force ) );
        for ( size_t shift = -180 ; shift < 180; ++shift )
        {
            coordinates offset( coordinates_( shift, 0 ) );
            great_circle::arc arc1( arc1_base.begin_coordinates() + offset, arc1_base.end_coordinates() + offset );
            great_circle::arc arc2( arc2_base.begin_coordinates() + offset, arc2_base.end_coordinates() + offset );
            EXPECT_TRUE( arc1.may_intersect( arc2 ) );
            EXPECT_TRUE( arc1.intersection_with( arc2, force ) );
        }
    }
}

TEST(geometry, may_intersect_05)
{
    {
        // spin a pair of non-intersecting arcs in latitude: shall never report intersection regardless of latitude shift
        bool force = true;
        great_circle::arc arc1_base( coordinates_( -10.0,10.0 ), coordinates_( 10.0,15.0 ) );
        great_circle::arc arc2_base( coordinates_( 5.0,17.0 ), coordinates_( -5.0,37.0 ) );
        EXPECT_FALSE( arc1_base.may_intersect( arc2_base ) );
        EXPECT_FALSE( arc1_base.intersection_with( arc2_base, force ) );
        for ( size_t shift = -180 ; shift < 180; ++shift )
        {
            coordinates offset( coordinates_( shift, 0 ) );
            great_circle::arc arc1( arc1_base.begin_coordinates() + offset, arc1_base.end_coordinates() + offset );
            great_circle::arc arc2( arc2_base.begin_coordinates() + offset, arc2_base.end_coordinates() + offset );
            EXPECT_FALSE( arc1.may_intersect( arc2 ) );
            EXPECT_FALSE( arc1.intersection_with( arc2, force ) );
        }
    }
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
