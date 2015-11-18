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
        great_circle::arc arc1( coordinates_( 50.2727777778,133.6175 ), coordinates_( 13.8841666667,-73.3333333333 ) );
        great_circle::arc arc2( coordinates_( 64.4083528,-80.0000139 ), coordinates_( 54.7666806,-108.4166917 ) );
        EXPECT_TRUE( arc1.may_intersect( arc2 ) );
    }
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
