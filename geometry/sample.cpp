#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <Eigen/Geometry>
#include <comma/base/exception.h>
#include <snark/math/range_bearing_elevation.h>
#include "aero/geometry/circle.h"
#include "aero/geometry/sample.h"

#include <iostream>

namespace acfr { namespace aero {

namespace impl {

boost::mt19937 generator;
boost::uniform_real< double > distribution( 0, 1 );
boost::variate_generator< boost::mt19937&, boost::uniform_real< double > > random( generator, distribution );

} // namespace impl {

coordinates pretty_uniform_sample( const region& r, const coordinates& centre, double radius ) { return pretty_uniform_sample( r, circle( centre, radius ) ); }

coordinates pretty_uniform_sample( const region& r ) { return pretty_uniform_sample( r, r.containing_circle() ); }

coordinates pretty_uniform_sample( const region& r, const circle& c )
{
    if( c.radius >= M_PI ) { COMMA_THROW( comma::exception, "support containing circle radius less than pi; got " << c.radius ); }
    const Eigen::Matrix3d& r1 = Eigen::AngleAxis< double >( c.centre.longitude, Eigen::Vector3d( 0, 0, 1 ) ).toRotationMatrix();
    static const unsigned int attempts = 100000;
    for( std::size_t i = 0; i < attempts; ++i )
    {
        double a = ( impl::random() * 2 - 1 ) * c.radius; // double a = impl::random() * M_PI * 2;
        double b = ( impl::random() * 2 - 1 ) * c.radius; // double b = std::sqrt( impl::random() ) * c.radius;
        const Eigen::Vector3d& s = snark::range_bearing_elevation( 1, a, b ).to_cartesian(); // const Eigen::Vector3d& s = snark::range_bearing_elevation( 1, b * std::cos( a ), b * std::sin( a ) ).to_cartesian();
        const Eigen::Matrix3d& r2 = Eigen::AngleAxis< double >( c.centre.latitude, Eigen::Vector3d( 0, -1, 0 ) ).toRotationMatrix();
        coordinates p( r2 * r1 * s );
        if( r.includes( p ) ) { return p; }
    }
    COMMA_THROW( comma::exception, "failed to get a sample point after " << attempts << " attempts" );
}

} } // namespace acfr { namespace aero {
