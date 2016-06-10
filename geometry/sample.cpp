// Copyright (c) 2013-2016. This code was produced by the
// Australian Centre for Field Robotics, The University of Sydney under
// the Future Flight Planning project, University Reference 13996, contract
// NSW-CPS-2011-015264, Work Orders 5, 7 and 8. The intellectual property
// ownership is as set out in these contracts, as registered with
// Commercial Development and Industry Partnerships.

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <Eigen/Geometry>
#include <comma/base/exception.h>
#include <snark/math/range_bearing_elevation.h>
#include "circle.h"
#include "sample.h"

namespace acfr { namespace aero {

namespace impl {

boost::mt19937 generator;
boost::uniform_real< double > distribution( 0, 1 );
boost::variate_generator< boost::mt19937&, boost::uniform_real< double > > random( generator, distribution );

} // namespace impl {

coordinates pretty_uniform_sample( const coordinates& centre, double radius ) { return pretty_uniform_sample( circle( centre, radius ) ); }

coordinates pretty_uniform_sample( const circle& c )
{
    if( c.radius >= M_PI ) { COMMA_THROW( comma::exception, "support containing circle radius less than pi; got " << c.radius ); }
    const Eigen::Matrix3d& r1 = Eigen::AngleAxis< double >( c.centre.longitude, Eigen::Vector3d( 0, 0, 1 ) ).toRotationMatrix();
    double a = ( impl::random() * 2 - 1 ) * c.radius; // double a = impl::random() * M_PI * 2;
    double b = ( impl::random() * 2 - 1 ) * c.radius; // double b = std::sqrt( impl::random() ) * c.radius;
    const Eigen::Vector3d& s = snark::range_bearing_elevation( 1, a, b ).to_cartesian(); // const Eigen::Vector3d& s = snark::range_bearing_elevation( 1, b * std::cos( a ), b * std::sin( a ) ).to_cartesian();
    const Eigen::Matrix3d& r2 = Eigen::AngleAxis< double >( c.centre.latitude, Eigen::Vector3d( 0, -1, 0 ) ).toRotationMatrix();
    //return coordinates( r2 * r1 * s );
    return coordinates( r1 * r2 * s );
}

} } // namespace acfr { namespace aero {
