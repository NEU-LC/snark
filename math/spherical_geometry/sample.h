// Copyright (c) 2013-2016. This code was produced by the
// Australian Centre for Field Robotics, The University of Sydney under
// the Future Flight Planning project, University Reference 13996, contract
// NSW-CPS-2011-015264, Work Orders 5, 7 and 8. The intellectual property
// ownership is as set out in these contracts, as registered with
// Commercial Development and Industry Partnerships.

#ifndef ACFR_AERO_GEOMETRY_SAMPLE_H_
#define ACFR_AERO_GEOMETRY_SAMPLE_H_

#include "circle.h"

namespace acfr { namespace aero {

/// return a uniformly sampled point in given region (it gets denser near the poles)
/// @param centre centre of a circle containing the region
/// @param radius radius of a circle containing the region
coordinates pretty_uniform_sample( const coordinates& centre, double radius );

/// return a uniformly sampled point in given region (it gets denser near the poles)
/// @param c centre and radius of a circle containing the region
coordinates pretty_uniform_sample( const circle& c );

} } // namespace acfr { namespace aero {

#endif //ACFR_AERO_GEOMETRY_SAMPLE_H_
