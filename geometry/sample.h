#ifndef ACFR_AERO_GEOMETRY_SAMPLE_H_
#define ACFR_AERO_GEOMETRY_SAMPLE_H_

#include "aero/geometry/coordinates.h"
#include "aero/geometry/circle.h"
#include "aero/geometry/region.h"

namespace acfr { namespace aero {

/// return a uniformly sampled point in given region (it gets denser near the poles)
/// @param r region to sample on
/// @param centre centre of a circle containing the region
/// @param radius radius of a circle containing the region
coordinates pretty_uniform_sample( const region& r, const coordinates& centre, double radius );

/// return a uniformly sampled point in given region (it gets denser near the poles)
/// @param r region to sample on
/// @param c centre and radius of a circle containing the region
coordinates pretty_uniform_sample( const region& r, const circle& c );

/// return a uniformly sampled point in given region (it gets denser near the poles)
/// @param r region to sample on
coordinates pretty_uniform_sample( const region& r );

} } // namespace acfr { namespace aero {

#endif //ACFR_AERO_GEOMETRY_SAMPLE_H_
