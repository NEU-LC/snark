#ifndef SNARK_KML_POINT_H_
#define SNARK_KML_POINT_H_

#include <string>
#include <boost/optional.hpp>
#include "./coordinates.h"

namespace snark { namespace kml {

struct point
{
    kml::position coordinates;
};

} } // namespace snark { namespace kml {

#endif // SNARK_KML_POINT_H_
