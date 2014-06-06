#ifndef SNARK_KML_LINE_STRING_H_
#define SNARK_KML_LINE_STRING_H_

#include <string>
#include "./coordinates.h"

namespace snark { namespace kml {

struct line_string
{
    boost::optional< std::string > altitude_mode;
    std::vector< kml::position > coordinates;
};

} } // namespace snark { namespace kml {

#endif // SNARK_KML_LINE_STRING_H_
