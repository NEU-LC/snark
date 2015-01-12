#ifndef SNARK_RENDER_KML_LINE_STRING_H_
#define SNARK_RENDER_KML_LINE_STRING_H_

#include <string>
#include "./coordinates.h"

namespace snark { namespace render { namespace kml {

struct line_string
{
    boost::optional< std::string > altitude_mode;
    std::vector< kml::position > coordinates;
};

} } } // namespace snark { namespace render { namespace kml {

#endif // SNARK_RENDER_KML_LINE_STRING_H_
