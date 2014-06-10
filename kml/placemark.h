#ifndef SNARK_KML_PLACEMARK_H_
#define SNARK_KML_PLACEMARK_H_

#include <string>
#include <boost/optional.hpp>
#include "./line_string.h"
#include "./point.h"

namespace snark { namespace kml {

struct placemark
{
    boost::optional< std::string > name;
    boost::optional< std::string > description;
    boost::optional< std::string > style_url;
    boost::optional< std::string > altitude_mode;
    kml::point point;
    boost::optional< kml::line_string > line_string;
};

} } // namespace snark { namespace kml {

#endif // SNARK_KML_PLACEMARK_H_
