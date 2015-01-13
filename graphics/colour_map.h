#ifndef SNARK_GRAPHICS_COLOUR_MAP_H_
#define SNARK_GRAPHICS_COLOUR_MAP_H_

#include <boost/array.hpp>
#include <comma/math/cyclic.h>

namespace snark { namespace graphics {

class colour_map // quick and dirty
{
    public:
        typedef boost::array< unsigned char, 3 > pixel;

        typedef boost::array< pixel, 256 > values;

        enum channels { red = 0, green = 1, blue = 2 };

        static values constant( unsigned char r, unsigned char g, unsigned char b );

        static values temperature( unsigned char offset_r, unsigned char offset_g );

        static values jet();

        static pixel contrast_to( const values& v );
};

} } // namespace snark { namespace graphics {

#endif //SNARK_GRAPHICS_COLOUR_MAP_H_
