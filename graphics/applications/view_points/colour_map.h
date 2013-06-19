#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEW_POINTS_COLOUR_MAP_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEW_POINTS_COLOUR_MAP_H_

#include <boost/array.hpp>
#include <comma/math/cyclic.h>

namespace snark {

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

    private:
        static void jet_impl_( values& v, channels channel, int offset );
};

} // namespace snark {

#endif //SNARK_GRAPHICS_APPLICATIONS_VIEW_POINTS_COLOUR_MAP_H_
