#ifndef SNARK_RENDER_COLOUR_MAP_H_
#define SNARK_RENDER_COLOUR_MAP_H_

#include <boost/array.hpp>
#include <boost/optional/optional.hpp>
#include <comma/math/cyclic.h>
#include <snark/render/colour.h>

namespace snark { namespace render {

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

        colour_map() { }
        colour_map( const double from, const double to, const colour< unsigned char >& from_colour, const colour< unsigned char >& to_colour );
        colour_map( const double from, const double to, const values& v );

        colour< unsigned char > map( const double scalar ) const;
        colour< unsigned char > operator()( const double scalar ) const { return map( scalar ); }

    private:
        double from;
        double to;
        double diff;
        boost::optional< values > values_;
        colour< unsigned char > from_colour;
        colour< unsigned char > to_colour;
};

} } // namespace snark { namespace render {

#endif //SNARK_RENDER_COLOUR_MAP_H_
