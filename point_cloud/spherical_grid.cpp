#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "./spherical_grid.h"

#include <iostream>

namespace snark {


bearing_elevation_grid::index::index( const snark::rbe& begin, const snark::rbe& resolution )
    : begin_( begin )
    , resolution_( resolution )
{
}

bearing_elevation_grid::index::index( const snark::rbe& resolution )
    : begin_( 1, -M_PI, -M_PI / 2 )
    , resolution_( resolution )
{
}

bearing_elevation_grid::index::index( double bearing_begin, double elevation_begin, double bearing_resolution, double elevation_resolution )
    : begin_( 1, bearing_begin, elevation_begin )
    , resolution_( 1, bearing_resolution, elevation_resolution )
{
}

bearing_elevation_grid::index::index( double bearing_begin, double elevation_begin, double resolution )
    : begin_( 1, bearing_begin, elevation_begin )
    , resolution_( 1, resolution, resolution )
{
}

bearing_elevation_grid::index::index( double bearing_resolution, double elevation_resolution )
    : begin_( 1, -M_PI, -M_PI / 2 )
    , resolution_( 1, bearing_resolution, elevation_resolution )
{
}

bearing_elevation_grid::index::index( double resolution )
    : begin_( 1, -M_PI, -M_PI / 2 )
    , resolution_( 1, resolution, resolution )
{
}

const snark::rbe& bearing_elevation_grid::index::begin() const { return begin_; }

const snark::rbe& bearing_elevation_grid::index::resolution() const { return resolution_; }

bearing_elevation_grid::index::type bearing_elevation_grid::index::operator()( double bearing, double elevation ) const
{
    return operator()( snark::rbe( 1, bearing, elevation ) );
}

static std::size_t rounded( double d )
{
    double c = std::ceil( d );
    return static_cast< std::size_t >( ( c - d ) < 1e-6 ? c : std::floor( d ) );
}

bearing_elevation_grid::index::type bearing_elevation_grid::index::operator()( const snark::rbe& v ) const
{
    double db = v.b() - begin_.b();
    if( comma::math::less( db, 0 ) ) { db += M_PI * 2; } // quick and dirty
    double de = v.e() - begin_.e();
    if( comma::math::less( de, 0 ) ) { COMMA_THROW( comma::exception, "expected elevation greater than " << begin_.e() << "; got " << v.e() ); }
    index::type i = {{ rounded( db / resolution_.b() ), rounded( de / resolution_.e() ) }};
    return i;
}

snark::rbe bearing_elevation_grid::index::range_bearing_elevation( const type& i ) const
{
    return snark::rbe( 1, begin_.b() + resolution_.b() * i[0], begin_.e() + resolution_.e() * i[1] );
}

snark::rbe bearing_elevation_grid::index::rbe( const type& i ) const { return bearing_elevation_grid::index::range_bearing_elevation( i ); }

} // namespace snark {
