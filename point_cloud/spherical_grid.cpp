#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "./spherical_grid.h"

#include <iostream>

namespace snark {


bearing_elevation_grid::index::index( const snark::bearing_elevation& begin, const snark::bearing_elevation& resolution )
    : begin_( begin )
    , resolution_( resolution )
{
}

bearing_elevation_grid::index::index( const snark::bearing_elevation& resolution )
    : begin_( -M_PI, -M_PI / 2 )
    , resolution_( resolution )
{
}

bearing_elevation_grid::index::index( double bearing_begin, double elevation_begin, double bearing_resolution, double elevation_resolution )
    : begin_( bearing_begin, elevation_begin )
    , resolution_( bearing_resolution, elevation_resolution )
{
}

bearing_elevation_grid::index::index( double bearing_begin, double elevation_begin, double resolution )
    : begin_( bearing_begin, elevation_begin )
    , resolution_( resolution, resolution )
{
}

bearing_elevation_grid::index::index( double bearing_resolution, double elevation_resolution )
    : begin_( -M_PI, -M_PI / 2 )
    , resolution_( bearing_resolution, elevation_resolution )
{
}

bearing_elevation_grid::index::index( double resolution )
    : begin_( -M_PI, -M_PI / 2 )
    , resolution_( resolution, resolution )
{
}

const snark::bearing_elevation& bearing_elevation_grid::index::begin() const { return begin_; }

const snark::bearing_elevation& bearing_elevation_grid::index::resolution() const { return resolution_; }

bearing_elevation_grid::index::type bearing_elevation_grid::index::operator()( double bearing, double elevation ) const
{
    return operator()( snark::bearing_elevation( bearing, elevation ) );
}

static std::size_t rounded( double d )
{
    double c = std::ceil( d );
    return static_cast< std::size_t >( ( c - d ) < 1e-6 ? c : std::floor( d ) );
}

bearing_elevation_grid::index::type bearing_elevation_grid::index::operator()( const snark::bearing_elevation& v ) const
{
    double db = v.b() - begin_.b();
    if( comma::math::less( db, 0 ) ) { db += M_PI * 2; } // quick and dirty
    double de = v.e() - begin_.e();
    if( comma::math::less( de, 0 ) ) { COMMA_THROW( comma::exception, "expected elevation greater than " << begin_.e() << "; got " << v.e() ); }
    index::type i = {{ rounded( db / resolution_.b() ), rounded( de / resolution_.e() ) }};
    return i;
}

snark::bearing_elevation bearing_elevation_grid::index::bearing_elevation( const type& i ) const
{
    return snark::bearing_elevation( begin_.b() + resolution_.b() * i[0], begin_.e() + resolution_.e() * i[1] );
}

} // namespace snark {
