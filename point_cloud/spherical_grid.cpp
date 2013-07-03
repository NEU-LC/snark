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

bearing_elevation_grid::bounds bearing_elevation_grid::bearing_index::get_bounds( const double value ) const
{
    /// Normalise the input bearing
    double bearing(std::fmod(value, double(M_PI * 2)));
    if (bearing >= M_PI) { bearing -= ( M_PI * 2 ); }
    else if (bearing < -M_PI) { bearing += ( M_PI * 2 ); }

    bounds b;
    b.lower_index = operator()( bearing );

    /// Normalised lower value
    double lower(std::fmod(b.lower_index * resolution() + begin(), (double)(M_PI * 2)));
    if (lower >= M_PI) { lower -= (M_PI * 2); }
    else if (lower < -M_PI) { lower += (M_PI * 2); }

    if ( std::fabs(bearing - lower) <= 2*std::numeric_limits< double >::epsilon())
    {
        b.upper_index = b.lower_index;
        b.scaled_distance = 0;
    }
    else
    {
        b.upper_index = operator()( bearing + resolution());
        b.scaled_distance = ( bearing - lower ) / resolution();
    }

    return b;
}

double bearing_elevation_grid::bearing_index::value( int index ) const // quick and dirty
{
    return bearing_elevation( index_.begin().bearing() + index_.resolution().bearing() * index, 0 ).bearing();
}

double bearing_elevation_grid::elevation_index::value( int index ) const // quick and dirty
{
    return bearing_elevation( 0, index_.begin().elevation() + index_.resolution().elevation() * index ).elevation();
}

bearing_elevation_grid::bounds bearing_elevation_grid::elevation_index::get_bounds( const double value ) const
{
    /// Normalise the input elevation
    double elevation( std::fmod( value, double( M_PI * 2 ) ) );
    if( elevation > M_PI / 2 ) { elevation = M_PI - elevation; }
    else if ( elevation < -M_PI / 2 ) { elevation = -M_PI - elevation; }

    bounds b;
    b.lower_index = operator()(elevation);
    double lower = b.lower_index * resolution() + begin();

    if (comma::math::equal(elevation, lower))
    {
        b.upper_index = b.lower_index;
        b.scaled_distance = 0;
    }
    else
    {
        b.upper_index = b.lower_index + 1;
        b.scaled_distance = (elevation - lower) / resolution();
    }
    return b;
}

} // namespace snark {
