#include "colour_map.h"

namespace snark { namespace render {

colour_map::values colour_map::constant( unsigned char r, unsigned char g, unsigned char b )
{
    values v;
    //for( unsigned int i = 0; i < 255; ++i ) { v[i][red] = ( i * r ) / 256; v[i][green] = ( i * g ) / 256; v[i][blue] = ( i * b ) / 256; }
    for( unsigned int i = 0; i < 256; ++i )
    {
        v[i][red] = ( i * r ) / 255; v[i][green] = ( i * g ) / 255; v[i][blue] = ( i * b ) / 255;
    }
    return v;
}

colour_map::values colour_map::temperature( unsigned char offset_r, unsigned char offset_g )
{
    values v;
    for( unsigned int i = 0; i < offset_r; ++i )
    {
        v[i][red] = ( i * 255 ) / offset_r;
        v[i][green] = 0;
        v[i][blue] = 0;
    }
    for( unsigned int i = offset_r; i < ( offset_r + offset_g ); ++i )
    {
        v[i][red] = 255;
        v[i][green] = ( ( i - offset_r ) * 255 ) / offset_g;
        v[i][blue] = 0;
    }
    for( unsigned int i = offset_r + offset_g; i < 256; ++i )
    {
        v[i][red] = 255;
        v[i][green] = 255;
        v[i][blue] = ( ( i - offset_r - offset_g ) * 255 ) / ( 256 - offset_r - offset_g );
    }
    return v;
}

static void jet_impl_( colour_map::values& v, colour_map::channels channel, int offset )
{
    for( unsigned int i = 0; i < 256; ++i ) { v[i][channel] = 0; }
    comma::math::cyclic< unsigned int > c( 0, 256 );
    c += offset;
    for( unsigned int i = 1; i < 64; ++i, ++c ) { v[ c() ][channel] = i * 4; }
    for( unsigned int i = 0; i < 65; ++i, ++c ) { v[ c() ][channel] = 255; }
    for( unsigned int i = 1; i < 64; ++i, ++c ) { v[ c() ][channel] = 255 - i * 4; }
}

colour_map::values colour_map::jet()
{
    values v;
    jet_impl_( v, red, 32 + 64 );
    jet_impl_( v, green, 32 );
    jet_impl_( v, blue, -32 );
    for( unsigned int i = 0; i < 64; ++i ) { v[i][red] = 0; }
    for( unsigned int i = 256 - 64; i < 256; ++i ) { v[i][blue] = 0; }
    return v;
}

colour_map::pixel colour_map::contrast_to( const values& v )
{
    pixel p;
    for( unsigned int i = 0; i < 3; ++i )
    {
        unsigned int average = 0;
        for( unsigned int k = 0; k < 256; ++k ) { average += v[k][i]; }
        average /= 256;
        p[i] = 255 - average;
    }
    return p;
}

colour_map::colour_map( double from, double to, const colour< unsigned char >& from_colour, const colour< unsigned char >& to_colour )
    : from( from )
    , to( to )
    , diff( to - from )
    , from_colour( from_colour )
    , to_colour( to_colour )
{
}

colour_map::colour_map( double from, double to, const values& v )
    : from( from )
    , to( to )
    , diff( to - from )
    , values_( v )
{
}

colour< unsigned char > colour_map::map( const double scalar ) const
{
    double v = ( scalar - from ) / diff;
    v = ( v < 0 ? 0 : v > 1 ? 1 : v );
    if( !values_ ) { return from_colour * ( 1 - v ) + to_colour * v; }
    unsigned int i = v * 255;
    return colour< unsigned char >( ( *values_ )[i][0], ( *values_ )[i][1], ( *values_ )[i][2] );
}

} } // namespace snark { namespace render {
