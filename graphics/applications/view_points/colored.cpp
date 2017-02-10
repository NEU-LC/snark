// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


/// @author Vsevolod Vlaskine

#include <algorithm>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <qnamespace.h>
#include <comma/string/string.h>
#include "colored.h"

namespace snark { namespace graphics { namespace view {

static color_t color_from_name( const std::string& name )
{
         if( name == "black" )   { return color_t( Qt::black ); }
    else if( name == "white" )   { return color_t( Qt::white ); }
    else if( name == "red" )     { return color_t( Qt::red ); }
    else if( name == "green" )   { return color_t( Qt::green ); }
    else if( name == "blue" )    { return color_t( Qt::blue ); }
    else if( name == "yellow" )  { return color_t( Qt::yellow ); }
    else if( name == "magenta" ) { return color_t( Qt::magenta ); }
    else if( name == "cyan" )    { return color_t( Qt::cyan ); }
    else if( name == "grey" )    { return color_t( Qt::gray ); }
    else if( name == "pink" )    { return color_t( 255, 128, 128 ); }
    else if( name == "orange" )  { return color_t( 255, 128, 0 ); }
    else if( name == "salad" )   { return color_t( 128, 255, 128 ); }
    else if( name == "sky" )     { return color_t( 128, 128, 255 ); }
    else
    {
        const std::vector< std::string >& v = comma::split( name, ',' );
        try
        {
            switch( v.size() )
            {
                case 3: return color_t( boost::lexical_cast< int >( v[0] )
                                      , boost::lexical_cast< int >( v[1] )
                                      , boost::lexical_cast< int >( v[2] ) );
                case 4: return color_t( boost::lexical_cast< int >( v[0] )
                                      , boost::lexical_cast< int >( v[1] )
                                      , boost::lexical_cast< int >( v[2] )
                                      , boost::lexical_cast< int >( v[3] ) );
                default: COMMA_THROW( comma::exception, "expected color, got " << name );
            }
        }
        catch( ... )
        {
            COMMA_THROW( comma::exception, "expected color, got " << name );
        }
    }
}

static color_t multiply( const color_t& color, double scalar )
{
#if Qt3D_VERSION==1
    double red = scalar * color.redF();
    double green = scalar * color.greenF();
    double blue = scalar * color.blueF();
    double alpha = scalar * color.alphaF();
    return color_t::fromRgbF( red, green, blue, alpha );
#else
    return color * scalar;
#endif
}

#if Qt3D_VERSION==1
static unsigned int threshold( unsigned int i, unsigned int threshold )
{
    if( i > threshold )
    {
        return threshold;
    }
    else
    {
        return i;
    }
}
#endif

static color_t add( const color_t& a, const color_t& b )
{
#if Qt3D_VERSION==1
    unsigned int red = threshold( a.red() + b.red(), 255 );
    unsigned int green = threshold( a.green() + b.green(), 255 );
    unsigned int blue = threshold( a.blue() + b.blue(), 255 );
    unsigned int alpha = threshold( a.alpha() + b.alpha(), 255 );
    return color_t( red, green, blue, alpha );
#else
    return a + b;
#endif
}


fixed::fixed( const std::string& name ) : color_( color_from_name( name ) ) {}

color_t fixed::color( const Eigen::Vector3d&, comma::uint32, double, const color_t& ) const { return color_; }

by_height::by_height( double from
                    , double to
                    , const color_t& from_color
                    , const color_t& to_color
                    , bool cyclic
                    , bool linear
                    , bool sharp )
    : from( from )
    , to( to )
    , sum( to + from )
    , diff( to - from )
    , middle( from / sum )
    , from_color( from_color )
    , to_color( to_color )
    , cyclic( cyclic )
    , linear( linear )
    , sharp( sharp )
{
    average_color = color_t( from_color.red() / 2   + to_color.red() / 2
                           , from_color.green() / 2 + to_color.green() / 2
                           , from_color.blue() / 2  + to_color.blue() / 2
                           , from_color.alpha() / 2 + to_color.alpha() / 2 );
}

color_t by_height::color( const Eigen::Vector3d& point, comma::uint32, double, const color_t& ) const
{
    if( cyclic )
    {
        double v = std::fmod( point.z() - std::floor( point.z() / sum ) * sum, sum ) / sum;
        if( sharp ) { return v < middle ? from_color : to_color; }
        if( linear )
        {
            if( v < middle * 0.5 )
            {
                return add( multiply( average_color, 1 - v ), multiply( from_color, v ) );
            }
            else if( v < ( 1 + middle ) * 0.5 )
            {
                v = v - middle * 0.5;
                return add( multiply( from_color, ( 1 - v ) ), multiply( to_color, v ) );
            }
            else
            {
                v = v - ( 1 + middle ) * 0.5;
                return add( multiply( to_color, 1 - v ), multiply( average_color, v ) );
            }
        }
        else
        {
            if( v < middle )
            {
                v = v / middle - 0.5;
                v = v * v * 4;
                return add( multiply( from_color, 1 - v ), multiply( average_color, v ) );
            }
            else
            {
                v = ( v - middle ) / ( 1 - middle ) - 0.5;
                v = v * v * 4;
                return add( multiply( average_color, v ), multiply( to_color, 1 - v ) );
            }
        }
    }
    else
    {
        double v = ( point.z() - from ) / diff;
        v = ( v < 0 ? 0 : v > 1 ? 1 : v );
        return add( multiply( from_color, 1 - v ), multiply( to_color, v ) );
    }
}

by_scalar::by_scalar( double from, double to, const color_t& from_color, const color_t& to_color )
    : from( from )
    , to( to )
    , diff( to - from )
    , from_color( from_color )
    , to_color( to_color )
{
}

by_scalar::by_scalar( double from, double to, const snark::render::colour_map::values& map )
    : from( from )
    , to( to )
    , diff( to - from )
    , map( map )
{
}

color_t by_scalar::color( const Eigen::Vector3d& point, comma::uint32, double scalar, const color_t& ) const
{
    (void)point;
    double v = ( scalar - from ) / diff;
    v = ( v < 0 ? 0 : v > 1 ? 1 : v );
    if( !map ) { return add( multiply( from_color, 1 - v ), multiply( to_color, v ) ); }
    unsigned int i = v * 255;
    return color_t( ( *map )[i][0], ( *map )[i][1], ( *map )[i][2], 255 );
}

namespace impl {

static boost::array< unsigned int, 256 > colorInit()
{
    boost::array< unsigned int, 256 > a;
    for( std::size_t i = 0; i < 256; ++i ) { a[i] = i; }
    std::random_shuffle( a.begin(), a.end() );
    return a;
}
static unsigned char colorIndex = 23; // whatever
static boost::array< unsigned int, 256 > colorIndices = colorInit();

} // namespace impl {

by_id::by_id( const color_t& backgroundcolor )
    : background_( backgroundcolor )
    , has_scalar_( false )
{
}

by_id::by_id( const color_t& backgroundcolor
            , double from
            , double to )
    : background_( backgroundcolor )
    , has_scalar_( true )
    , from_( from )
    , diff_( to - from )
{
}

color_t by_id::color( const Eigen::Vector3d&, comma::uint32 id, double scalar, const color_t& ) const // quick and dirty
{
    static const float b = 0.95f;
    unsigned char i = ( id & 0xff ) + ( ( id & 0xff00 ) >> 16 );
    const float h = b * float( impl::colorIndices[ i ] ) / 255;
    const float a = 0.3f * float( 255 - impl::colorIndices[ i ] ) / 255;
    color_t color;
    switch( ( i * 13 + impl::colorIndex ) % 6 )
    {
#if Qt3D_VERSION==1
        case 0 : color = color_t::fromRgbF( b, h, a ); break;
        case 1 : color = color_t::fromRgbF( a, b, h ); break;
        case 2 : color = color_t::fromRgbF( h, a, b ); break;
        case 3 : color = color_t::fromRgbF( b, a, h ); break;
        case 4 : color = color_t::fromRgbF( h, b, a ); break;
        default: color = color_t::fromRgbF( a, h, b ); break;
#else
        case 0 : color = color_t( b, h, a ); break;
        case 1 : color = color_t( a, b, h ); break;
        case 2 : color = color_t( h, a, b ); break;
        case 3 : color = color_t( b, a, h ); break;
        case 4 : color = color_t( h, b, a ); break;
        default: color = color_t( a, h, b ); break;
#endif
    }
    if( !has_scalar_ ) { return color; }
    double v = ( scalar - from_ ) / diff_;
    v = ( v < 0 ? 0 : v > 1 ? 1 : v );
    return add( multiply( color, v ), multiply( background_, 1 - v ) );
}

color_t by_rgb::color( const Eigen::Vector3d& , comma::uint32, double, const color_t& c ) const
{
    return c;
}

colored* color_from_string( const std::string& s, const std::string& fields, const color_t& backgroundcolor )
{
    std::vector< std::string > f = comma::split( fields, ',' );
    bool hasId = false;
    for( unsigned int i = 0; !hasId && i < f.size(); ++i ) { hasId = f[i] == "id"; } // quick and dirty
    bool hasScalar = false;
    for( unsigned int i = 0; !hasScalar && i < f.size(); ++i ) { hasScalar = f[i] == "scalar"; } // quick and dirty
    bool r = false;
    bool g = false;
    bool b = false;
    bool a = false;
    for( unsigned int i = 0; !( r && g && b ) && !a && i < f.size(); ++i ) // quick and dirty
    {
        if( f[i] == "r" ) { r = true; }
        if( f[i] == "g" ) { g = true; }
        if( f[i] == "b" ) { b = true; }
        if( f[i] == "a" ) { a = true; }
    }
    colored* c;
    try
    {
        if( hasId )
        {
            if( hasScalar )
            {
                std::vector< std::string > v = comma::split( comma::split( s, ',' )[0], ':' );
                if( v.size() != 2 ) { COMMA_THROW( comma::exception, "expected range (-5:20), got " << s ); }
                double from = boost::lexical_cast< double >( v[0] );
                double to = boost::lexical_cast< double >( v[1] );
                c = new by_id( backgroundcolor, from, to );
            }
            else
            {
                c = new by_id( backgroundcolor );
            }
        }
        else if( r || g || b || a )
        {
            c = new by_rgb;
        }
        else if( hasScalar )
        {
            color_t from_color, to_color;
            double from = 0;
            double to = 0;
            boost::optional< snark::render::colour_map::values > map;

            std::vector< std::string > v = comma::split( s, ',' );
            switch( v.size() )
            {
                case 1:
                    {
                        std::vector< std::string > w = comma::split( v[0], ':' );
                        switch( w.size() )
                        {
                            case 1:
                                if( w[0] == "green" ) { map = snark::render::colour_map::constant( 0, 255, 0 ); }
                                else if( w[0] == "red" ) { map = snark::render::colour_map::constant( 255, 0, 0 ); }
                                else if( w[0] == "hot" ) { map = snark::render::colour_map::temperature( 96, 96 ); }
                                else if( w[0] == "jet" ) { map = snark::render::colour_map::jet(); }
                                else { COMMA_THROW( comma::exception, "expected colour map, got: " << s ); }
                                break;
                            case 2:
                                if( w[0][0] >= 'a' && w[0][0] <= 'z' )
                                {
                                    from_color = color_from_name( w[0] );
                                    to_color = color_from_name( w[1] );
                                }
                                else
                                {
                                    from = boost::lexical_cast< double >( w[0] );
                                    to = boost::lexical_cast< double >( w[1] );
                                }
                                break;
                            default:
                                COMMA_THROW( comma::exception, "expected range (e.g. -5:20,red:blue or 3:10), or colour map name, got " << s );
                        }
                        break;
                    }
                case 2:
                    {
                        std::vector< std::string > w = comma::split( v[0], ':' );
                        from = boost::lexical_cast< double >( w[0] );
                        to = boost::lexical_cast< double >( w[1] );
                        w = comma::split( v[1], ':' );
                        switch( w.size() )
                        {
                            case 1:
                                if( w[0] == "green" ) { map = snark::render::colour_map::constant( 0, 255, 0 ); }
                                else if( w[0] == "red" ) { map = snark::render::colour_map::constant( 255, 0, 0 ); }
                                else if( w[0] == "hot" ) { map = snark::render::colour_map::temperature( 96, 96 ); }
                                else if( w[0] == "jet" ) { map = snark::render::colour_map::jet(); }
                                else { COMMA_THROW( comma::exception, "expected colour map, got: " << s ); }
                                break;
                            case 2:
                                from_color = color_from_name( w[0] );
                                to_color = color_from_name( w[1] );
                                break;
                            default:
                                COMMA_THROW( comma::exception, "expected range (e.g. -5:20,red:blue or 3:10), or colour map name, got " << s );
                        }
                        break;
                    }
                default:
                    COMMA_THROW( comma::exception, "expected range (e.g. -5:20,red:blue or 3:10), or colour map name, got " << s );
            }
            c = map ? new by_scalar( from, to, *map )
                    : new by_scalar( from, to, from_color, to_color );
        }
        else
        {
            c = new fixed( s );
        }
    }
    catch( ... )
    {
        std::vector< std::string > v = comma::split( s, ',' );
        double from = 0;
        double to = 1;
        color_t from_color = color_from_name( "red" );
        color_t to_color = color_from_name( "yellow" );
        bool sharp = false;
        bool cyclic = false;
        bool linear = true;
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            if( v[i].empty() ) { continue; }
            else if( v[i] == "sharp" ) { sharp = true; }
            else if( v[i] == "smooth" ) { sharp = false; }
            else if( v[i] == "cyclic" ) { cyclic = true; from = 1; }
            else if( v[i] == "linear" ) { linear = true; }
            else if( v[i] == "quadratic" ) { linear = false; }
            else
            {
                std::vector< std::string > w = comma::split( v[i], ':' );
                if( w[0][0] >= 'a' && w[0][0] <= 'z' )
                {
                    from_color = color_from_name( w[0] );
                    to_color = color_from_name( w[1] );
                }
                else
                {
                    from = boost::lexical_cast< double >( w[0] );
                    to = boost::lexical_cast< double >( w[ w.size() == 1 ? 0 : 1 ] );
                }
            }
        }
        c = new by_height( from, to, from_color, to_color, cyclic, linear, sharp );
    }
    return c;
}

} } } // namespace snark { namespace graphics { namespace view {
