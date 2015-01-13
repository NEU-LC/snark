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


#ifndef SNARK_GRAPHICS_COLOUR_MAPPER_HEADER_GUARD_
#define SNARK_GRAPHICS_COLOUR_MAPPER_HEADER_GUARD_

#include <snark/graphics/colour.h>
#include <snark/graphics/colour_map.h>

namespace snark { namespace graphics {

class colour_mapper
{
    public:
        colour_mapper() { }
        colour_mapper( const double from, const double to, const colour< unsigned char >& from_colour, const colour< unsigned char >& to_colour );
        colour_mapper( const double from, const double to, const colour_map::values& values );

        colour< unsigned char > map( const double scalar ) const;
        colour< unsigned char > operator()( const double scalar ) const { return map( scalar ); }

    private:
        double from;
        double to;
        double diff;
        boost::optional< colour_map::values > values;
        colour< unsigned char > from_colour;
        colour< unsigned char > to_colour;
};

colour_mapper::colour_mapper( double from, double to, const colour< unsigned char >& from_colour, const colour< unsigned char >& to_colour )
    : from( from )
    , to( to )
    , diff( to - from )
    , from_colour( from_colour )
    , to_colour( to_colour )
{
}

colour_mapper::colour_mapper( double from, double to, const colour_map::values& values )
    : from( from )
    , to( to )
    , diff( to - from )
    , values( values )
{
}

colour< unsigned char > colour_mapper::map( const double scalar ) const
{
    double v = ( scalar - from ) / diff;
    v = ( v < 0 ? 0 : v > 1 ? 1 : v );
    if( !values ) { return from_colour * ( 1 - v ) + to_colour * v; }
    unsigned int i = v * 255;
    return colour< unsigned char >( ( *values )[i][0], ( *values )[i][1], ( *values )[i][2] );
}

} } // namespace snark { namespace graphics {

#endif /*SNARK_GRAPHICS_COLOUR_MAPPER_HEADER_GUARD_*/

