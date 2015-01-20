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


#ifndef SNARK_RENDER_SVG_H
#define SNARK_RENDER_SVG_H

#include <iostream>
#include <string>
#include <vector>
#include <boost/optional/optional.hpp>

namespace snark { namespace render { namespace svg {

struct element
{
    static std::string DEFAULT_ATTRIBUTES;
    static std::string DEFAULT_STYLE;

    std::string attributes;
    std::string style;

    element() : attributes( DEFAULT_ATTRIBUTES ), style( DEFAULT_STYLE ) { }
    element( const std::string& style );

    friend std::ostream& operator<<( std::ostream& os, const element& e );
};

struct header : public element
{
    std::string width;
    std::string height;
    std::string viewbox;
    boost::optional< std::string > css;

    header() { }
    header( const std::string& width, const std::string &height, const std::string& viewbox, const boost::optional< std::string >& css )
        : width( width ), height( height ), viewbox( viewbox ), css( css )
    { }

    friend std::ostream& operator<<( std::ostream& os, const header& h );
};

struct footer
{
    friend std::ostream& operator<<( std::ostream& os, const footer& f );
};

struct script
{
    static std::string begin();
    static std::string end();

    std::string file;

    script( const std::string& file ) : file( file ) { }

    friend std::ostream& operator<<( std::ostream& os, const script& s );
};

struct style
{
    static std::string begin();
    static std::string end();
};

struct g : public element
{
    g() { }

    static std::string end();

    friend std::ostream& operator<<( std::ostream& os, const g& gg );
};

struct circle : public element
{
    static double DEFAULT_RADIUS;

    double cx;
    double cy;
    double r;

    circle() : r( DEFAULT_RADIUS ) { }
    circle( const double cx, const double cy, const double r ) : cx( cx ), cy( cy ), r( r ) { }
    circle( const circle& c, const std::string& colour );

    friend std::ostream& operator<<( std::ostream& os, const circle& c );
};

struct line : public element
{
    double x1;
    double y1;
    double x2;
    double y2;

    line() { }
    line( const double x1, const double y1, const double x2, const double y2 ) : x1( x1 ), y1( y1 ), x2( x2 ), y2( y2 ) { }
    line( const double x1, const double y1, const double x2, const double y2, const std::string& colour );
    line( const line& l, const std::string& colour );

    friend std::ostream& operator<<( std::ostream& os, const line& l );
};

struct point
{
    double x;
    double y;
    
    point() { }
    point( const double x, const double y ) : x( x ), y( y ) { }
};

struct polyline : public element
{
    std::vector< point > points;

    friend std::ostream& operator<<( std::ostream& os, const polyline& p );
};

struct polygon : public element
{
    std::vector< point > points;

    friend std::ostream& operator<<( std::ostream& os, const polygon& p );
};

struct text : public element
{
    double x;
    double y;
    std::string value;

    text() { }
    text( const text& t, const std::string & colour );

    friend std::ostream& operator<<( std::ostream& os, const text& t );
};

} } } // namespace snark { namespace render { namespace svg {

#endif // SNARK_RENDER_SVG_H
