#ifndef SNARK_RENDER_SVG_H
#define SNARK_RENDER_SVG_H

#include <iostream>
#include <string>
#include <vector>
#include <boost/optional/optional.hpp>

namespace snark { namespace render { namespace svg {

struct header
{
    std::string width;
    std::string height;
    unsigned int viewbox_min_x;
    unsigned int viewbox_min_y;
    unsigned int viewbox_width;
    unsigned int viewbox_height;
    boost::optional< std::string > style;
    boost::optional< std::string > css;

    header() { }
    header( const std::string& width, const std::string &height, const unsigned int viewbox_min_x, const unsigned int viewbox_min_y, const unsigned int viewbox_width, const unsigned int viewbox_height, const boost::optional< std::string >& style, const boost::optional< std::string >& css ) :
          width( width )
        , height( height )
        , viewbox_min_x( viewbox_min_x )
        , viewbox_min_y( viewbox_min_y )
        , viewbox_width( viewbox_width )
        , viewbox_height( viewbox_height )
        , style( style )
        , css( css )
    { }

    friend std::ostream& operator<<( std::ostream& os, const header& h );
};

struct footer
{
    friend std::ostream& operator<<( std::ostream& os, const footer& f );
};

struct style
{
    static std::string begin();
    static std::string end();
};

struct element
{
    boost::optional< std::string > klass;
    boost::optional< std::string > id;
    boost::optional< std::string > style;

    element() { }
    element( const boost::optional< std::string >& klass, const boost::optional< std::string >& id, const boost::optional< std::string >& style ) : klass( klass ), id( id ), style( style ) { }
};

struct g : public element
{
    g( const boost::optional< std::string >& klass, const boost::optional< std::string >& id, const boost::optional< std::string >& style ) : element( klass, id, style ) { }

    static std::string close();

    friend std::ostream& operator<<( std::ostream& os, const g& gg );
};

struct circle
{
    double cx;
    double cy;
    double r;

    circle() { }
    circle( const double cx, const double cy, const double r ) : cx( cx ), cy( cy ), r( r ) { }

    friend std::ostream& operator<<( std::ostream& os, const circle& c );
};

struct line
{
    double x1;
    double y1;
    double x2;
    double y2;

    line() { }
    line( const double x1, const double y1, const double x2, const double y2 ) : x1( x1 ), y1( y1 ), x2( x2 ), y2( y2 ) { }

    friend std::ostream& operator<<( std::ostream& os, const line& l );
};

struct point
{
    double x;
    double y;
};

struct polyline
{
    std::vector< point > points;

    void add( const point& p );

    friend std::ostream& operator<<( std::ostream& os, const polyline& p );
};

} } } // namespace snark { namespace render { namespace svg {

#endif // SNARK_RENDER_SVG_H
