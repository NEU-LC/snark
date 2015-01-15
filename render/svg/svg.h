#ifndef SNARK_RENDER_SVG_H
#define SNARK_RENDER_SVG_H

#include <iostream>
#include <string>
#include <vector>
#include <boost/optional/optional.hpp>

namespace snark { namespace render { namespace svg {

struct element
{
    boost::optional< std::string > klass;
    boost::optional< std::string > id;
    boost::optional< std::string > style;
    boost::optional< std::string > transform;

    element() { }
    element( const boost::optional< std::string >& style ) : style( style ) { }
    element( const boost::optional< std::string >& klass, const boost::optional< std::string >& id, const boost::optional< std::string >& style, const boost::optional< std::string >& transform ) : klass( klass ), id( id ), style( style ), transform( transform ) { }

    friend std::ostream& operator<<( std::ostream& os, const element& e );
};

struct header : public element
{
    std::string width;
    std::string height;
    double viewbox_min_x;
    double viewbox_min_y;
    double viewbox_width;
    double viewbox_height;
    boost::optional< std::string > css;

    header() { }
    header( const std::string& width, const std::string &height, const double viewbox_min_x, const double viewbox_min_y, const double viewbox_width, const double viewbox_height, const boost::optional< std::string >& style, const boost::optional< std::string >& css ) :
          element( style )
        , width( width )
        , height( height )
        , viewbox_min_x( viewbox_min_x )
        , viewbox_min_y( viewbox_min_y )
        , viewbox_width( viewbox_width )
        , viewbox_height( viewbox_height )
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

struct g : public element
{
    g( const boost::optional< std::string >& klass, const boost::optional< std::string >& id, const boost::optional< std::string >& style, const boost::optional< std::string >& transform ) : element( klass, id, style, transform ) { }

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
};

struct polyline
{
    std::vector< point > points;

    void add( const point& p );

    friend std::ostream& operator<<( std::ostream& os, const polyline& p );
};

} } } // namespace snark { namespace render { namespace svg {

#endif // SNARK_RENDER_SVG_H
