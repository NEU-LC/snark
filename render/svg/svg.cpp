#include "svg.h"

namespace snark { namespace render { namespace svg {

std::string element::DEFAULT_ATTRIBUTES = std::string();
std::string element::DEFAULT_STYLE = std::string();

element::element( const std::string& style_ ) : attributes( DEFAULT_ATTRIBUTES ), style( DEFAULT_STYLE )
{
    if( style_.size() )
    {
        if( style.size() ) { style += ";"; }
        style += style_;
    }
}

std::ostream& operator<<( std::ostream& os, const element& e )
{
    if( e.attributes.size() ) { os << e.attributes; }
    if( e.style.size() ) { os << " style=\"" << e.style << "\""; }
    return os;
}

std::ostream& operator<<( std::ostream& os, const header& h )
{
    os << "<?xml version=\"1.0\"?>" << std::endl;
    if( h.css ) { os << "<?xml-stylesheet type=\"text/css\" href=\"" << *h.css << "\"?>" << std::endl; }
    return os << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">" << std::endl
        << "<svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\"" << (element)h 
        << " width=\"" << h.width << "\" height=\"" << h.height << "\""
        << " viewBox=\"" << h.viewbox << "\""
        << ">";
}

std::ostream& operator<<( std::ostream& os, const footer& f )
{
    return os << "</svg>";
}

std::string script::begin()
{
    return "<defs><script type=\"text/javascript\"><![CDATA[";
}

std::string script::end()
{
    return "]]></script></defs>";
}

std::ostream& operator<<( std::ostream& os, const script& s )
{
    return os << "<script type=\"text/javascript\" xlink:href=\"" << s.file << "\"></script>";
}

std::string style::begin()
{
    return "<defs><style type=\"text/css\"><![CDATA[";
}

std::string style::end()
{
    return "]]></style></defs>";
}

std::ostream& operator<<( std::ostream& os, const g& gg )
{
    return os << "<g" << (element)gg << ">";
}

std::string g::end()
{
    return "</g>";
}

double circle::DEFAULT_RADIUS = 5;

circle::circle( const circle& c, const std::string& colour ) : element( "fill:" + colour ), cx( c.cx ), cy( c.cy ), r( c.r )
{
}

std::ostream& operator<<( std::ostream& os, const circle& c )
{
    return os << "<circle"
        << (element)c
        << " cx=\"" << c.cx << "\""
        << " cy=\"" << c.cy << "\""
        << " r=\"" << c.r << "\""
        << "/>";
}

line::line( const double x1, const double y1, const double x2, const double y2, const std::string& colour ) : element( "stroke:" + colour ), x1( x1 ), y1( y1 ), x2( x2 ), y2( y2 )
{
}

line::line( const line& l, const std::string& colour ) : element( "stroke:" + colour ), x1( l.x1 ), y1( l.y1 ), x2( l.x2 ), y2( l.y2 )
{
}

std::ostream& operator<<( std::ostream& os, const line& l )
{
    return os << "<line"
        << (element)l
        << " x1=\"" << l.x1 << "\""
        << " y1=\"" << l.y1 << "\""
        << " x2=\"" << l.x2 << "\""
        << " y2=\"" << l.y2 << "\""
        << "/>";
}

static std::ostream& operator<<( std::ostream& os, const point& p )
{
    return os << p.x << ',' << p.y;
}

static std::ostream& operator<<( std::ostream& os, const std::vector< point >& points )
{
    if( points.empty() ) { return os; }
    os << " points=\"" << points[0];
    for( unsigned int i = 1; i < points.size(); ++i ) { os << ' ' << points[i]; }
    return os << "\"";
}

std::ostream& operator<<( std::ostream& os, const polyline& p )
{
    return os << "<polyline" << (element)p << p.points << "/>";
}

std::ostream& operator<<( std::ostream& os, const polygon& p )
{
    return os << "<polygon" << (element)p << p.points << "/>";
}

text::text( const text& t, const std::string& colour ) : element( "fill:" + colour ), x( t.x ), y( t.y ), value( t.value )
{
}

std::ostream& operator<<( std::ostream& os, const text& t )
{
    return os << "<text"
        << (element)t
        << " x=\"" << t.x << "\""
        << " y=\"" << t.y << "\""
        << ">" << t.value << "</text>";
}

} } } // namespace snark { namespace render { namespace svg {
