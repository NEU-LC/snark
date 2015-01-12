#include "svg.h"

namespace snark { namespace render { namespace svg {

std::ostream& operator<<( std::ostream& os, const header& h )
{
    os << "<?xml version=\"1.0\"?>" << std::endl;
    if( h.css ) { os << "<?xml-stylesheet type=\"text/css\" href=\"" << *h.css << "\"?>" << std::endl; }
    os  << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">" << std::endl
        << "<svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\""
        << " width=\"" << h.width << "\" height=\"" << h.height << "\""
        << " viewBox=\"" << h.viewbox_min_x << " " << h.viewbox_min_y << " " << h.viewbox_width << " " << h.viewbox_height << "\"";
    if( h.style ) { os << " style=\"" << *h.style << "\""; }
    return os << ">";
}

std::ostream& operator<<( std::ostream& os, const footer& f )
{
    return os << "</svg>";
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
    os << "<g";
    if( gg.klass ) { os << " class=\"" << *gg.klass << "\""; }
    if( gg.id ) { os << " id=\"" << *gg.id << "\""; }
    if( gg.style && gg.style->size() ) { os << " style=\"" << *gg.style << "\""; }
    return os << ">";
}

std::string g::close()
{
    return "</g>";
}

std::ostream& operator<<( std::ostream& os, const circle& c )
{
    return os << "<circle "
        << "cx=\"" << c.cx << "\" "
        << "cy=\"" << c.cy << "\" "
        << "r=\"" << c.r << "\" "
        << "/>";
}

std::ostream& operator<<( std::ostream& os, const line& l )
{
    return os << "<line "
        << "x1=\"" << l.x1 << "\" "
        << "y1=\"" << l.y1 << "\" "
        << "x2=\"" << l.x2 << "\" "
        << "y2=\"" << l.y2 << "\" "
        << "/>";
}

void polyline::add( const point& p )
{
    points.push_back( p );
}

static std::ostream& operator<<( std::ostream& os, const point& p )
{
    return os << p.x << ',' << p.y;
}

static std::ostream& operator<<( std::ostream& os, const std::vector< point >& points )
{
    if( points.empty() ) { return os; }
    os << points[0];
    for( unsigned int i = 1; i < points.size(); ++i ) { os << ' ' << points[i]; }
    return os;
}

std::ostream& operator<<( std::ostream& os, const polyline& p )
{
    return os << "<polyline points=\"" << p.points << "\"/>";
}

} } } // namespace snark { namespace render { namespace svg {
