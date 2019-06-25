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

/// @author vsevolod vlaskine

#include <iostream>
#include <sstream>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/to_xml.h>
#include <comma/visiting/apply.h>
#include <comma/visiting/traits.h>
#include "../document.h"
#include "../traits.h"

void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | csv-to-kml [<what>] [<options>] > points.kml" << std::endl;
    std::cerr << std::endl;
    std::cerr << "what" << std::endl;
    std::cerr << "    header: output kml document header" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    footer: output kml document footer" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    points,placemarks: output placemarks" << std::endl;
    std::cerr << "        fields: latitude,longitude,name,style,image; default: latitude,longitude" << std::endl;
    std::cerr << "                values of any other non-empty field names will be added to" << std::endl;
    std::cerr << "                the text baloon" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    line-string,trajectory: output trajectory as line" << std::endl;
    std::cerr << "        fields: latitude,longitude,altitude; default: latitude,longitude" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --altitude-mode=<how>; absolute, relative; default: relative (todo)" << std::endl;
    std::cerr << "    --style=<properties>; placemark style as ';'-separated name-value pairs" << std::endl;
    std::cerr << "          line-color=<color>; hexadecimal aabbggrr, e.g: ff00ff00; style will be generated" << std::endl;
    std::cerr << "          line-width=<width>; pixels; default=1" << std::endl;
    std::cerr << "          icon=<url>; icon url; style will be generated" << std::endl;
    std::cerr << "          icon-scale=<scale>; icon scaling factor; default=1" << std::endl;
    std::cerr << "          url=<url>; style url; if style defined in the kml file itself," << std::endl;
    std::cerr << "                                prepend with hash, e.g: '#my_style'" << std::endl;
    std::cerr << "                                if style properties given, e.g. icon," << std::endl;
    std::cerr << "                                url required and has to start with #" << std::endl;
    std::cerr << "    --name=<name>; name" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    plot a trajectory: " << std::endl;
    std::cerr << std::endl;
    std::cerr << "    csv-to-kml header > test.kml" << std::endl;
    std::cerr << "    cat <<EOF | csv-to-kml trajectory >> test.kml" << std::endl;
    std::cerr << "-33.9461111111,151.177222222,21" << std::endl;
    std::cerr << "-33.942777778,151.180555556,1702.78073958" << std::endl;
    std::cerr << "-33.7108055556,150.299622222,22666.311661" << std::endl;
    std::cerr << "-33.8165928731,149.048263568,34000" << std::endl;
    std::cerr << "-33.8479,148.645530556,34000" << std::endl;
    std::cerr << "-33.8944124384,148.364675696,36000" << std::endl;
    std::cerr << "-34.2494444444,146.065358333,36000" << std::endl;
    std::cerr << "-34.2624725064,145.64835576,38000" << std::endl;
    std::cerr << "-34.1348777778,135.339813889,38000" << std::endl;
    std::cerr << "-34.495,131.5,38000" << std::endl;
    std::cerr << "EOF" << std::endl;
    std::cerr << "    csv-to-kml footer >> test.kml" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    apply styles: " << std::endl;
    std::cerr << std::endl;
    std::cerr << "    echo -e \"0,0\\n0,1\" | csv-to-kml trajectory --style='url=#my_style;line-color=ff00ff00;line-width=3'" << std::endl;
    std::cerr << std::endl;
}

struct input
{
    snark::render::kml::position position;
    std::string name;
    std::string style;
    std::string image;
};

struct style_t
{
    std::string line_color;
    unsigned int line_width;
    std::string icon;
    double icon_scale;
    std::string url;
    style_t() : line_width( 1 ), icon_scale( 1 ) {}
    bool empty() const { return icon.empty() && line_color.empty() && line_width == 1 && icon_scale == 1; }
};

namespace comma { namespace visiting {

template <> struct traits< input > // quick and dirty?
{
    template< typename K, typename V > static void visit( const K&, input& t, V& v )
    {
        v.apply( "position", t.position );
        v.apply( "name", t.name );
        v.apply( "style", t.style );
        v.apply( "image", t.image );
    }

    template< typename K, typename V > static void visit( const K&, const input& t, V& v )
    {
        v.apply( "position", t.position );
        v.apply( "name", t.name );
        v.apply( "style", t.style );
        v.apply( "image", t.image );
    }
};

template <> struct traits< style_t > // quick and dirty?
{
    template< typename K, typename V > static void visit( const K&, style_t& t, V& v )
    {
        v.apply( "line-color", t.line_color );
        v.apply( "line-width", t.line_width );
        v.apply( "icon", t.icon );
        v.apply( "icon-scale", t.icon_scale );
        v.apply( "url", t.url );
    }

    template< typename K, typename V > static void visit( const K&, const style_t& t, V& v )
    {
        v.apply( "line-color", t.line_color );
        v.apply( "line-width", t.line_width );
        v.apply( "icon", t.icon );
        v.apply( "icon-scale", t.icon_scale );
        v.apply( "url", t.url );
    }
};

} } // namespace comma { namespace visiting {

static std::vector< std::string > exclude_fields( const std::string& fields, const std::string& excluded )
{
    std::vector< std::string > v = comma::split( fields, ',' );
    std::vector< std::string > e = comma::split( excluded, ',' );
    for( std::size_t i = 0; i < e.size(); ++i )
    {
        for( std::size_t j = 0; j < v.size(); ++j ) { if( v[j] == e[i] ) { v[j] = ""; } }
    }
    bool empty = true;
    for( std::size_t j = 0; j < v.size() && empty; empty = v[j].empty(), ++j );
    if( empty ) { v.clear(); }
    return v;
}

static std::string description( const std::vector< std::string >& fields
                              , const std::vector< std::string >& values
                              , const std::string& image )
{
    std::ostringstream oss;
    oss << "<![CDATA[" << std::endl;
    oss << "    <table>" << std::endl;
    for( std::size_t i = 0; i < fields.size(); ++i )
    {
        if( !fields[i].empty() ) { oss << "        <tr><td><b>" << fields[i] << ":</b></td><td>" << values[i] << "</td></tr>" << std::endl; }
    }
    oss << "    </table>" << std::endl;
    if( !image.empty() ) { oss << "    <img src=\"" << image << "\"/>" << std::endl; }
    oss << "]]>" << std::endl;
    return oss.str();
}

static void output_style( const style_t& s ) // todo: quick and dirty: implement as a proper class
{
    if( s.empty() ) { return; }
    if( s.url.empty() || s.url[0] != '#' ) { std::cerr << "csv-to-kml: for non-empty style, expected style id, e.g. #my_style, got: \"" << s.url << "\"" << std::endl; }
    std::cout << "    <Style id=\"" << s.url.substr( 1 ) << "\">" << std::endl;
    std::cout << "        <IconStyle>" << std::endl;
    if( !s.icon.empty() ) { std::cout << "            <Icon><href>" << s.icon << "</href></Icon>" << std::endl; }
    std::cout << "            <scale>" << s.icon_scale << "</scale>" << std::endl;
    std::cout << "        </IconStyle>" << std::endl;
    std::cout << "        <LineStyle>" << std::endl;
    if( !s.line_color.empty() ) { std::cout << "            <color>" << s.line_color << "</color>" << std::endl; }
    std::cout << "            <width>" << s.line_width << "</width>" << std::endl;
    std::cout << "        </LineStyle>" << std::endl;
    std::cout << "    </Style>" << std::endl;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        csv.full_xpath = false;
        const std::vector< std::string >& unnamed = options.unnamed( "--verbose,-v" , "-.*" );
        if( unnamed.size() > 1 ) { std::cerr << "csv-to-kml: expected one operation name; got " << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        std::string what = "points";
        if( !unnamed.empty() ) { what = unnamed[0]; }
        boost::optional< std::string > name = options.optional< std::string >( "--name" );
        style_t style = comma::name_value::parser().get< style_t >( options.value< std::string >( "--style", "" ) );
        boost::optional< std::string > altitude_mode = options.optional< std::string >( "--altitude-mode" );
        output_style( style );
        if( what == "trajectory" || what == "line-string" )
        {
            snark::render::kml::document document;
            document.placemarks.resize( 1 );
            snark::render::kml::placemark& placemark = document.placemarks[0];
            placemark.line_string = snark::render::kml::line_string();
            placemark.name = name;
            if( !style.url.empty() ) { placemark.style_url = style.url; }
            placemark.line_string->altitude_mode = altitude_mode;
            if( csv.fields.empty() ) { csv.fields = "latitude,longitude"; }
            comma::csv::input_stream< input > istream( std::cin, csv );
            while( std::cin.good() )
            {
                const input* p = istream.read();
                if( !p ) { break; }
                placemark.line_string->coordinates.push_back( p->position );
            }
            comma::to_xml x( std::cout, 4, 1 );
            comma::visiting::apply( x ).to( document );
            return 0;
        }
        else if( what == "placemarks" || what == "points" )
        {
            snark::render::kml::document default_document;
            default_document.placemarks.resize( 1 );
            default_document.placemarks[0].name = name;
            if( !style.url.empty() ) { default_document.placemarks[0].style_url = style.url; }
            default_document.placemarks[0].altitude_mode = altitude_mode;
            if( csv.fields.empty() ) { csv.fields = "latitude,longitude"; }
            const std::vector< std::string > description_fields = exclude_fields( csv.fields, comma::join( comma::csv::names< input >( false ), ',' ) );
            comma::csv::input_stream< input > istream( std::cin, csv );
            while( std::cin.good() ) // we could collect placemarks in a single document, but on large files streaming is more memory-efficient
            {
                const input* p = istream.read();
                if( !p ) { break; }
                snark::render::kml::document document = default_document;
                document.placemarks[0].point = snark::render::kml::point();
                document.placemarks[0].point->coordinates = p->position;
                if( !p->name.empty() ) { document.placemarks[0].name = p->name; }
                if( !p->style.empty() ) { document.placemarks[0].style_url = p->style; }
                if( !description_fields.empty() || !p->image.empty() )
                {
                    document.placemarks[0].description = description( description_fields
                                                                    , csv.binary()
                                                                    ? comma::split( csv.format().bin_to_csv( istream.binary().last() ), ',' )
                                                                    : istream.ascii().last()
                                                                    , p->image );
                }
                comma::to_xml x( std::cout, 4, 1 );
                comma::visiting::apply( x ).to( document );
            }
            return 0;
        }
        if( what == "header" ) { std::cout << snark::render::kml::header() << snark::render::kml::document::tag(); return 0; }
        if( what == "footer" ) { std::cout  << snark::render::kml::document::gat() << snark::render::kml::footer(); return 0; }
        std::cerr << "csv-to-kml: expected an operation; got: \"" << what << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex )
    {
        std::cerr << "csv-to-kml: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "csv-to-kml: unknown exception" << std::endl;
    }
    return 1;
}
