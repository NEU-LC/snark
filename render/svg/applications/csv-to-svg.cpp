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

/// @author Vinny Do

#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/application/contact_info.h>
#include <comma/csv/stream.h>
#include <comma/visiting/traits.h>
#include <snark/render/svg/svg.h>
#include <snark/render/svg/traits.h>
#include <snark/graphics/colours.h>
#include <snark/graphics/colour_mapper.h>

void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | csv-to-svg [<what>] [<options>] > points.svg" << std::endl;
    std::cerr << std::endl;
    std::cerr << "what" << std::endl;
    std::cerr << "    header: output svg document header" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --width=<length>: viewport width of the svg document" << std::endl;
    std::cerr << "            --height=<length>: viewport height of the svg document" << std::endl;
    std::cerr << "            --viewbox-min-x=<width> (default: 0)" << std::endl;
    std::cerr << "            --viewbox-min-y=<height> (default: 0)" << std::endl;
    std::cerr << "            --viewbox-width=<width>" << std::endl;
    std::cerr << "            --viewbox-height=<height>" << std::endl;
    std::cerr << "                  svg viewBox attribute which specifies a rectangle in user space which should be mapped to the bounds of the viewport" << std::endl;
    std::cerr << "            --style=<style>" << std::endl;
    std::cerr << "            --css=<path-to-stylesheet>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    footer: output svg document footer" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    style_begin: begin style definitions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    style_end: end style definitions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    point, circle:" << std::endl;
    std::cerr << "        fields: x,y,r[,scalar] (default for point: x,y)" << std::endl;
    std::cerr << "        --point-size,--weight,--radius,-r=<size> (default: " << snark::render::svg::circle::DEFAULT_RADIUS << ")" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    line:" << std::endl;
    std::cerr << "        fields: x1,y1,x2,y2" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    lines: alternate implementation of polylines using <line>s" << std::endl;
    std::cerr << "        fields: x,y[,scalar]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    polyline" << std::endl;
    std::cerr << "        fields: x,y" << std::endl;
    std::cerr << "        --loop: connect the last point to the first" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --class=<string>: implies --have-css" << std::endl;
    std::cerr << "    --id=<string>" << std::endl;
    std::cerr << "    --style=\"<style>\" ;-seperated CSS name:value pairs e.g." << std::endl;
    std::cerr << "          fill:<colour>" << std::endl;
    std::cerr << "          stroke:<colour>" << std::endl;
    std::cerr << "          stroke-width:<width>" << std::endl;
    std::cerr << "    --transform=\"<transform-list>\": svg transformations" << std::endl;
    std::cerr << "    --have-css: disables generating the following default styles:" << std::endl;
    std::cerr << "                  <line>: stroke:black" << std::endl;
    std::cerr << "                  <polyline>: stroke:black;fill:none" << std::endl;
    std::cerr << "                specify when using external css" << std::endl;
    std::cerr << "    --colour,--color,-c=<colour>: any svg recognised colour" << std::endl;
    std::cerr << "                                  if scalar field present:" << std::endl;
    std::cerr << "                                      <min>:<max>,<from-colour>:<to-colour> (black, blue, green, cyan, red, magenta, yellow, white)" << std::endl; 
    std::cerr << "                                      <min>:<max>,<colourmap> (jet, hot)" << std::endl; 
    std::cerr << "    --width=<width>: specifies stroke-width attributes for <line> and <polyline>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: todo" << std::endl;
    std::cerr << std::endl;
    std::cerr << comma::contact_info << std::endl;
    std::cerr << std::endl;
}

template< typename T >
struct coloured : public T
{
    double scalar;
};

namespace comma { namespace visiting {

template < typename T > struct traits< coloured< T > >
{
    template< typename K, typename V > static void visit( const K& k, coloured< T >& t, V& v )
    {
        traits< T >::visit( k, t, v );
        v.apply( "scalar", t.scalar );
    }

    template< typename K, typename V > static void visit( const K& k, const coloured< T >& t, V& v )
    {
        traits< T >::visit( k, t, v );
        v.apply( "scalar", t.scalar );
    }
};

} } // namespace comma { namespace visiting {

boost::optional< std::string > make_style( const comma::command_line_options& options, const std::string& what, const bool parse_colour )
{
    std::string style = options.value< std::string >( "--style", "" );
    if( what == "point" || what == "circle" )
    {
        if( parse_colour && options.exists( "--colour,--color,-c" ) )
        {
            if( style.size() ) { style += ";"; }
            style += "fill:" + options.value< std::string >( "--colour,--color,-c" );
        }
    }
    else if( what == "line" || what == "lines" || what == "polyline" || what == "polygon" )
    {
        if( parse_colour && options.exists( "--colour,--color,-c" ) )
        {
            if( style.size() ) { style += ";"; }
            style += "stroke:" + options.value< std::string >( "--colour,--color,-c" );
        }
        if( options.exists( "--width" ) )
        {
            if( style.size() ) { style += ";"; }
            style += "stroke-width:" + options.value< std::string >( "--width" );
        }
        if( !options.exists( "--class" ) && !options.exists( "--have-css" ) )
        {
            if( parse_colour && style.find( "stroke:" ) == std::string::npos )
            {
                if( style.size() ) { style += ";"; }
                style += "stroke:" + options.value< std::string >( "--colour,--color,-c", "black" );
            }
            if( what == "polyline" || what == "polygon" )
            {
                if( style.find( "fill: " ) == std::string::npos )
                {
                    if( style.size() ) { style += ";"; }
                    style += "fill:none";
                }
            }
        }
    }
    if( style.empty() ) { return boost::none; }
    return style;
}

snark::graphics::colour_mapper parse_colour_map( const std::string& colour )
{
    double from = 0;
    double to = 1;
    snark::graphics::colour< unsigned char > from_colour = snark::graphics::colours< unsigned char >::magenta();
    snark::graphics::colour< unsigned char > to_colour = snark::graphics::colours< unsigned char >::cyan();
    boost::optional< snark::graphics::colour_map::values > map;
    std::vector< std::string > v = comma::split( colour, ',' );
    switch( v.size() )
    {
        case 0:
            break;
        case 2:
            {
                std::vector< std::string > w = comma::split( v[1], ':' );
                if( w.empty() || w.size() > 2 ) { COMMA_THROW( comma::exception, "expected <from-colour>:<to-colour> or <colourmap>; got " << v[1] ); }
                switch( w.size() )
                {
                    case 1:
                        if( w[0] == "jet" ) { map = snark::graphics::colour_map::jet(); }
                        else if( w[0] == "hot" ) { map = snark::graphics::colour_map::temperature( 96, 96 ); }
                        else { COMMA_THROW( comma::exception, "unknown colourmap: " << w[0] ); }
                        break;
                    case 2:
                        from_colour = snark::graphics::colours< unsigned char >::from_string( w[0] );
                        to_colour = snark::graphics::colours< unsigned char >::from_string( w[1] );
                        break;
                }
            }
        case 1:
            if( v[0].size() )
            {
                std::vector< std::string > w = comma::split( v[0], ':' );
                if( w.size() != 2 ) { COMMA_THROW( comma::exception, "invalid '--colour': expected <min>:<max>; got " << v[0] ); }
                from = boost::lexical_cast< double >( w[0] );
                to = boost::lexical_cast< double >( w[1] );
            }
            break;
        default: COMMA_THROW( comma::exception, "invalid '--colour'; got " << colour );
    }
    if( map ) { return snark::graphics::colour_mapper( from, to, *map ); }
    return snark::graphics::colour_mapper( from, to, from_colour, to_colour );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        const std::vector< std::string >& unnamed = options.unnamed( "--verbose,-v,--have-css,--loop" , "-.*" );
        if( unnamed.size() != 1 ) { std::cerr << "csv-to-svg: expected one operation name" << ( unnamed.size() ? "; got " + comma::join( unnamed, ' ' ) : "" ) << std::endl; return 1; }
        std::string what = unnamed[0];
        if( what == "header" )
        {
            std::cout << snark::render::svg::header(
                  options.value< std::string >( "--width" )
                , options.value< std::string >( "--height" )
                , options.value< double >( "--viewbox-min-x", 0 )
                , options.value< double >( "--viewbox-min-y", 0 )
                , options.value< double >( "--viewbox-width" )
                , options.value< double >( "--viewbox-height" )
                , options.optional< std::string >( "--style" )
                , options.optional< std::string >( "--css" )
                ) << std::endl;
            return 0;
        }
        else if( what == "footer" ) { std::cout << snark::render::svg::footer() << std::endl; return 0; }
        else if( what == "style_begin" ) { std::cout << snark::render::svg::style::begin() << std::endl; return 0; }
        else if( what == "style_end" ) { std::cout << snark::render::svg::style::end() << std::endl; return 0; }

        comma::csv::options csv( options );
        snark::graphics::colour_mapper colour_mapper;
        bool parse_colour = true;
        if( ( what == "point" || what == "circle" || what == "line" || what == "lines" ) && csv.has_field( "scalar" ) )
        {
            colour_mapper = parse_colour_map( options.value< std::string >( "--colour,--color,-c", "" ) );
            parse_colour = false;
        }

        std::cout << snark::render::svg::g(
                  options.optional< std::string >( "--class" )
                , options.optional< std::string >( "--id" )
                , make_style( options, what, parse_colour )
                , options.optional< std::string >( "--transform" )
                ) << std::endl;

        if( what == "point" || what == "circle" )
        { 
            boost::optional< double > r = options.optional< double >( "--point-size,--weight,--radius,-r" );
            if( r ) { snark::render::svg::circle::DEFAULT_RADIUS = *r; }
            if( csv.has_field( "scalar" ) )
            {
                if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< coloured< snark::render::svg::circle > >( csv.fields, true ) ); }
                comma::csv::input_stream< coloured< snark::render::svg::circle > > istream( std::cin, csv );
                while( std::cin.good() )
                {
                    const coloured< snark::render::svg::circle >* c = istream.read();
                    if( !c ) { break; }
                    std::cout << snark::render::svg::circle( *c, colour_mapper.map( c->scalar ).hex() ) << std::endl;
                }
            }
            else
            {
                if( csv.fields.empty() ) { csv.fields = what == "point" ? "x,y" : comma::join( comma::csv::names< snark::render::svg::circle >(), ',' ); }
                if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::circle >( csv.fields, true ) ); }
                comma::csv::input_stream< snark::render::svg::circle > istream( std::cin, csv );
                while( std::cin.good() )
                {
                    const snark::render::svg::circle* c = istream.read();
                    if( !c ) { break; }
                    std::cout << *c << std::endl;
                }
            }
        }
        else if( what == "line" )
        {
            if( csv.has_field( "scalar" ) )
            {
                if( options.exists( "--binary,-b" ) ) { csv.format( options.value< std::string >( "--binary,-b", comma::csv::format::value< coloured< snark::render::svg::line > >( csv.fields, true ) ) ); }
                comma::csv::input_stream< coloured< snark::render::svg::line > > istream( std::cin, csv );
                while( std::cin.good() )
                {
                    const coloured< snark::render::svg::line >* l = istream.read();
                    if( !l ) { break; }
                    std::cout << snark::render::svg::line( *l, colour_mapper.map( l->scalar ).hex() ) << std::endl;
                }
            }
            else
            {
                if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::line >(), ',' ); }
                if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::line >( csv.fields, true ) ); }
                comma::csv::input_stream< snark::render::svg::line > istream( std::cin, csv );
                while( std::cin.good() )
                {
                    const snark::render::svg::line* l = istream.read();
                    if( !l ) { break; }
                    std::cout << *l << std::endl;
                }
            }
        }
        else if( what == "lines" )
        {
            if( csv.has_field( "scalar" ) )
            {
                if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< coloured< snark::render::svg::point > >( csv.fields, true ) ); }
                comma::csv::input_stream< coloured< snark::render::svg::point > > istream( std::cin, csv );
                boost::optional< coloured< snark::render::svg::point > > prev;
                while( std::cin.good() )
                {
                    const coloured< snark::render::svg::point >* p = istream.read();
                    if( !p ) { break; }
                    if( prev ) { std::cout << snark::render::svg::line( prev->x, prev->y, p->x, p->y, colour_mapper.map( prev->scalar ).hex() ) << std::endl; }
                    prev = *p;
                }
            }
            else
            {
                if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::point >(), ',' ); }
                if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::point >( csv.fields, true ) ); }
                comma::csv::input_stream< snark::render::svg::point > istream( std::cin, csv );
                boost::optional< snark::render::svg::point > prev;
                while( std::cin.good() )
                {
                    const snark::render::svg::point* p = istream.read();
                    if( !p ) { break; }
                    if( prev ) { std::cout << snark::render::svg::line( prev->x, prev->y, p->x, p->y ) << std::endl; }
                    prev = *p;
                }
            }
        }
        else if( what == "polyline" )
        {
            if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::point >(), ',' ); }
            if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::point >( csv.fields, true ) ); }
            comma::csv::input_stream< snark::render::svg::point > istream( std::cin, csv );
            snark::render::svg::polyline polyline;
            while( std::cin.good() )
            {
                const snark::render::svg::point* p = istream.read();
                if( !p ) { break; }
                polyline.points.push_back( *p );
            }
            if( polyline.points.size() )
            {
                if( options.exists( "--loop" ) ) { polyline.points.push_back( polyline.points[0] ); }
                std::cout << polyline << std::endl;
            }
        }
        else if( what == "polygon" )
        {
            if( csv.fields.empty() ) { csv.fields = comma::join( comma::csv::names< snark::render::svg::point >(), ',' ); }
            if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< snark::render::svg::point >( csv.fields, true ) ); }
            comma::csv::input_stream< snark::render::svg::point > istream( std::cin, csv );
            snark::render::svg::polygon polygon;
            boost::optional< snark::render::svg::point > first;
            boost::optional< snark::render::svg::point > prev;
            while( std::cin.good() )
            {
                const snark::render::svg::point* p = istream.read();
                if( !p ) { break; }
                if( !first ) { first = *p; }
                polygon.points.push_back( *p );
            }
            if( polygon.points.size() ) { std::cout << polygon << std::endl; }
        }
        else if( what == "colour" )
        {
            colour_mapper = parse_colour_map( options.value< std::string >( "--colour,--color,-c", "" ) );
            std::string line;
            while( std::cin.good() )
            {
                std::getline( std::cin, line );
                if( line.empty() ) { break; }
                std::cout << colour_mapper.map( boost::lexical_cast< double >( line ) ).hex() << std::endl;
            }
        }
        else
        {
            std::cerr << "csv-to-svg: unrecognized operation: \"" << what << "\"" << std::endl;
            return 1;
        }

        std::cout << snark::render::svg::g::end() << std::endl;

        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "csv-to-svg: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "csv-to-svg: unknown exception" << std::endl;
    }
    return 1;
}
