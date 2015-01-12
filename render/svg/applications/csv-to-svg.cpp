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

void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | csv-to-svg [<what>] [<options>] > points.svg" << std::endl;
    std::cerr << std::endl;
    std::cerr << "what" << std::endl;
    std::cerr << "    header: output svg document header" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --width=<width>" << std::endl;
    std::cerr << "            --height=<height>" << std::endl;
    std::cerr << "            --view-min-x=<width>" << std::endl;
    std::cerr << "            --view-min-y=<height>" << std::endl;
    std::cerr << "            --view-width=<width>" << std::endl;
    std::cerr << "            --view-height=<height>" << std::endl;
    std::cerr << "            --style=<style>" << std::endl;
    std::cerr << "            --css=<path>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    footer: output svg document footer" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    style_begin: begin style definitions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    style_end: end style definitions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    points: special case of circle" << std::endl;
    std::cerr << "        fields: x,y" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            point-size,--weight=<size> (default: 5)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    circle" << std::endl;
    std::cerr << "        fields: x,y,r" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    line:" << std::endl;
    std::cerr << "        fields: x1,y1,x2,y2" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    lines: alternate implementation of polylines using <line>s" << std::endl;
    std::cerr << "        fields: x,y" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    polyline" << std::endl;
    std::cerr << "        fields: x,y" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --class <string>: implies --have-css" << std::endl;
    std::cerr << "    --id <string>" << std::endl;
    std::cerr << "    --style=\"<style>\" ;-seperated CSS name:value pairs" << std::endl;
    std::cerr << "          fill:<color>" << std::endl;
    std::cerr << "          stroke:<color>" << std::endl;
    std::cerr << "          stroke-width:<width>" << std::endl;
    std::cerr << "    --have-css: disables generating default styles for:" << std::endl;
    std::cerr << "                  <line>: stroke:black" << std::endl;
    std::cerr << "                  <polyline>: stroke:black;fill:none" << std::endl;
    std::cerr << "                specify when using external css" << std::endl;
    std::cerr << "    --colour,--color,-c <color>: specifies attribute for:" << std::endl;
    std::cerr << "                                     <circle>: fill" << std::endl; 
    std::cerr << "                                     <line>: stroke" << std::endl; 
    std::cerr << "                                     <polyline>: stroke" << std::endl; 
    std::cerr << "    --width <width>: specifies stroke-width attributes for <line> and <polyline>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: todo" << std::endl;
    std::cerr << std::endl;
    std::cerr << comma::contact_info << std::endl;
    std::cerr << std::endl;
}

namespace comma { namespace visiting {

} } // namespace comma { namespace visiting {

std::string make_style( const comma::command_line_options& options, const std::string& what )
{
    std::string style = options.value< std::string >( "--style", "" );
    if( what == "points" || what == "circle" )
    {
        if( options.exists( "--colour,--color,-c" ) )
        {
            if( style.size() ) { style += ";"; }
            style += "fill:" + options.value< std::string >( "--colour,--color,-c" );
        }
    }
    else if( what == "line" || what == "lines" || what == "polyline" )
    {
        if( options.exists( "--width" ) )
        {
            if( style.size() ) { style += ";"; }
            style += "stroke-width:" + options.value< std::string >( "--width" );
        }
        if( !options.exists( "--class" ) && !options.exists( "--have-css" ) )
        {
            if( style.find( "stroke:" ) == std::string::npos )
            {
                if( style.size() ) { style += ";"; }
                style += "stroke:" + options.value< std::string >( "--colour,--color,-c", "black" );
            }
            if( what == "polyline" )
            {
                if( style.find( "fill: " ) == std::string::npos )
                {
                    if( style.size() ) { style += ";"; }
                    style += "fill:none";
                }
            }
        }
    }
    return style;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        const std::vector< std::string >& unnamed = options.unnamed( "--verbose,-v,--have-css" , "-.*" );
        if( unnamed.size() != 1 ) { std::cerr << "csv-to-svg: expected one operation name" << ( unnamed.size() ? "; got " + comma::join( unnamed, ' ' ) : "" ) << std::endl; return 1; }
        std::string what = unnamed[0];
        comma::csv::options csv( options );
        if( what == "header" )
        {
            std::cout << snark::render::svg::header(
                  options.value< std::string >( "--width" )
                , options.value< std::string >( "--height" )
                , options.value< unsigned int >( "--viewbox-min-x", 0 )
                , options.value< unsigned int >( "--viewbox-min-y", 0 )
                , options.value< unsigned int >( "--viewbox-width" )
                , options.value< unsigned int >( "--viewbox-height" )
                , options.optional< std::string >( "--style" )
                , options.optional< std::string >( "--css" )
                ) << std::endl;
            return 0;
        }
        else if( what == "footer" ) { std::cout << snark::render::svg::footer() << std::endl; return 0; }
        else if( what == "style_begin" ) { std::cout << snark::render::svg::style::begin() << std::endl; return 0; }
        else if( what == "style_end" ) { std::cout << snark::render::svg::style::end() << std::endl; return 0; }

        std::cout << snark::render::svg::g(
              options.optional< std::string >( "--class" )
            , options.optional< std::string >( "--id" )
            , make_style( options, what )
            ) << std::endl;

        if( what == "points" )
        {
            double r = options.value< double >( "--point-size,--weight", 5 );
            comma::csv::input_stream< snark::render::svg::point > istream( std::cin, csv );
            while( std::cin.good() )
            {
                const snark::render::svg::point* p = istream.read();
                if( !p ) { break; }
                std::cout << snark::render::svg::circle( p->x, p->y, r ) << std::endl;
            }
        }
        else if( what == "circle" )
        {
            comma::csv::input_stream< snark::render::svg::circle > istream( std::cin, csv );
            while( std::cin.good() )
            {
                const snark::render::svg::circle* c = istream.read();
                if( !c ) { break; }
                std::cout << *c << std::endl;
            }
        }
        else if( what == "line" )
        {
            comma::csv::input_stream< snark::render::svg::line > istream( std::cin, csv );
            while( std::cin.good() )
            {
                const snark::render::svg::line* l = istream.read();
                if( !l ) { break; }
                std::cout << *l << std::endl;
            }
        }
        else if( what == "lines" )
        {
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
        else if( what == "polyline" )
        {
            comma::csv::input_stream< snark::render::svg::point > istream( std::cin, csv );
            snark::render::svg::polyline polyline;
            while( std::cin.good() )
            {
                const snark::render::svg::point* p = istream.read();
                if( !p ) { break; }
                polyline.add( *p );
            }
            if( polyline.points.size() ) { std::cout << polyline << std::endl; }
        }
        else
        {
            std::cerr << "csv-to-svg: unrecognized operation: \"" << what << "\"" << std::endl;
            return 1;
        }

        std::cout << snark::render::svg::g::close() << std::endl;

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
