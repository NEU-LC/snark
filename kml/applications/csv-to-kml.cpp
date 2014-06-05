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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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
#include <comma/visiting/traits.h>

void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | csv-to-kml [<what>] [<options>] > points.kml" << std::endl;
    std::cerr << std::endl;
    std::cerr << "what" << std::endl;
    std::cerr << "    header: output kml header" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    footer: output kml footer" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    points: output points (todo)" << std::endl;
    std::cerr << "        fields: latitude,longitude,altitude; default: latitude,longitude" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    trajectory: output trajectory as line" << std::endl;
    std::cerr << "        fields: latitude,longitude,altitude; default: latitude,longitude,altitude" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --name=<name>; name" << std::endl;
    std::cerr << "    --style=<style url>; style url" << std::endl;
    std::cerr << "    --altitude-mode=<how>; absolute | relative; default: relative" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
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
}

namespace snark { namespace kml {

struct coordinates
{
    double latitude;
    double longitude;

    coordinates() : latitude( 0 ), longitude( 0 ) {}
};

struct position
{
    kml::coordinates coordinates;
    double altitude;

    position() : altitude( 0 ) {}
};

} } // namespace snark { namespace kml {

struct input
{
    snark::kml::position position;
};

namespace comma { namespace visiting {

template <> struct traits< snark::kml::coordinates > // quick and dirty?
{
    template< typename K, typename V > static void visit( const K&, snark::kml::coordinates& t, V& v )
    {
        v.apply( "latitude", t.latitude );
        v.apply( "longitude", t.longitude );
    }

    template< typename K, typename V > static void visit( const K&, const snark::kml::coordinates& t, V& v )
    {
        v.apply( "latitude", t.latitude );
        v.apply( "longitude", t.longitude );
    }
};

template <> struct traits< snark::kml::position > // quick and dirty?
{
    template< typename K, typename V > static void visit( const K&, snark::kml::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "altitude", t.altitude );
    }

    template< typename K, typename V > static void visit( const K&, const snark::kml::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "altitude", t.altitude );
    }
};

template <> struct traits< input > // quick and dirty?
{
    template< typename K, typename V > static void visit( const K&, input& t, V& v )
    {
        v.apply( "position", t.position );
    }

    template< typename K, typename V > static void visit( const K&, const input& t, V& v )
    {
        v.apply( "position", t.position );
    }
};

} } // namespace comma { namespace visiting {

static const std::string header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                                  "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\">\n"
                                  "<Document>";

static const std::string footer = "</Document>\n"
                                  "</kml>";

static std::string trajectory_placemark_header( const std::string& name, const std::string& style, const std::string& altitude_mode )
{
    std::ostringstream oss;
    oss << "<Placemark>\n";
    if( !name.empty() ) { oss << "    <name>" << name << "</name>\n"; }
    if( !style.empty() ) { oss << "    <styleUrl>" << style << "</styleUrl>\n"; }
    oss << "    <LineString>\n";
    oss << "        <altitudeMode>" << altitude_mode << "</altitudeMode>\n";
    oss << "        <coordinates>";
    return oss.str();
}

static const std::string trajectory_placemark_footer = "        </coordinates>\n"
                                                       "    </LineString>\n"
                                                       "</Placemark>";

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        const std::vector< std::string >& unnamed = options.unnamed( "--verbose,-v" , "-.*" );
        if( unnamed.size() > 1 ) { std::cerr << "csv-to-kml: expected one operation name; got " << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        std::string what = "points";
        if( !unnamed.empty() ) { what = unnamed[0]; }
        if( what == "points" )
        {
            std::cerr << "csv-to-kml: todo" << std::endl;
            return 1;
        }
        else if( what == "trajectory" )
        {
            std::string name = options.value< std::string >( "--name", "" );
            std::string style = options.value< std::string >( "--style", "" );
            std::string altitude_mode = options.value< std::string >( "--altitude-mode", "relative" );

            // todo: any non-zero fields -> cdata

            std::cout << trajectory_placemark_header( name, style, altitude_mode ) << std::endl;
            comma::csv::input_stream< input > istream( std::cin, csv );
            while( std::cin.good() )
            {
                const input* p = istream.read();
                if( !p ) { break; }
                std::cout << "            " << p->position.coordinates.longitude << "," << p->position.coordinates.latitude << "," << p->position.altitude << std::endl;
            }
            std::cout << trajectory_placemark_footer << std::endl;
            return 0;
        }
        else if( what == "header" )
        {
            std::cout << header << std::endl;
        }
        else if( what == "footer" )
        {
            std::cout << footer << std::endl;
        }
        return 0;
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
