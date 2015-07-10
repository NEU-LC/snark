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
#include <comma/application/command_line_options.h>
#include <comma/csv/impl/fieldwise.h>
#include <comma/csv/stream.h>
#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../math/spherical_geometry/traits.h"
#include "../../../timing/timestamped.h"
#include "../../../timing/traits.h"
#include "../messages.h"
#include "../string.h"
#include "../traits.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "convert nmea data to csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: netcat localhost 12345 | nmea-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: output help; --help --verbose: more help" << std::endl;
    std::cerr << "    --fields=<fields>: output fields" << std::endl;
    std::cerr << "    --output-all,--all: if present, output records on every gps update," << std::endl;
    std::cerr << "                        even if values of output fields have not changed" << std::endl;
    std::cerr << "    --verbose,-v: more output to stderr" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields" << std::endl;
    std::cerr << "    default: position" << std::endl;
    std::cerr << "    todo" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << std::endl << "binary options" << std::endl << comma::csv::options::usage() << std::endl; }
    exit( 0 );
}

struct position
{
    snark::spherical::coordinates coordinates;
    double z;
    
    position() : z( 0 ) {}
    position( const snark::spherical::coordinates& coordinates, double z ) : coordinates( coordinates ), z( z ) {}
};

struct orientation
{
    double roll;
    double pitch;
    double yaw;
    
    orientation() : roll( 0 ), pitch( 0 ), yaw( 0 ) {}
    orientation( double roll, double pitch, double yaw ) : roll( roll ), pitch( pitch ), yaw( yaw ) {}
};

struct output
{
    struct data
    {
        ::position position;
        ::orientation orientation;
    };
    
    typedef snark::timestamped< data > type;
};

namespace comma { namespace visiting {

template <> struct traits< position >
{
    template < typename Key, class Visitor > static void visit( const Key&, position& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "z", p.z );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const position& p, Visitor& v )
    {
        v.apply( "coordinates", p.coordinates );
        v.apply( "z", p.z );
    }
};
    
template <> struct traits< orientation >
{
    template < typename Key, class Visitor > static void visit( const Key&, orientation& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const orientation& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
};
    
template <> struct traits< output::data >
{
    template < typename Key, class Visitor > static void visit( const Key&, output::data& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const output::data& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
    }
};

} } // namespace comma { namespace visiting {

static comma::csv::fieldwise make_fieldwise( const comma::csv::options& csv )
{
    std::vector< std::string > v = comma::split( csv.fields, ',' );
    for( unsigned int i = 0; i < v.size(); ++i ) { if( v[i] == "t" ) { v[i] = ""; } }
    comma::csv::options c = csv;
    c.fields = comma::join( v, ',' );
    return comma::csv::fieldwise( output::type(), c );
}

static void output( const comma::csv::fieldwise& fieldwise
                  , const output::type& v
                  , comma::csv::output_stream< output::type >& os )
{
    static std::string last;
    std::string current;
    if( os.is_binary() )
    {
        current.resize( os.binary().binary().format().size() );
        os.binary().binary().put( v, &current[0] );
        if( !fieldwise.binary().equal( &last[0], &current[0] ) ) { std::cout.write( &current[0], os.binary().binary().format().size() ); }
    }
    else
    {
        os.ascii().ascii().put( v, current );
        if( !fieldwise.ascii().equal( last, current ) ) { std::cout << current << std::endl; }
    }
    last = current;
}

using namespace snark;

void handle( const nmea::message< nmea::messages::gpgga >& m )
{
    // todo
}

template < typename T > void handle( const nmea::string& s )
{
    static nmea::string::as< T > m;
    handle( m.from( s ) );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        bool output_all = options.exists( "--output-all,--all" );
        bool verbose = options.exists( "--verbose,-v" );
        
        // todo? --no-timestamp?
        
        comma::csv::options csv( options );
        comma::csv::fieldwise fieldwise = make_fieldwise( csv ); // todo: plug in
        comma::csv::output_stream< output::type > os( std::cout, csv );
        std::string last;
        while( std::cin.good() )
        {
            std::string line;
            std::getline( std::cin, line );
            if( line.empty() ) { continue; }
            nmea::string s( line );
            if( !s.valid() ) { if( verbose ) { std::cerr << "nmea-to-csv: invalid nmea string: \"" << line << "\"" << std::endl; } continue; }
            
            // todo: if( !s.complete() ) { continue; }
            
            if( s.type() == "GPGGA" ) { handle< nmea::message< nmea::messages::gpgga > >( s ); }
            
            // todo
            
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "trimble-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "trimble-to-csv: unknown exception" << std::endl; }
    return 1;
}
