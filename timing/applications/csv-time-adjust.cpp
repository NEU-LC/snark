// This file is part of comma, a generic and flexible library
// // Copyright (c) 2011 The University of Sydney
// // All rights reserved.
// //
// // Redistribution and use in source and binary forms, with or without
// // modification, are permitted provided that the following conditions are met:
// // 1. Redistributions of source code must retain the above copyright
// //    notice, this list of conditions and the following disclaimer.
// // 2. Redistributions in binary form must reproduce the above copyright
// //    notice, this list of conditions and the following disclaimer in the
// //    documentation and/or other materials provided with the distribution.
// // 3. Neither the name of the University of Sydney nor the
// //    names of its contributors may be used to endorse or promote products
// //    derived from this software without specific prior written permission.
// //
// // NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// // GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// // HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// // WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// // MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// // DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// // BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// // WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// // OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// // IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/format.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>

#include "../clocked_time_stamp.h"

struct timestamp_io
{
    boost::posix_time::ptime timestamp;
    timestamp_io() {}
    timestamp_io( boost::posix_time::ptime const& i_timestamp ) : timestamp( i_timestamp ) {}
};

namespace comma { namespace visiting {

template <> struct traits< timestamp_io >
{
    template < typename K, typename V > static void visit( const K&, const timestamp_io& p, V& v ) { v.apply( "t", p.timestamp ); }
    template < typename K, typename V > static void visit( const K&, timestamp_io& p, V& v ) { v.apply( "t", p.timestamp ); }
};

} } // namespace comma { namespace visiting {

// todo
// DONE - --period, --threshold, and --reset: use seconds
// DONE - options per operation
// - github.com/acfr/snark/wiki: add to the list of applications
// - demo for Zhe
//

static void usage( bool )
{
    std::cerr << std::endl;
    std::cerr << "Takes timestamped csv stream and outputs the stream with the adjusted timestamps, to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage:" << std::endl;
    std::cerr << "    csv-time-adjust <operation> <options> < <input_stream>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Operation:" << std::endl;
    std::cerr << "    clocked: adjust for total deviation." << std::endl;
    std::cerr << "        Options:" << std::endl;
    std::cerr << "            --period=<seconds>: interval of the signal, in seconds." << std::endl;
    std::cerr << std::endl;
    std::cerr << "    periodic: use closest point as base and add periods for later points." << std::endl;
    std::cerr << "        Options:" << std::endl;
    std::cerr << "            --period=<seconds>: interval of the signal, in seconds." << std::endl;
    std::cerr << "            --reset=[<seconds>]: time interval to periodically reset adjusted time, in seconds." << std::endl;
    std::cerr << "            --threshold=[<seconds>]: upper bound for reseting adjusted timestamp, in seconds." << std::endl;
    // options for periodic: todo
    std::cerr << std::endl;
    std::cerr << "Common options:" << std::endl;
    std::cerr << "    --help,-h: display instructions on using this program and exit." << std::endl;
    std::cerr << "    --fields=[<fields>]: input fields; default t." << std::endl;
    std::cerr << "    --binary=[<format>]: binary format of input fields" << std::endl;
    std::cerr << "    --replace: flag to change the timestamps in place rather than appending the adjusted timestamps." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Examples:" << std::endl;
    std::cerr << "    cat timestamped_data.csv | csv-time-adjust clocked --fields=t,a,b --period=200" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    csv-time-adjust periodic --fields=t,a,b --period=200 --threshold=500 --reset=500 < timestamped_data.csv" << std::endl;
    std::cerr << std::endl;
}

bool check_mode_clocked_else_periodic( std::string const& i_mode_string )
{
    if( "clocked" == i_mode_string ) { return true; }
    if( "periodic" != i_mode_string ) { COMMA_THROW( comma::exception, "Expected either 'clocked' or 'periodic' for operation. Got: '" << i_mode_string << '"'); }
    return false;
}

long to_microseconds( double const i_seconds )
{
    return ( long )( i_seconds * 1000000 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        bool const is_inplace = options.exists( "--replace" );

        std::vector< std::string > unnamed = options.unnamed( "", "--binary,-b,--format,--fields,-f,--period,--threshold,--reset,--replace" );
        if( unnamed.empty() ) { std::cerr << comma::verbose.app_name() << ": please specify operations" << std::endl; exit( 1 ); }
        if( 1 < unnamed.size() ) { std::cerr << comma::verbose.app_name() << ": Only one operation allowed. Got multiple: " << comma::join( unnamed, ',' ) << std::endl; }
        bool const is_clocked_else_periodic = check_mode_clocked_else_periodic( unnamed.front() );

        boost::optional< snark::timing::clocked_time_stamp > clocked_timestamp;
        boost::optional< snark::timing::periodic_time_stamp > periodic_timestamp;

        if( is_clocked_else_periodic )
        {
            clocked_timestamp = snark::timing::clocked_time_stamp( boost::posix_time::microseconds( to_microseconds( options.value< double >("--period" ) ) ) );
        }
        else
        {
            periodic_timestamp = snark::timing::periodic_time_stamp( boost::posix_time::microseconds( to_microseconds( options.value< double >("--period" ) ) )
                                                                   , boost::posix_time::microseconds( to_microseconds( options.value< double >("--threshold" ) ) )
                                                                   , boost::posix_time::microseconds( to_microseconds( options.value< double >("--reset" ) ) ) );
        }

        comma::csv::options csv( options );
#ifdef WIN32
        if( csv.binary() ) { _setmode( _fileno( stdin ), _O_BINARY ); _setmode( _fileno( stdout ), _O_BINARY ); }
#endif
        comma::csv::input_stream< timestamp_io > istream( std::cin, csv );
        comma::csv::output_stream< timestamp_io > ostream( std::cout, csv );
        comma::csv::tied< timestamp_io, timestamp_io > tied( istream, ostream );
        while( std::cin.good() && !std::cin.eof() )
        {
            const timestamp_io* p = istream.read();
            if( !p ) { break; }
            timestamp_io q = *p;
            q.timestamp =  is_clocked_else_periodic  ? clocked_timestamp->adjusted( p->timestamp ) : periodic_timestamp->adjusted( p->timestamp );
            if( is_inplace )
            {
                if( csv.binary() ) { ostream.write( q, istream.binary().last() ); }
                else { ostream.write( q, istream.ascii().last() ); }
            }
            else { tied.append( q ); }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "csv-time-adjust: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "csv-time-adjust: unknown exception" << std::endl; }
    return 1;
} 

