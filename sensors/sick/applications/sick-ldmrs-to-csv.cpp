#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <snark/visiting/eigen.h>
#include <comma/name_value/map.h>
#include <snark/math/range_bearing_elevation.h>
#include <comma/string/string.h>
#include <snark/timing/ntp.h>
#include <comma/visiting/traits.h>
#include <snark/sensors/sick/protocol.h>

#include <fcntl.h>

struct csv_point
{
    boost::posix_time::ptime timestamp;
    ::Eigen::Matrix< double, 3, 1 > cartesian;
    snark::range_bearing_elevation polar;
    comma::uint32 layer;
    comma::uint32 echo;
    comma::uint32 what;
    double width;
    comma::uint32 scan;
};

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "take sick ldmrs data feed on stdin, output csv data to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage" << std::endl;
    std::cerr << "    netcat pantilt 1234 | sick-ldmrs-to-csv [<options>]" << std::endl;
    std::cerr << "    cat ldmrs.log | sick-ldmrs-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
    std::cerr << "    --binary,-b: output binary equivalent of csv" << std::endl;
    std::cerr << "    --echo <threshold>: output only echoes <= threshold (0-3); default: 3 (all)" << std::endl;
    std::cerr << "    --fields=<fields>: output only given fields" << std::endl;
    std::cerr << "        default: " << comma::join( comma::csv::names< csv_point >( false ), ',' ) << " (" << comma::csv::format::value< csv_point >() << ")" << std::endl;
    std::cerr << "        t: timestamp" << std::endl;
    std::cerr << "        x,y,z: cartesian coordinates in sensor frame" << std::endl;
    std::cerr << "        range,bearing,elevation or r,b,e: polar coordinates in sensor frame" << std::endl;
    std::cerr << "        id: laser id (or \"layer\": 0-3)" << std::endl;
    std::cerr << "        echo: echo number (0-2)" << std::endl;
    std::cerr << "        what: transparent (1), dust (2), dirt (8)" << std::endl;
    std::cerr << "        width: echo width in metres" << std::endl;
    std::cerr << "    --format: output binary format for given fields to stdout and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "author:" << std::endl;
    std::cerr << "    vsevolod vlaskine, v.vlaskine@acfr.usyd.edu.au" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

namespace comma { namespace visiting {

template <> struct traits< csv_point >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const csv_point& p, Visitor& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "cartesian", p.cartesian );
        v.apply( "polar", p.polar );
        v.apply( "layer", p.layer );
        v.apply( "echo", p.echo );
        v.apply( "what", p.what );
        v.apply( "width", p.width );
        v.apply( "scan", p.scan );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{    
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        unsigned int threshold = options.value( "--echo", 3 );
        comma::csv::options csv;
        csv.fields = options.value< std::string >( "--fields", "" );
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        for( std::size_t i = 0; i < v.size(); ++i ) // convenience shortcuts
        {
            if( v[i] == "r" ) { v[i] = "range"; }
            else if( v[i] == "b" ) { v[i] = "bearing"; }
            else if( v[i] == "e" ) { v[i] = "elevation"; }
            else if( v[i] == "id" ) { v[i] = "layer"; }
        }
        csv.fields = comma::join( v, ',' );
        if( options.exists( "--binary,-b" ) || options.exists( "--format" ) ) { csv.format( comma::csv::format::value< csv_point >( csv.fields, false ) ); }
        if( options.exists( "--format" ) ) { std::cout << csv.format().string(); return 0; }
        csv.full_xpath = false;
        comma::csv::output_stream< csv_point > ostream( std::cout, csv );
        snark::sick::ldmrs::protocol protocol( std::cin );
        comma::signal_flag isShutdown;
        while( !std::cin.eof() )
        {
            if( isShutdown ) { std::cerr << "sick-ldmrs-to-csv: caught signal, exit" << std::endl; return -1; }
            const snark::sick::ldmrs::scan_packet* p = protocol.readscan();
            if( p == NULL ) { std::cerr << "sick-ldmrs-to-csv: done" << std::endl; return 0; }
            std::size_t count = p->packet_scan.scan_header.points_count();
            snark::sick::ldmrs::scan::timestamps timestamps( p->packet_scan );
            csv_point point;
            point.scan = p->packet_scan.scan_header.measurement_number();
            for( std::size_t i = 0; i < count; ++i )
            {
                point.echo = p->packet_scan.points()[i].id.echo();
                if( point.echo > threshold ) { continue; }
                point.polar.range( double( p->packet_scan.points()[i].range() ) / 100 );
                point.polar.bearing( p->packet_scan.angle_as_radians( p->packet_scan.points()[i] ) );
                point.polar.elevation( p->packet_scan.points()[i].elevation() );
                point.cartesian = point.polar.to_cartesian();
                point.layer = p->packet_scan.points()[i].id.layer();
                point.what = p->packet_scan.points()[i].flags();
                point.width = double( p->packet_scan.points()[i].echo_pulse_width() ) / 100;
                point.timestamp = timestamps[i];
                ostream.write( point );
            }
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "sick-ldmrs-to-csv: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "sick-ldmrs-to-csv: unknown exception" << std::endl;
    }
    usage();
}
