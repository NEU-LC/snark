#include <boost/array.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <comma/csv/stream.h>
#include <comma/packed/byte.h>
#include <comma/packed/struct.h>
#include <comma/packed/string.h>
#include <comma/application/signal_flag.h>
#include "../../../timing/timestamped.h"
#include "../../../timing/traits.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take braille battery can bus data feed on stdin, output csv data to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "the feed consists of four stream identified by id:" << std::endl;
    std::cerr << "    0C0: battery status channel (e.g., battery state of charge)" << std::endl;
    std::cerr << "    0C1: info channel (e.g., voltage, current, etc)" << std::endl;
    std::cerr << "    0C2: charge channel, which lists various charge properties" << std::endl;
    std::cerr << "    0C4: trace channel (e.g., min and max temperatures)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage:" << std::endl;
    std::cerr << "    nc localhost <port> | braille-to-csv [<options>]" << std::endl;
    std::cerr << "    cat battery.log | braille-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
    std::cerr << "    --id: output csv data for the specified id (0C0, 0C1, 0C2, 0C4)" << std::endl;
    std::cerr << "    --type: output csv data for the specified channel type (status, info, charge, trace)" << std::endl;
    std::cerr << "    --list-ids: output available ids to stdout and exit" << std::endl;
    std::cerr << "    --list-fields,--output-fields: output available fields to stdout and exit" << std::endl;
    std::cerr << "    --fields=<fields>: output only given fields" << std::endl;
    std::cerr << "    --binary,-b: output binary equivalent of csv" << std::endl;
    std::cerr << "    --format: output binary format for given fields to stdout and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    for all channels:" << std::endl;
    std::cerr << "    cat battery.log | braille-to-csv" << std::endl;
    std::cerr << "    braille-to-csv --list-fields" << std::endl;
    std::cerr << "    cat battery.log | braille-to-csv --fields=t,state_of_charge,voltage" << std::endl;
    std::cerr << "    braille-to-csv --fields=t,state_of_charge,voltage --format" << std::endl;
    std::cerr << "    cat battery.log | braille-to-csv --fields=t,state_of_charge,voltage --binary > battery.bin" << std::endl;
    std::cerr << "    nc localhost 13000 | braille-to-csv --fields=t,state_of_charge,voltage --binary > battery.bin" << std::endl;
    std::cerr << "    candump can0,0C0:FF0 | csv-time-stamp | braille-to-csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    for a single channel:" << std::endl;
    std::cerr << "    braille-to-csv --list-ids" << std::endl;
    std::cerr << "    braille-to-csv --id=0C1 --list-fields" << std::endl;
    std::cerr << "    cat battery.log | braille-to-csv --id=0C1" << std::endl;
    std::cerr << "    braille-to-csv --id=0C1 --format" << std::endl;
    std::cerr << "    cat battery.log | braille-to-csv --id=0C1 --binary > 0C1.bin" << std::endl;
    std::cerr << "    cat battery.log | braille-to-csv --id=0C1 | name-value-from-csv $( braille-to-csv --id=0C1 --list-fields ) -n" << std::endl;
    std::cerr << std::endl;

    exit( -1 );
}

comma::signal_flag is_shutdown;

namespace braille {

struct header : public comma::packed::packed_struct< header, 18 >
{
    comma::packed::const_byte< ' ' > p1;
    comma::packed::const_byte< ' ' > p2;
    comma::packed::string< 4 > name;
    comma::packed::const_byte< ' ' > p3;
    comma::packed::const_byte< ' ' > p4;
    comma::packed::string< 3 > id;
    comma::packed::const_byte< ' ' > p5;
    comma::packed::const_byte< ' ' > p6;
    comma::packed::const_byte< ' ' > p7;
    comma::packed::const_byte< '[' > p8;
    comma::packed::byte number_of_bytes;
    comma::packed::const_byte< ']' > p9;
    comma::packed::const_byte< ' ' > p10;
};

struct byte : public comma::packed::packed_struct< byte, 3 >
{
    comma::packed::const_byte< ' ' > padding;
    comma::packed::ascii_hex< comma::uint16, 2 > value;
};

template < unsigned int Size >
struct packet : public comma::packed::packed_struct< packet< Size >, braille::header::size + Size * braille::byte::size >
{
    braille::header header;
    boost::array< braille::byte, Size > bytes;
};

struct packets
{
    struct status { enum { size = 8 }; };
    struct info { enum { size = 8 }; };
    struct charge { enum { size = 5 }; };
    struct trace { enum { size = 8 }; };
};

} // namespace braille {

struct output
{
    struct status
    {
        status() : state_of_charge( 0 ) {}
        status( const braille::packet< braille::packets::status::size >& p )
        {
            state_of_charge = p.bytes[0].value();
        }
        static std::string id() { return "0C0"; }
        double state_of_charge;
    };

    struct info
    {
        info() : voltage( 0 ), current( 0 ), max_discharge_current( 0 ), max_regenerative_current( 0 ) {}
        info( const braille::packet< braille::packets::info::size >& p )
        {
            double volts_per_value = 2;
            voltage = p.bytes[0].value() * volts_per_value;
            current = p.bytes[1].value() + p.bytes[2].value() * 256 - 32768;
            max_discharge_current = p.bytes[3].value() + p.bytes[4].value() * 256;
            max_regenerative_current = p.bytes[5].value() + p.bytes[7].value() * 256;
        }
        static std::string id() { return "0C1"; }
        double voltage;
        double current;
        double max_discharge_current;
        double max_regenerative_current;
    };

    struct charge
    {
        charge() : charge_current_set_point( 0 ), charge_voltage_set_point( 0 ) {}
        charge( const braille::packet< braille::packets::charge::size >& p )
        {
            charge_current_set_point = p.bytes[0].value();
            charge_voltage_set_point = p.bytes[1].value() + p.bytes[2].value() * 256;
        }
        static std::string id() { return "0C2"; }
        double charge_current_set_point;
        double charge_voltage_set_point;
    };

    struct trace
    {
        trace() : max_temperature( 0 ), min_temperature( 0 ) {}
        trace( const braille::packet< braille::packets::trace::size >& p )
        {
            double shift = 40;
            max_temperature = p.bytes[0].value() - shift;
            min_temperature = p.bytes[1].value() - shift;
        }
        static std::string id() { return "0C4"; }
        double max_temperature;
        double min_temperature;
    };

};

struct accumulated_output
{
    accumulated_output() {}
    output::status status;
    output::info info;
    output::charge charge;
    output::trace trace;
};

namespace comma { namespace visiting {

template <> struct traits< output::status >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, output::status& p, Visitor& v )
    {
        v.apply( "state_of_charge", p.state_of_charge );
    }
    template< typename Key, typename Visitor >
    static void visit( const Key&, const output::status& p, Visitor& v )
    {
        v.apply( "state_of_charge", p.state_of_charge );
    }
};

template <> struct traits< output::info >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, output::info& p, Visitor& v )
    {
        v.apply( "voltage", p.voltage );
        v.apply( "current", p.current );
        v.apply( "max_discharge_current", p.max_discharge_current );
        v.apply( "max_regenerative_current", p.max_regenerative_current );
    }
    template< typename Key, typename Visitor >
    static void visit( const Key&, const output::info& p, Visitor& v )
    {
        v.apply( "voltage", p.voltage );
        v.apply( "current", p.current );
        v.apply( "max_discharge_current", p.max_discharge_current );
        v.apply( "max_regenerative_current", p.max_regenerative_current );
    }
};

template <> struct traits< output::charge >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, output::charge& p, Visitor& v )
    {
        v.apply( "charge_current_set_point", p.charge_current_set_point );
        v.apply( "charge_voltage_set_point", p.charge_voltage_set_point );
    }
    template< typename Key, typename Visitor >
    static void visit( const Key&, const output::charge& p, Visitor& v )
    {
        v.apply( "charge_current_set_point", p.charge_current_set_point );
        v.apply( "charge_voltage_set_point", p.charge_voltage_set_point );
    }
};

template <> struct traits< output::trace >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, output::trace& p, Visitor& v )
    {
        v.apply( "max_temperature", p.max_temperature );
        v.apply( "min_temperature", p.min_temperature );
    }
    template< typename Key, typename Visitor >
    static void visit( const Key&, const output::trace& p, Visitor& v )
    {
        v.apply( "max_temperature", p.max_temperature );
        v.apply( "min_temperature", p.min_temperature );
    }
};

template <> struct traits< accumulated_output >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, accumulated_output& p, Visitor& v )
    {
        v.apply( "status", p.status );
        v.apply( "info", p.info );
        v.apply( "charge", p.charge );
        v.apply( "trace", p.trace );
    }
    template< typename Key, typename Visitor >
    static void visit( const Key&, const accumulated_output& p, Visitor& v )
    {
        v.apply( "status", p.status );
        v.apply( "info", p.info );
        v.apply( "charge", p.charge );
        v.apply( "trace", p.trace );
    }
};

} } // namespace comma { namespace visiting {

typedef snark::timestamped< std::string > input_t;

template < typename T, typename P >
void process( const comma::command_line_options& options, const std::string& output_id )
{
    typedef snark::timestamped< T > output_t;
    if( options.exists( "--list-fields,--output-fields" ) ) { std::cout << comma::join( comma::csv::names< output_t >( false ), ',' ) << std::endl; return; }
    comma::csv::options csv;
    csv.fields = options.value< std::string >( "--fields", "" );
    csv.full_xpath = false;
    if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< output_t >( csv.fields, false ) << std::endl; return; }
    if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< output_t >( csv.fields, false ) ); }
    comma::csv::ascii_input_stream< input_t > istream( std::cin, "" );
    comma::csv::output_stream< output_t > ostream( std::cout, csv );
    while( !is_shutdown && std::cin.good() && !std::cin.eof() )
    {
        const input_t* input = istream.read();
        if( !input ) { break; }
        std::string input_id = reinterpret_cast< const braille::header* >( &input->data[0] )->id();
        if( output_id != input_id )  { continue; }
        ostream.write( output_t( input->t, *reinterpret_cast< const braille::packet< P::size >* >( &input->data[0] ) ) );
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--list-ids" ) ) { std::cout << output::status::id() << "," << output::info::id() << "," << output::charge::id() << "," << output::trace::id() << std::endl; return 0; }
        options.assert_mutually_exclusive( "--id,--type" );
        std::string output_id;
        std::string type = options.value< std::string >( "--type", "" );
        if( !type.empty() )
        {
            if( type == "status" ) { output_id = output::status::id(); }
            else if( type == "info" ) { output_id = output::info::id(); }
            else if( type == "charge" ) { output_id = output::charge::id(); }
            else if( type == "trace" ) { output_id = output::trace::id(); }
            else { std::cerr << "braille-to-csv: expected type, got: \"" << type << "\"" << std::endl; return 1; }
        }
        output_id = options.value< std::string >( "--id", output_id );
        if( !output_id.empty() )
        {
            if(       output_id == output::status::id() ) { process< output::status, braille::packets::status >( options, output_id ); }
            else if ( output_id == output::info::id() ) { process< output::info, braille::packets::info >( options, output_id ); }
            else if ( output_id == output::charge::id() ) { process< output::charge, braille::packets::charge >( options, output_id ); }
            else if ( output_id == output::trace::id() ) { process< output::trace, braille::packets::trace >( options, output_id ); }
            else { std::cerr << "braille-to-csv: unexpected id: \"" << output_id << "\"" << std::endl; return 1; }
        }
        else
        {
            typedef snark::timestamped< accumulated_output > output_t;
            if( options.exists( "--list-fields,--output-fields" ) ) { std::cout << comma::join( comma::csv::names< output_t >( false ), ',' ) << std::endl; return 0; }
            comma::csv::options csv;
            csv.fields = options.value< std::string >( "--fields", "" );
            csv.full_xpath = false;
            if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< output_t >( csv.fields, false ) << std::endl; return 0; }
            if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< output_t >( csv.fields, false ) ); }
            comma::csv::ascii_input_stream< input_t > istream( std::cin, comma::csv::options() );
            comma::csv::output_stream< output_t > ostream( std::cout, csv );
            output_t output;
            while( !is_shutdown && std::cin.good() && !std::cin.eof() )
            {
                const input_t* input = istream.read();
                if( !input ) { break; }
                std::string input_id = reinterpret_cast< const braille::header* >( &input->data[0] )->id();
                output.t = input->t;
                if(      input_id == output::status::id() ) { output.data.status = *reinterpret_cast< const braille::packet< braille::packets::status::size >* >( &input->data[0] ); }
                else if( input_id == output::info::id() ) { output.data.info = *reinterpret_cast< const braille::packet< braille::packets::info::size >* >( &input->data[0] ); }
                else if( input_id == output::charge::id() ) { output.data.charge = *reinterpret_cast< const braille::packet< braille::packets::charge::size >* >( &input->data[0] ); }
                else if( input_id == output::trace::id() ) { output.data.trace = *reinterpret_cast< const braille::packet< braille::packets::trace::size >* >( &input->data[0] ); }
                else { std::cerr << "braille-to-csv: expected id, got: \"" << input_id << "\"" << std::endl; return 1; }
                ostream.write( output );
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "braille-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "braille-to-csv: unknown exception" << std::endl; }
    return 1;
}
