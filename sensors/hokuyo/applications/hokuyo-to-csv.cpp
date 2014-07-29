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


#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include <snark/sensors/hokuyo/message.h>
#include <snark/sensors/hokuyo/traits.h>
#include "../sensors.h"

const char* name() { return "hokuyo-to-csv: "; }

namespace hok = snark::hokuyo;


class app_exit
{
    std::ostream& oss_;
public:
    app_exit( std::ostream& oss ) : oss_( oss ) {}
    ~app_exit()
    {
        const hok::state_command stop( "QT" );
        oss_.write( stop.data(), hok::state_command::size );
        oss_.flush();
    }
};

struct data_point
{
    data_point() : timestamp( boost::posix_time::microsec_clock::local_time() ), 
        x(0), y(0), z(0), intensity(0), bearing(0), elevation(0) {}
    
    bool is_nan() const { return ( x == 0 && y == 0 && z == 0 ); }
    
    /// Set the data point
    /// distance in meters, bearing in (radians)
    void set( double distance, double intensity, double bearing );
    
    boost::posix_time::ptime timestamp;
    double x;
    double y;
    double z;
    double distance; // meters
    double intensity;   
    double bearing; //radians
    double elevation;   // radians
};


void data_point::set(double distance, double intensity, double bearing)
{
    if( distance == hok::ust_10lx::distance_nan || distance <= ( hok::ust_10lx::distance_min / 1000.0 ) )
    {
        // timestamp stays the same
        x = 0;
        y = 0;
        z = 0; 
        intensity = 0;
        bearing = 0;
        elevation = 0;
        return;
    }
    
    this->distance = distance;
    this->intensity = intensity;
    this->bearing = bearing;
    // timestamp stays the same
    x = distance / std::sin( bearing );
    y = distance / std::cos( bearing );
    // z = 0;
}


namespace comma { namespace visiting {
    
template < > struct traits< data_point >
{
    
    template< typename K, typename V > static void visit( const K& k, const data_point& t, V& v )
    {
        v.apply( "timestamp", t.timestamp );
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
        v.apply( "distance", t.distance );
        v.apply( "intensity", t.intensity );
        v.apply( "bearing", t.bearing );
        v.apply( "elevation", t.elevation );
    }
};
    
} } // namespace comma { namespace visiting {

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "turns on the laser scanner and broad cast laser data." << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage" << std::endl;
    std::cerr << "    socat tcp:hokuyo-laser:port - | hokuyo-to-csv --publish <tcp:port|pipe> [ --fields t,x,y,z,intensity,range,bearing ]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
    std::cerr << "    --binary,-b: output binary equivalent of csv" << std::endl;
    std::cerr << "    --fields=<fields>: output only given fields" << std::endl;
//     std::cerr << "        default: " << comma::join( comma::csv::names< csv_point >( false ), ',' ) << " (" << comma::csv::format::value< csv_point >() << ")" << std::endl;
    std::cerr << "        t: timestamp" << std::endl;
    std::cerr << "        x,y,z: cartesian coordinates in sensor frame, where <0,0,0> is no data" << std::endl;
    std::cerr << "        i: intensity of the data point." << std::endl;
    std::cerr << "        range,bearing or r,b: polar coordinates in sensor frame" << std::endl;
    std::cerr << "    --format: output binary format for given fields to stdout and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Output format:" << std::endl;
    comma::csv::binary< data_point > binary( "", "" );
    std::cerr << "   format: " << binary.format().string() << " total size is " << binary.format().size() << " bytes" << std::endl;
    std::vector< std::string > names = comma::csv::names< data_point >();
    std::cerr << "   fields: " << comma::join( names, ','  ) << " number of fields: " << names.size() << std::endl;
    std::cerr << std::endl;
    std::cerr << "author:" << std::endl;
    std::cerr << "    dewey nguyen, duynii@gmail.com" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}


static bool is_binary = false;
static std::string output_fields = "";

void output_data( const data_point& point, comma::io::publisher& publisher )
{
    if( is_binary )
    {
        comma::csv::options csv;
        csv.fields = output_fields;
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        for( std::size_t i = 0; i < v.size(); ++i ) // convenience shortcuts
        {
            if( v[i] == "i" ) { v[i] = "intensity"; }
            else if( v[i] == "d" ) { v[i] = "distance"; }
            else if( v[i] == "b" ) { v[i] = "bearing"; }
            else if( v[i] == "e" ) { v[i] = "elevation"; }
            else if( v[i] == "t" ) { v[i] = "timestamp"; }
        }
        csv.fields = comma::join( v, ',' );
        csv.full_xpath = false;
        // see sick-ldmrs-to-csv
        csv.format( comma::csv::format::value< data_point >( csv.fields, false ) );
        
        static comma::csv::binary< data_point > binary( csv );
        static std::vector<char> line( binary.format().size() );
        binary.put( point, line.data() );
        publisher.write( line.data(), line.size());
    }
    else
    {
        static std::string tmp;
        static comma::csv::ascii< data_point > ascii( output_fields );
        publisher << ( ascii.put( point, tmp ) + "\n" );
    }
}

int main( int ac, char** av )
{
    comma::signal_flag signaled;

    comma::command_line_options options( ac, av );
    
    comma::io::istream input( "-", comma::io::mode::binary );
    comma::io::ostream output( "-", comma::io::mode::binary );
    
    
    if( options.exists( "--help,-h" ) ) { usage(); }
    
    try
    {
        comma::io::publisher publisher( options.value< std::string >( "--publish" ), 
                                        is_binary ? comma::io::mode::binary : comma::io::mode::ascii );
        
        if( options.exists( "--fields,-f" ) ) { output_fields = options.value< std::string >( "--fields,-f" ); }
        is_binary = options.exists( "--binary,-b" );
        
        hok::state_command start( "BM" ); // starts transmission
        hok::state_reply start_reply;
    
        output->write( start.data(), hok::state_command::size  );
        input->read( start_reply.data(), hok::state_reply::size  );
//         std::cerr << name() << "received " << start_reply.data();
//         std::cerr << name() << "starting status of " << start_reply.status() << " request " << start.data() << std::endl;
    
    
        {
            app_exit onexit( *output );
            
            if( start_reply.status() != 0 && start_reply.status() != 2 ) { 
                COMMA_THROW( comma::exception, std::string("Starting laser with BM command failed, status: ") + start_reply.data() ); 
            }
        
            static const int STEPS = 1081;
            hok::request_md me( true );
            me.header.start_step = 0;
            me.header.end_step = 1080;
            
            comma::uint32 start_encoder_step = 0;
            
            output->write( me.data(), hok::request_md::size );
            output->flush();
            
            hok::reply_md state;
            input->read( state.data(), hok::reply_md::size );
            
            if( state.request.message_id != me.message_id ) { COMMA_THROW( comma::exception, "message id mismatch for ME status reply" ); }
            if( state.status.status() != 0 ) 
            { 
                std::ostringstream ss;
                ss << "status reply to ME request is not success: " << state.status.status(); // to change to string
                COMMA_THROW( comma::exception, ss.str() ); 
            }
            
            hok::reply_me_data< STEPS > response; // reply with data
            hok::di_data< STEPS >::rays rays;
            while( !signaled && std::cin.good() )
            {
                // TODO just read the status response first, or timeout on read()
                input->read( response.data(), hok::reply_me_data< STEPS >::size );
                if( response.request.message_id != me.message_id ) { COMMA_THROW( comma::exception, "message id mismatch for ME status reply" ); }
                if( response.status.status() != hok::status::data_success ) 
                { 
                    std::ostringstream ss;
                    ss << "data reply to ME request is not success: " << response.status.status(); // to change to string
                    COMMA_THROW( comma::exception, ss.str() ); 
                }
                
                response.encoded.get_values( rays );
                
//                 std::cerr << "some data here:" << rays.steps[0].distance() << ',' << rays.steps[1].distance() << std::endl;
                data_point point3d;
                for( std::size_t i=0; i<STEPS; ++i )
                {
                    point3d.set( rays.steps[i].distance() / 1000.0, 
                                 rays.steps[i].intensity(), 
                                 hok::ust_10lx::step_to_bearing( i + start_encoder_step ) );
                    output_data( point3d, publisher );
                }
            
            }
        }
    }
    catch( std::exception& ex )
    {
        std::cerr << name() << ex.what() << std::endl; return 1;
    }
    catch( ... )
    {
        std::cerr << name() << "unknown exception" << std::endl; return 1;
    }
    
}