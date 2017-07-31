#include "scip2.h"
#include <vector>
#include "../message.h"
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include "../traits.h"
#include <cstdio>

extern bool debug_verbose;
extern bool verbose;

namespace snark { namespace hokuyo {


//static profiles
urg_04lx scip2_device::urg_04lx_;
uhg_08lx scip2_device::uhg_08lx_;
utm_30lx scip2_device::utm_30lx_;

scip2_device::output_t::output_t() : 
            t( boost::posix_time::microsec_clock::universal_time() ), 
            x(0), y(0), z(0), block(0), range(0), bearing(0), elevation(0) 
{
}

scip2_device::output_t::output_t(double distance, double abearing) : 
            t( boost::posix_time::microsec_clock::universal_time() ), 
            x(0), y(0), z(0), block(0), range(0), bearing(abearing), elevation(0)
{
    range = distance / 1000.0;
    x = range * std::cos( bearing );
    y = range * std::sin( bearing );
}

static std::string raw_receive(stream_base& ios)
{
    std::string s;
    for(int i=0;i<20;i++)
    {
        static char buf[500];
        std::size_t len=((serial_stream&)ios).port.read_some( boost::asio::buffer(buf, sizeof(buf))  );
        //std::cerr<<"len: "<<len<<std::endl;
        s+=std::string(buf,len);
        usleep(100000);
        if(len>=2&&buf[len-1]=='\n'&&buf[len-2]=='\n')
            return s;
    }
    return s;
}
// static bool raw_send(stream_base& ios, const char* cmd)
// {
//     if(debug_verbose) { std::cerr<<"->"<<cmd<<std::endl; }
//     ios.write(cmd,strlen(cmd));
//     std::string s=raw_receive(ios);
//     if(debug_verbose) { std::cerr<<"<-"<<s; }
//     int len=s.size();
//     return len>2 && s[len-1]=='\n' && s[len-2]=='\n';
// }
std::string try_receive(stream_base& ios, int timeout_seconds = 1)
{
    comma::io::select select;
    select.read().add( ios.native() );
    select.wait( timeout_seconds ); // wait one select for reply, it can be much smaller
    if(!select.read().ready( ios.native() )) { return std::string(); }
    return raw_receive(ios);
}
bool try_connect(stream_base& ios, int baud_rate)
{
    if(verbose){std::cerr<<"trying to connect on "<<baud_rate<<" bps"<<std::endl;}
    static const char* cmd="SCIP2.0\n";
    ((serial_stream&)ios).set_baud_rate(baud_rate);
    ios.write(cmd, strlen(cmd));
    return (try_receive(ios).find("\n\n") != std::string::npos);
}
boost::shared_ptr<stream_base> scip2_device::connect()
{
    //setup communication
    boost::shared_ptr<stream_base> ios( new serial_stream(port,19200,8,
                                                boost::asio::serial_port_base::parity::none,
                                                boost::asio::serial_port_base::stop_bits::one) );
    //auto baud rate
    bool connected=false;
    if(baud_rate ==0)
    {
        static int supported_baud_rates[]={19200, 57600, 115200, 500000};
        for(int i=sizeof(supported_baud_rates)/sizeof(supported_baud_rates[0])-1;i>=0;i--)
        {
            baud_rate=supported_baud_rates[i];
            connected=try_connect(*ios, baud_rate);
            if(connected) { break; }
        }
    }
    else { connected=try_connect(*ios, baud_rate); }
    if(!connected) { COMMA_THROW(comma::exception, "failed to connect to device on port: " <<port); }
    else if(verbose) { std::cerr<<"connected to "<<port << " on "<< baud_rate<<" bps"<<std::endl; }
    //change baud rate
    if(set_baud_rate && set_baud_rate!=baud_rate)
    {
        char cmd[20];
        std::sprintf(cmd, "SS%06d\n", set_baud_rate);
        ios->write(cmd,strlen(cmd));
        if(try_receive(*ios).find("\n00")!=std::string::npos && try_connect(*ios, set_baud_rate))
        {
            //connected
            if(verbose){std::cerr<<"successfully changed baud rate to: "<<set_baud_rate<<std::endl;}
        }
        else {std::cerr<<"failed to change baud rate to "<<set_baud_rate<<std::endl;}
    }
    return ios;
}
void scip2_device::setup(stream_base& ios)
{
    //swtich to scip2
    //if(raw_send(ios,"V\n"))
    //raw_send(ios,"SCIP2.0\n");
    //raw_send(ios,"VV\n");
}
void scip2_device::request_scan(stream_base& ios, int start_step, int end_step, int num_of_scans)
{
    //these are for URG-04LX, use PP command to get device's parameters
    if(start_step==-1) { start_step=profile->min_step; }
    if(end_step==-1) { end_step=profile->max_step; }
    
    if(start_step>=end_step) { COMMA_THROW( comma::exception, "start step should be less than end step: " << start_step <<","<<end_step ); }
    if(start_step<0) { COMMA_THROW( comma::exception, "start step should be greater than 0 : " << start_step); }
    if(end_step<0) { COMMA_THROW( comma::exception, "end step should be greater than 0 : " << end_step); }
    profile->range_check(start_step,end_step);

    md = boost::shared_ptr<request_md>(new request_md(false));
    md->header.start_step = start_step;
    md->header.end_step = end_step;
    md->num_of_scans = num_of_scans;
    
    ios.write( md->data(), request_md::size );
    ios.flush();
    
    reply_md reply;
    ios.read( reply.data(), reply_md::size );
    
    if( reply.request.message_id != md->message_id ) { COMMA_THROW( comma::exception, "message id mismatch for MD status reply, got: " << md->message_id.str() << " expected: " << reply.request.message_id.str() ); }
    if( reply.status.status() != 0 ) { COMMA_THROW( comma::exception, "status reply to ME request is not success: " << reply.status.status() ); }
}

struct reply_md_header : comma::packed::packed_struct< reply_md_header, sizeof( reply::md_header ) >
{
    typedef reply::md_status status_type;
    reply::md_header header;
};

static unsigned int block=0;

bool scip2_device::receive_response(stream_base& ios)
{
    block++;
    if(debug_verbose) { std::cerr<<"receive_response2"<<std::endl; }
    //read header
    reply_md_header reply;
    int status = read( reply, ios );
    if( status != status::data_success ) 
    {
        COMMA_THROW( comma::exception, "failure dectected when reading data, status: " << status );
    }
    if( reply.header.request.message_id != md->message_id ) { 
        COMMA_THROW( comma::exception, "message id mismatch for MD data reply, got: " << reply.header.request.message_id.str()  << " expected: " << md->message_id.str()); 
    }
    if(debug_verbose) { std::cerr<<"/header"<<std::endl; }

    //now read data
    int start=reply.header.request.header.start_step.unpack(reply.header.request.header.start_step.data());
    int end=reply.header.request.header.end_step.unpack(reply.header.request.header.end_step.data());
    if(debug_verbose) { std::cerr<<"start "<<start<<" end "<<end<<std::endl; }
    scan_data.set_param(start, end-start+1);
    ios.read( scan_data.buf, scan_data.buf_size );
    //read second linefeed
    char lf;
    ios.read(&lf, 1);
    if(lf!='\n')
    {
        if(debug_verbose)
        {
            std::cerr<<"expected \\n, got: "<<int(lf)<<std::endl;
            std::string s=raw_receive(ios);
            std::cerr<<"dumping "<<s.size()<<" chars:"<<serial_stream::dump(s.c_str(),s.size())<<std::endl;
        }
        else { COMMA_THROW(comma::exception, "expected \\n, got: "<<int(lf) ); }
    }
    scan_data.unpack();
    
    int num_of_scans=reply.header.request.num_of_scans.unpack(reply.header.request.num_of_scans.data());
    return num_of_scans!=0;
}

scip2_device::data_t scip2_device::get_data(int scan)
{
    unsigned distance = scan_data.get(scan);
    return data_t(distance, profile->step_to_bearing(scan + scan_data.start_step) );
}

bool scip2_device::is_valid(data_t& data) const
{
    return (data.distance > profile->min_distance);
}

scip2_device::output_t scip2_device::convert(data_t& data)
{
    output_t out(data.distance, data.bearing);
    out.block=block;
    return out;
}

int scip2_device::get_steps() const
{
    return scan_data.count_steps;
}
/******************************************************************************************************/
static void strip_checksum( const char* raw, std::size_t raw_size, char* target )
{
    comma::uint32 size = raw_size;
    const comma::uint32 block = 66; // every block ( 64 bytes ) ending in checksum bit and line feed byte
    const comma::uint32 data_size = 64;
    
    // strip out all sum + line feed
    while( size > block+1 ) // if data is exactly 64 bytes or less, it is different and ends in two line feeds
    {
        memcpy( target, raw, data_size );
        
        if( raw[block-1] != '\n' ) { 
            std::cerr << "strip remaining size is " << size << std::endl;
            COMMA_THROW( comma::exception, "failed to find line feed after 64 data bytes and checksum byte, data block: " << std::string( raw, block )  ); 
        }
        // verify checksum
        
        // todo: more informative expression
        if( !verify_checksum( std::string( raw, block-1 ) ) ) { COMMA_THROW( comma::exception, "checksum of data failed in 64 bytes data block (65th byte is checksum): " << std::string( raw, block-1 ) );  }
        raw += block;
        target += data_size; // advance pointer pass data, checksum and line feed
        
        size -= block;
    }
    
    if( size > 0 )
    {
//         std::cerr << "final block: '" << std::string( raw, size-1 ) << '\'' << std::endl; 
        if( !verify_checksum( std::string( raw, size-1 ) ) ) 
        { 
            if(debug_verbose) { std::cerr<<"checksum of data failed at final odd data block: " << std::string( raw, size-1 )<<std::endl; }
            else { COMMA_THROW( comma::exception, "checksum of data failed at final odd data block: " << std::string( raw, size-1 ) ); }
        }
        // if size is 1, 2 or 3, then it is an error
        memcpy( target, raw, size-2 ); // it ends in triplet <sum, lf>
    }
}

scip2_data::scip2_data() : buf(NULL), buf_size(0), data(NULL), count_steps(0), start_step(0)
{
}
void scip2_data::set_param(int start, int steps)
{
    if(steps<0 ||start<0){COMMA_THROW(comma::exception, "invalid param @scip2_data::set_param " << start <<"," <<steps );}
    start_step=start;
    if(count_steps==steps){return;}
    clear();
    count_steps=steps;
    if(count_steps)
    {
        int data_only_size=count_steps*single_t::size;
        int num_of_sums = ( (data_only_size)/64 ) + ( data_only_size % 64 > 0 ? 1 : 0 );
        buf_size = data_only_size + num_of_sums*(size_of_sum + 1); /// adding one for line feed
        buf=new char[buf_size];
        data=new char[data_only_size];
    }
}
scip2_data::~scip2_data()
{
    clear();
}
void scip2_data::clear()
{
    if(buf!=NULL)
    {
        delete buf;
        buf=NULL;
        buf_size=0;
    }
    if(data!=NULL)
    {
        delete data;
        data=NULL;
        count_steps=0;
    }
}
void scip2_data::unpack()
{
    strip_checksum( buf, buf_size, data);
}
comma::uint32 scip2_data::get(int i)
{
    if(i<0 || i>=count_steps) { COMMA_THROW( comma::exception, "index out of range: "<<i<<"/"<<count_steps ); }
    if(data==NULL) { COMMA_THROW( comma::exception, "data is empty"); }
    return single_t::unpack(data+ i * single_t::size);
}
/************************************************************************************/
double scip2_profile::step_to_bearing(int step)
{
    double angular_resolution=2*M_PI/slit_division;  //one step in radian
    return ( step - sensor_front_step ) * angular_resolution;
}
void scip2_profile::range_check(int start_step, int end_step)
{
    if(end_step>max_step){COMMA_THROW( comma::exception, "end step should be less than max_step("<<max_step<<") : " << end_step);}
}
urg_04lx::urg_04lx()
{
    min_step=44;       //initial measrement step; min valid step: 0
    max_step=725;  //last measurement; max valid step: 768
    slit_division=1024;
    sensor_front_step=384;
    min_distance=0.020;
}
void urg_04lx::range_check(int start_step, int end_step)
{
    static const int MAX_VALID_STEP=768;
    if(end_step>MAX_VALID_STEP) { COMMA_THROW( comma::exception, "end step should be less than MAX_VALID_STEP("<<MAX_VALID_STEP<<") : " << end_step); }
}

uhg_08lx::uhg_08lx()
{
    min_step=0;
    max_step=768;
    slit_division=1024;
    sensor_front_step=384;
}

utm_30lx::utm_30lx()
{
    min_step=0;
    max_step=1080;
    slit_division=1440;
    sensor_front_step=540;
}

} }  // namespace snark { namespace hokuyo {
