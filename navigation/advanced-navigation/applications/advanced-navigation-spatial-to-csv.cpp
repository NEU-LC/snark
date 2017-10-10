// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

/// @author Navid Pirmarzdashti

#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <Eigen/Core>
#include <boost/date_time/posix_time/ptime.hpp>
#include "../../../math/roll_pitch_yaw.h"
#include "../../../visiting/traits.h"
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include "../device.h"
#include "../messages.h"
#include <comma/application/signal_flag.h>
#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../math/spherical_geometry/traits.h"
#include "../traits.h"
#include <comma/io/stream.h>
#include <tbb/concurrent_queue.h>
#include <thread>
#include <chrono>

using namespace snark::navigation::advanced_navigation;

comma::signal_flag signaled;
const unsigned default_baud_rate=115200;
const unsigned default_sleep=10000;

void usage(bool detail)
{
    std::cerr<<"    connect to Advanced Navigation Spatial device and output GPS data" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " <what> [<options>]" << std::endl;
    std::cerr<< "    <port>: serial port" << std::endl;
    std::cerr<< "    <what>: select data packet to output, default: navigation"<< std::endl;
    std::cerr << std::endl;
    std::cerr<< "what: " << std::endl;
    std::cerr<< "    navigation: navigation data from system state packet" << std::endl;
    std::cerr<< "    system-state: full system state packet"<< std::endl;
    std::cerr<< "    raw-sensors" << std::endl;
    std::cerr<< "    all: combines system-state, raw-sensors and standard deviations into one record" << std::endl;
    std::cerr<< "    satellites" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h:       show help" << std::endl;
    std::cerr << "    --verbose,-v:    show detailed messages" << std::endl;
    std::cerr << "    --output-fields: print output fields and exit" << std::endl;
    std::cerr << "    --output-format: print output format and exit" << std::endl;
    
    std::cerr << "    --raw: output raw packets to stdout, " << std::endl;
    /* (?) read raw packets from stdin 
    std::cerr << "    --stdin; read packets from stdin, can't be used with options that need to write to device (e.g. --ntrip)" << std::endl;
    */
    
    std::cerr << "    --device=<filename>; filename for serial port e.g. /dev/usb/ttyUSB0" << std::endl;
    std::cerr << "    --baud-rate=<n>: baud rate for connection, default "<< default_baud_rate << std::endl;
    std::cerr << "    --sleep=<n>: microsecond sleep between reading, default "<< default_sleep << std::endl;
    std::cerr << "    --ntrip=<stream>: read ntrip data from stream and send it to device" << std::endl;
    std::cerr << "        stream can be \"-\" for stdin; or a filename or \"tcp:<host>:<port>\" etc" << std::endl;
    std::cerr << "    --description=<field>; print out one line description text for input values of <field>; csv options apply to input" << std::endl;
    std::cerr << "        <field>: system_status | filter_status" << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    sudo mknod /dev/usb/ttyUSB0 c 188 0" << std::endl;
    std::cerr << "    "<<comma::verbose.app_name()<<" \"/dev/usb/ttyUSB0\" " << std::endl;
    std::cerr << "    "<<comma::verbose.app_name()<<" \"/dev/usb/ttyUSB0\" --raw-sensors" << std::endl;
    std::cerr << "    echo 4096 | "<<comma::verbose.app_name()<<" --description filter_status" << std::endl;
    std::cerr << std::endl;
}

struct output
{
    boost::posix_time::ptime t;
    snark::spherical::coordinates coordinates;
    double height;
    snark::roll_pitch_yaw orientation;
    uint16_t system_status;
    uint16_t filter_status;
};

struct output_all
{
    messages::system_state system_state;
    messages::raw_sensors raw_sensors;
    Eigen::Vector3f velocity_stddev;
    Eigen::Vector3f orientation_stddev;
};

struct status_data
{
    uint16_t status;
};

namespace comma { namespace visiting {

template <>
struct traits< output >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "height", p.height );
        v.apply( "orientation", p.orientation );
        v.apply( "system_status", p.system_status );
        v.apply( "filter_status", p.filter_status );
    }

    template < typename Key, class Visitor > static void visit( const Key&, output& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "height", p.height );
        v.apply( "orientation", p.orientation );
        v.apply( "system_status", p.system_status );
        v.apply( "filter_status", p.filter_status );
    }
};

template <>
struct traits< output_all >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output_all& p, Visitor& v )
    {
        v.apply( "", p.system_state );
        v.apply( "", p.raw_sensors );
        v.apply( "velocity_stddev", p.velocity_stddev );
        v.apply( "orientation_stddev", p.orientation_stddev );
    }

};

template <>
struct traits< status_data >
{
    template < typename Key, class Visitor > static void visit( const Key&, const status_data& p, Visitor& v )
    {
        v.apply( "status", p.status );
    }
    template < typename Key, class Visitor > static void visit( const Key&, status_data& p, Visitor& v )
    {
        v.apply( "status", p.status );
    }
};

} } // namespace comma { namespace visiting {

struct ntrip
{
    typedef std::shared_ptr<std::vector<char>> item_t;
    //create stream from name
    comma::io::istream is;
    //have queue<std::vecotr<char>> (or double buffer) to put data in
    static tbb::concurrent_bounded_queue<item_t> queue;
    //app_t can check the queue and write to gps, instead of sleep
    ntrip(const std::string& name) : is(name,comma::io::mode::binary,comma::io::mode::non_blocking), buf(4096)
    {
        
    }
    std::vector<char> buf;
    bool process()
    {
        if(!is->good())
            return false;
        unsigned size=is.available_on_file_descriptor();

        if(size)
        {
            //or use size=readsome...
            is->read(&buf[0],size);
            size=is->gcount();
            if(size)
            {
                //put it in queue
                queue.push(item_t(new std::vector<char>(buf.begin(),buf.begin()+size)));
            }
        }
        return true;
    }
};

struct ntrip_thread
{
    //make a thread for reading
    bool shutdown;
    std::thread thread;
    std::string filename;
    ntrip_thread(const std::string& filename,const comma::command_line_options& options) :
        shutdown(false),
        thread(&ntrip_thread::run,this),
        filename(filename)
    {
        
    }
    ~ntrip_thread()
    {
        shutdown=true;
        thread.join();
    }
    void run()
    {
        while(!shutdown)
        {
            try
            {
                ntrip nt(filename);
                while(!shutdown&&nt.process())
                {
                    usleep(20000);
                }
            }
            catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
            catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
            //try to connect again in 3 seconds
            for(unsigned i=0;i<100&&!shutdown;i++)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(30));
            }
        }
    }
};

struct app_i
{
    virtual ~app_i() { }
    virtual void run()=0;
    virtual void output_fields()=0;
};

struct app_base : protected device
{
    unsigned us;
public:
    app_base(const std::string& port,const comma::command_line_options& options) : 
        device(port,options.value<unsigned>("--baud-rate",default_baud_rate)),
        us(options.value<unsigned>("--sleep",default_sleep))
    {
        
    }
    void process()
    {
        while( !signaled && std::cout.good() )
        {
            device::process();
            if(!ntrip::queue.empty())
            {
                ntrip::item_t item;
                ntrip::queue.pop(item);
                if(item) { send_ntrip(*item); }
            }
            //process ntrip
            usleep(us);
        }
    }
};

template<typename T>
struct app_t : public app_base
{
    comma::csv::output_stream<T> os;
    app_t(const std::string& port,const comma::command_line_options& options) : 
        app_base(port,options),
        os(std::cout,comma::csv::options(options,"",true))
    {
        
    }
    static void output_fields()
    {
        std::cout<<comma::join( comma::csv::names<T>(true), ',' ) << std::endl; 
    }
    static void output_format()    //const std::string& fields
    {
        //std::cout<<comma::csv::format::value<output_t>(fields, true) << std::endl;
        std::cout<<comma::csv::format::value<T>() << std::endl;
    }
};

struct app_nav : public app_t<output>
{
    app_nav(const std::string& port,const comma::command_line_options& options) : app_t(port,options) { }
    //message handlers
    void handle(const messages::system_state* msg)
    {
        output o;
        o.t=msg->t();
        o.coordinates.latitude=msg->latitude();
        o.coordinates.longitude=msg->longitude();
        o.height=msg->height();
        o.orientation=snark::roll_pitch_yaw(msg->orientation[0](),msg->orientation[1](),msg->orientation[2]());
        o.system_status=msg->system_status();
        o.filter_status=msg->filter_status();
        os.write(o);
    }
};

struct app_raw : public app_base
{
    std::vector<char> obuf;
    app_raw(const std::string& port,const comma::command_line_options& options) : app_base(port,options), obuf(260)
    {
        
    }
    static void output_fields() { std::cout<<std::endl; }
    static void output_format() { std::cout<<std::endl;  }
protected:
    void handle_raw(messages::header* msg_header, const char* msg_data,std::size_t msg_data_length)
    {
        obuf.resize(messages::header::size+msg_data_length);
        std::memcpy(&obuf[0],msg_header->data(),messages::header::size);
        std::memcpy(&obuf[messages::header::size],msg_data,msg_data_length);
        std::cout.write(&obuf[0],obuf.size());
    }
};

/// accumulate several packets into one big output record
struct app_all : public app_t<output_all>
{
    app_all(const std::string& port,const comma::command_line_options& options) : app_t(port,options) { }
    output_all output;
    void handle(const messages::system_state* msg)
    {
        //make copy
        memcpy(output.system_state.data(), msg->data(),messages::system_state::size);
//         output.system_state=msg;

        os.write(output);
    }
    void handle(const messages::raw_sensors* msg)
    {
        std::memcpy(output.raw_sensors.data(),msg->data(),messages::raw_sensors::size);
    }
    void handle(const messages::velocity_standard_deviation* msg)
    {
        output.velocity_stddev=Eigen::Vector3f(msg->stddev[0](),msg->stddev[1](),msg->stddev[2]());
    }
    void handle(const messages::orientation_standard_deviation* msg)
    {
        output.orientation_stddev=Eigen::Vector3f(msg->stddev[0](),msg->stddev[1](),msg->stddev[2]());
    }
};

template<typename T>
struct app_packet : public app_t<T>
{
    app_packet(const std::string& port,const comma::command_line_options& options) : app_t<T>(port,options) { }
    void handle(const T* msg)
    {
        app_t<T>::os.write(*msg);
    }
};

struct factory_i
{
    virtual ~factory_i() { }
    virtual void output_fields()=0;
    virtual void output_format()=0;
    virtual void run(const std::string& input,const comma::command_line_options& options)=0;
};

template<typename T>
struct factory_t : public factory_i
{
    typedef T type;
    void output_fields() { T::output_fields(); }
    void output_format() { T::output_format(); }
    void run(const std::string& input,const comma::command_line_options& options)
    {

        T app(input,options);
        app.process();
    }
};

template<typename T>
struct description
{
    comma::csv::input_stream<T> is;
    description(const comma::command_line_options& options) : is(std::cin,comma::csv::options(options)) { }
    void process()
    {
        while(std::cin.good())
        {
            const T* p=is.read();
            if(!p) { break; }
            describe(p);
        }
    }
    virtual void describe(const T* p)=0;
};

struct system_status_description : public description<status_data>
{
    std::vector<std::string> text=
    {
        {"System Failure"},
        {"Accelerometer Sensor Failure"},
        {"Gyroscope Sensor Failure"},
        {"Magnetometer Sensor Failure"},
        {"Pressure Sensor Failure"},
        {"GNSS Failure"},
        {"Accelerometer Over Range"},
        {"Gyroscope Over Range"},
        {"Magnetometer Over Range"},
        {"Pressure Over Range"},
        {"Minimum Temperature Alarm"},
        {"Maximum Temperature Alarm"},
        {"Low Voltage Alarm"},
        {"High Voltage Alarm"},
        {"GNSS Antenna Short Circuit"},
        {"Data Output Overflow Alarm"}
    };
    system_status_description(const comma::command_line_options& options) : description(options) { }
    void describe(const status_data* p)
    {
        if(!p->status)
        {
            std::cout<<"null"<<std::endl;
            return;
        }
        unsigned bit=1;
        for(unsigned i=0;i<text.size();i++)
        {
            if(p->status & bit)
                std::cout<<i<<": "<<text[i]<<"; ";
            bit<<=1;
        }
        std::cout<<std::endl;
    }
};

struct filter_status_description : public description<status_data>
{
    std::vector<std::string> text=
    {
        {"Orientation Filter Initialised"},
        {"Navigation Filter Initialised"},
        {"Heading Initialised"},
        {"UTC Time Initialised"},
        {""},
        {""},
        {""},
        {"Event 1 Occurred"},
        {"Event 2 Occurred"},
        {"Internal GNSS Enabled"},
        {"Dual Antenna Heading Active"},
        {"Velocity Heading Enabled"},
        {"Atmospheric Altitude Enabled"},
        {"External Position Active"},
        {"External Velocity Active"},
        {"External Heading Active"}
    };
    std::vector<std::string> gnss_fix_text=
    {
        {"No GNSS fix"},
        {"2D GNSS fix"},
        {"3D GNSS fix"},
        {"SBAS GNSS fix"},
        {"Differential GNSS fix"},
        {"Omnistar/Starfire GNSS fix"},
        {"RTK Float GNSS fix"},
        {"RTK Fixed GNSS fix"}
    };
    filter_status_description(const comma::command_line_options& options) : description(options) { }
    void describe(const status_data* p)
    {
        unsigned index=(p->status >> 4) & 7;
        std::cout<<"GNSS fix "<<index<<": "<<gnss_fix_text[index]<<"; ";
        unsigned bit=1;
        for(unsigned i=0;i<text.size();i++)
        {
            if(p->status & bit)
                if(!text[i].empty())
                    std::cout<<i<<": "<<text[i]<<"; ";
            bit<<=1;
        }
        std::cout<<std::endl;
    }
};  

static void bash_completion( int argc, char** argv )
{
    std::cout << "--help --verbose" <<
        " navigation raw-sensors system-state satellites" <<
        " --output-fields --output-format"<< 
        std::endl;
}

tbb::concurrent_bounded_queue<ntrip::item_t> ntrip::queue;

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        
        if(options.exists("--bash-completion")) { bash_completion( argc, argv ); return 0; }
        
        std::vector<std::string> unnamed=options.unnamed( comma::csv::options::valueless_options()+ ",--verbose,-v,--output-fields,--output-format", "-.*" );
        
        auto opt_description=options.optional<std::string>("--description");
        if(opt_description)
        {
            if(*opt_description=="system_status")
            {
                system_status_description(options).process();
            }
            else if(*opt_description=="filter_status")
            {
                filter_status_description(options).process();
            }
            else
            {
                COMMA_THROW( comma::exception, "invalid field for description. expected 'system_status' or 'filter_status', got "<<*opt_description);
            }
            return 0;
        }
        
        std::unique_ptr<factory_i> factory;
        if(options.exists("--raw"))
        {
            factory.reset(new factory_t<app_raw>());
        }
        else
        {
            if(unnamed.size()!=1) { COMMA_THROW( comma::exception, "expected one unnamed arguement, got: "<<unnamed.size()); }
            if(unnamed[0]=="navigation") { factory.reset(new factory_t<app_nav>()); }
            else if(unnamed[0]=="all") { factory.reset(new factory_t<app_all>()); }
            else if(unnamed[0]=="raw-sensors") { factory.reset(new factory_t<app_packet<messages::raw_sensors>>()); }
            else if(unnamed[0]=="system-state") { factory.reset(new factory_t<app_packet<messages::system_state>>()); }
            else if(unnamed[0]=="satellites") { factory.reset(new factory_t<app_packet<messages::satellites>>()); }
            else { COMMA_THROW( comma::exception,"expected <what>: navigation | raw-sensors | system-state | all; got "<<unnamed[1]);}
        }
        
        if(options.exists("--output-fields")) { factory->output_fields(); return 0; }
        if(options.exists("--output-format")) { factory->output_format(); return 0; }

        std::unique_ptr<ntrip_thread> ntt;
        auto opt_ntrip=options.optional<std::string>("--ntrip");
        if(opt_ntrip) { ntt.reset(new ntrip_thread(*opt_ntrip,options)); }

        std::string input=options.value<std::string>("--device");
//         if(!options.exists("--device")) { COMMA_THROW( comma::exception, "Please specify either --device=<serial_port> or --stdin"); }

//         if(unnamed.size()<1) { COMMA_THROW( comma::exception, "expected at least one unnamed option for port name, got "<<unnamed.size()); }
        factory->run(input,options);
        
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
