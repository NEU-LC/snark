#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/base/exception.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <snark/imaging/cv_mat/serialization.h>
#include "../realsense.h"
#include "../../../../visiting/eigen.h"

using namespace snark::realsense;
comma::signal_flag signaled;

void usage(bool detail)
{
    std::cerr<<"    Intel realsense camera streaming" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --usb-port,--port=<port>: use device on given usb port" << std::endl;
    std::cerr << "    --serial-number,--serial=<number>: use device with given serial number" << std::endl;
    std::cerr << "    --list-devices,--list: output available devices and exit" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << std::endl;
    if( detail ) { /* todo: output csv options */ }
    std::cerr << "example" << std::endl;
    std::cerr << "      " << comma::verbose.app_name() << "  --stream=color --format=bgr8 -v | cv-cat \"resize=2;view;null\" " << std::endl;
    std::cerr << "      " << comma::verbose.app_name() << "  --stream=depth -v | cv-cat \"brightness=10;resize=2;view;null\" " << std::endl;
    std::cerr << std::endl;
}

struct color_t
{
//     unsigned char red;
//     unsigned char green;
//     unsigned char blue;
//     unsigned char alpha; // ???
    std::vector<unsigned char> buf;
    color_t() : buf(4) { }
    void set(const uchar* ptr)
    {
        memcpy(&buf[0],ptr,size());
    }
    size_t size() const { return buf.size()*sizeof(unsigned char); }
};

struct points_t
{
    struct output_t
    {
        boost::posix_time::ptime t;
        unsigned scan;
        Eigen::Vector3d coordinates;
        // todo: any more point attributes available?
        //cv::Mat?
        color_t color;
    };

    //process points cloud
    static void process(rs::device& device,const comma::csv::options& csv);
};

struct list
{
    struct output_t
    {
        std::string name;
        std::string usb_port;
        std::string serial_number;
        output_t():name(40,' '),usb_port(40,' '),serial_number(40,' ') { }
    };
    static void process(rs::context& context, const comma::csv::options& csv);
};

namespace comma { namespace visiting {
    
template <> struct traits< list::output_t >
{
    template< typename K, typename V > static void visit( const K& k, const list::output_t& p, V& v )
    {
        v.apply( "name", p.name );
        v.apply( "usb_port", p.usb_port );
        v.apply( "serial_number", p.serial_number );
    }
};

template <> struct traits< color_t >
{
    template< typename K, typename V > static void visit( const K& k, const color_t& p, V& v )
    {
        v.apply( "", p.buf );
    }
};

template <> struct traits< points_t::output_t >
{
    template< typename K, typename V > static void visit( const K& k, const points_t::output_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "scan", p.scan );
        v.apply( "coordinates", p.coordinates );
        v.apply( "color", p.color );
    }
};

} } // namespace comma { namespace visiting {

void process(rs::device& device,const std::string& stream_name,format_t format)
{
    stream_t stream(device,stream_name,format);
    //stream_writer writer(&device,stream);
    run_stream runner(device);
    stream.start_time=runner.start_time;
    snark::cv_mat::serialization::options opt;
    snark::cv_mat::serialization serializer(opt);
    while(!signaled)
    {
//         if(device.is_streaming())
        device.wait_for_frames();
        //writer.write(std::cout);
        serializer.write(std::cout,stream.get_frame());
//         if(device.poll_for_frames())
//             writer.write(std::cout);
//         else
//             usleep(1);
    }
    comma::verbose<<"signaled!"<<std::endl;
}

void points_t::process(rs::device& device,const comma::csv::options& csv)
{
    stream_t color(device,rs::stream::color);
//     stream_t depth(device,rs::stream::depth);
    points_cloud points(device);
//     stream_t infrared(device,rs::stream::infrared);
//     try { stream_t infrared2(device,rs::stream::infrared2); } catch(...) { /* ignore */}
    run_stream runner(device);
    comma::csv::output_stream<points_t::output_t> os(std::cout, csv);
    for(unsigned scan=0;!signaled;scan++)
    {
//         if(device.is_streaming())
        device.wait_for_frames();
        //get points
        output_t out;
        auto pair=color.get_frame();
        points.scan();
        out.t=pair.first;
        out.scan=scan;
        comma::verbose<<"elemSize " <<pair.second.elemSize() << " elemSize1 "<<pair.second.elemSize1()<<std::endl;
        for(int i=0;i<points.count();i++)
        {
            out.coordinates=points.get(i);
            auto index=points.project(i);
            out.color.set(pair.second.ptr(index.y(),index.x()));
            os.write(out);
        }
    }
}


void list::process(rs::context& context, const comma::csv::options& csv)
{
    int count=context.get_device_count();
    comma::csv::output_stream<list::output_t> os(std::cout, csv);
    for(int i=0;i<count;i++)
    {
        list::output_t out;
        rs::device* device=context.get_device(i);
        out.name=device->get_name();
        out.usb_port=device->get_usb_port_id();
        out.serial_number=device->get_serial();
        os.write(out);
    }
}

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        comma::csv::options csv(options);
        csv.full_xpath=true;
        rs::context context;
        if(options.exists("--list-devices,--list"))
        {
            list::process(context,csv);
            return 0;
        }
        std::string stream=options.value<std::string>("--stream","");
        format_t format(options.value<std::string>("--format",""));
        int count=context.get_device_count();
        comma::verbose<<"device count: "<<count<<std::endl;
        for(int i=0;i<count;i++)
        {
            rs::device* device=context.get_device(i);
            comma::verbose<<"name: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<std::endl;
        }
        if(count==0) { COMMA_THROW( comma::exception, "no realsense camera found!"); }
        if(count!=1) { COMMA_THROW( comma::exception, "more than one camera found!"); }
        if(!stream.empty())
            process(*context.get_device(0),stream,format);
        else
            points_t::process(*context.get_device(0),csv);
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
