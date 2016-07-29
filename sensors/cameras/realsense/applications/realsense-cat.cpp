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
    static void output_format();
    static void output_fields();
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
    camera_stream_t stream(device,stream_name);
    stream.init(format);
    run_stream runner(device);
    stream.start_time=runner.start_time;
    snark::cv_mat::serialization::options opt;
    snark::cv_mat::serialization serializer(opt);
    while(!signaled)
    {
        if(!device.is_streaming()) { COMMA_THROW(comma::exception, "device not streaming" ); }
        device.wait_for_frames();
        serializer.write(std::cout,stream.get_frame());
//         if(device.poll_for_frames())
//             writer.write(std::cout);
//         else
//             usleep(1);
    }
    comma::verbose<<"signaled!"<<std::endl;
}
int zero_count=0;
void points_t::process(rs::device& device,const comma::csv::options& csv)
{
    comma::verbose<<"points_t::process"<<std::endl;
    camera_stream_t color(device,rs::stream::color);
    camera_stream_t ir1(device,rs::stream::infrared);
    camera_stream_t ir2(device,rs::stream::infrared2);
    color.init(rs::format::rgba8);
    ir1.init(rs::preset::best_quality);
    ir2.init(rs::preset::best_quality);
    points_cloud points_cloud(device);
    points_cloud.init(rs::stream::color);
    run_stream runner(device);
    color.start_time=runner.start_time;
    comma::verbose<<"points_t::process running "<<device.is_streaming()<< " start_time "<< runner.start_time <<std::endl;
    comma::csv::output_stream<output_t> os(std::cout, true, true);
    for(unsigned scan=0;!signaled;scan++)
    {
        if(!device.is_streaming()) { COMMA_THROW(comma::exception, "device not streaming" ); }
        device.wait_for_frames();
        //get points
        output_t out;
        auto pair=color.get_frame();
        auto& points=points_cloud.scan();
        out.t=pair.first;
        out.scan=scan;
        cv::Mat& mat=pair.second;
        //comma::verbose<<"elemSize " <<mat.elemSize() << " elemSize1 "<<mat.elemSize1()<<std::endl;
        //mat.elemSize():4
        //mat.elemSize1():1
        comma::verbose<<"color: rows " <<mat.rows<<" cols "<<mat.cols<<std::endl;
        for(unsigned index=0;index<points.size();index++)
        {
            if(points[index].z)
            {
                out.coordinates=points_cloud.get(index);
                auto coord=points_cloud.deproject(index);
                out.color.set(mat.ptr(coord.y,coord.x));
                os.write(out);
            }
            else
                zero_count++;
        }
        comma::verbose<<"scanned: "<<scan<<" zero_count "<< zero_count<<std::endl;
    }
    comma::verbose<<"signaled!"<<std::endl;
}

void points_t::output_format()
{
    std::cout<<comma::csv::format::value<points_t::output_t>() << std::endl;
}
void points_t::output_fields()
{
    //std::cout<<comma::join( comma::csv::names<points_t::output_t>(true), ',' ) << std::endl;
    std::cout<<"t,scan,x,y,z,b,g,r,a"<< std::endl; 
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
        //
        if(options.exists("--list-devices,--list")) { list::process(context,csv); return 0; }
        if(options.exists("--output-format")) { points_t::output_format(); return 0; }
        if(options.exists("--output-fields")) { points_t::output_fields(); return 0; }
        //
        std::string stream=options.value<std::string>("--stream","");
        format_t format(options.value<std::string>("--format",""));
        //select device
        int count=context.get_device_count();
        comma::verbose<<"device count: "<<count<<std::endl;
        for(int i=0;i<count;i++)
        {
            rs::device* device=context.get_device(i);
            comma::verbose<<"name: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<std::endl;
        }
        if(count==0) { COMMA_THROW( comma::exception, "no realsense camera found!"); }
        if(count!=1) { COMMA_THROW( comma::exception, "more than one camera found!"); }
        //
        if(!stream.empty())
            process(*context.get_device(0),stream,format);
        else
        {
            points_t::process(*context.get_device(0),csv);
        }
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
