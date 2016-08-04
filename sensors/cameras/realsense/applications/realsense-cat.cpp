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
#include <comma/io/stream.h>
#include "../../../../imaging/cv_mat/serialization.h"
#include "../realsense.h"
#include "../../../../visiting/eigen.h"

using namespace snark::realsense;
comma::signal_flag signaled;
std::vector<uchar> margin_color={255,255,255};
bool discard_margin=false;

struct list
{
    struct output_t
    {
        std::string name;
        std::string usb_port;
        std::string serial_number;
    };
    static void process(rs::context& context, const comma::csv::options& csv);
    static std::string field_names();
};

struct device_t
{
    rs::context& context;
    rs::device* device;
    std::string output;
    std::string device_options;
    device_t(rs::context& context);
    device_t(rs::context& context,const std::string& s);
    void parse_option(const std::string& s);
    rs::device* find_device(const std::string& name, const std::string& value, std::function<bool (rs::device*,const std::string& s)> fn);
    static void help(std::ostream& out, const std::string& indent="    ");
};

void usage(bool detail)
{
    std::cerr<<"    Intel realsense camera streaming" << std::endl;
    std::cerr<<"        outputs binary csv of points cloud and mapped color" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --output-format: print output format and exit"<<std::endl;
    std::cerr << "    --output-fields: print output fields and exit"<<std::endl;
    std::cerr << "    --camera-stream,--stream=<stream>: output cv images of the specified camera stream, instead of points cloud csv" << std::endl;
    std::cerr << "        <stream>: "<<stream::name_list()<<std::endl;
    std::cerr << "    --image-format=<format>: use the specified format for cv image, when not specified uses default format"<<std::endl;
    std::cerr << "        <format>: "<<format::name_list()<<std::endl;
    std::cerr << "    --device=<param>: specify device and its parameters, multiple --device=<param> can be specified"<<std::endl;
    std::cerr << "        <param>:"<<std::endl;
    device_t::help(std::cerr,"            ");
    std::cerr << "    --list,--list-devices: output available devices and exit, csv fields: "<<list::field_names()<< std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --options,--camera-options=<options>: set options for all devices, <options> is a comma serparated list of <option_name>=<value>"<<std::endl;
    std::cerr << "    --list-options: list device options and exit"<<std::endl;
    std::cerr << "        output fields: device_index,name,value,min,max,step"<<std::endl;
    std::cerr << "    --get-options=<options>: get device options and exit; <options>: comma separated option names to be retrieved"<<std::endl;
    std::cerr << "        output fields: device_index,name,value"<<std::endl;
    std::cerr << "    --margin-color=<color>: color for points that are outside image camera; <color>: <r>,<g>,<b> default: 255,255,255"<<std::endl;
    std::cerr << "    --discard-margin: don't include marginal points in the output"<<std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
        std::cerr << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "example" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view camera streams:" << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << "  --stream=color --image-format=bgr8 | cv-cat \"resize=2;view;null\" " << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << "  --stream=depth | cv-cat \"brightness=10;resize=2;view;null\" " << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view points cloud mapped with color:" << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << " --binary t,ui,3d,3ub --fields t,block,x,y,z,r,g,b | csv-select --binary t,ui,3d,3ub --fields t,,x,y,z \"z;less=4\" | view-points --binary t,ui,3d,3ub --fields t,block,x,y,z,r,g,b" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view points cloud without color map:"<< std::endl;
    std::cerr << "        " << comma::verbose.app_name() << " --binary t,ui,3d --fields t,block,x,y,z | csv-select --binary t,ui,3d --fields t,b,x,y,z \"z;less=4\" | view-points --binary t,ui,3d --fields t,block,x,y,z"<< std::endl;
    std::cerr << std::endl;
}

struct color_t
{
    std::vector<unsigned char> buf;
    color_t() : buf(3) { }
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
        unsigned counter;
        unsigned block;
        Eigen::Vector3d coordinates;
        // todo: any more point attributes available?
        //cv::Mat?
        color_t color;
    };
    
    rs::device& device;
    bool has_color;
    stream color;
    snark::realsense::points_cloud points_cloud;
    comma::io::ostream out_stream;
    comma::csv::output_stream<output_t> os;
    
    points_t(device_t& device,bool has_color,const comma::csv::options& csv);
    void scan(unsigned block);
    //process points cloud
    static void process(std::vector<device_t>& devices,const comma::csv::options& csv);
    static void output_format();
    static void output_fields();
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
        //v.apply( "", p.buf );
        v.apply( "r", p.buf[0] );
        v.apply( "g", p.buf[1] );
        v.apply( "b", p.buf[2] );
    }
};

template <> struct traits< points_t::output_t >
{
    template< typename K, typename V > static void visit( const K& k, const points_t::output_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "counter", p.counter );
        v.apply( "block", p.block );
        v.apply( "coordinates", p.coordinates );
        v.apply( "color", p.color );
    }
};

} } // namespace comma { namespace visiting {

struct camera_stream
{
    static void process(rs::device& device,const std::string& stream_name,format format)
    {
        stream stream(device,stream_name);
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
        }
        comma::verbose<<"signaled!"<<std::endl;
    }
};

void points_t::process(std::vector<device_t>& devices,const comma::csv::options& csv)
{
    comma::verbose<<"points_t::process"<<std::endl;
    bool has_color=csv.fields.empty() || csv.has_some_of_fields("r,g,b");
    comma::verbose<<"csv.fields "<<csv.fields<<std::endl;
    if(!has_color)
    { comma::verbose<<"not processing color"<<std::endl; }
    //
    std::vector<std::unique_ptr<points_t>> all;
    for(unsigned i=0;i<devices.size();i++)
    {
        all.push_back(std::unique_ptr<points_t>(new points_t(devices[i],has_color,csv)));
    }
    //comma::verbose<<"points_t::process running "<<device.is_streaming()<< " start_time "<< boost::posix_time::to_iso_string(runner.start_time) <<std::endl;
    if(devices.size()==1)
    {
        run_stream runner(*devices[0].device);
        for(unsigned block=0;!signaled;block++)
        {
            all[0]->scan(block);
        }
    }
    else
    {
        unsigned block=0;
        while(!signaled)
        {
            for(unsigned i=0;i<all.size()&&!signaled;i++)
            {
                run_stream runner(*devices[i].device);
                all[i]->scan(block++);
            }
        }
    }
    comma::verbose<<"received a signal"<<std::endl;
}

points_t::points_t(device_t& dev,bool has_color,const comma::csv::options& csv) : 
    device(*dev.device),has_color(has_color),color(device,rs::stream::color),points_cloud(device),
    out_stream(dev.output), //,csv.binary()?comma::io::mode::binary : comma::io::mode::ascii),
    os(*out_stream, csv)
{
    if(has_color)
    {
        color.init(rs::preset::best_quality);
        if(color.format.value!=rs::format::rgb8) { COMMA_THROW( comma::exception, "expected image format "<<rs::format::rgb8<<", got "<<color.format); }
    }
    points_cloud.init(rs::stream::color);
}
void points_t::scan(unsigned block)
{
    if(!device.is_streaming()) { COMMA_THROW(comma::exception, "device not streaming" ); }
    device.wait_for_frames();
    //get points
    output_t out;
    out.t=boost::posix_time::microsec_clock::universal_time();
    out.counter=points_cloud.scan();
    out.block=block;
    auto pair=color.get_frame();
    cv::Mat& mat=pair.second;
    rs::float3 point;
    for(unsigned index=0;index<points_cloud.count();index++)
    {
        bool discard=false;
        if(points_cloud.get(index,point))
        {
            out.coordinates=Eigen::Vector3d(point.x,point.y,point.z);
            if(has_color)
            {
                auto color_pixel=points_cloud.project(point);
                const int cx = (int)std::round(color_pixel.x);
                const int cy = (int)std::round(color_pixel.y);
                if(cx < 0 || cy < 0 || cx >= mat.cols || cy >= mat.rows)
                {
                    if(discard_margin) {discard=true;}
                    out.color.set(&margin_color[0]);
                }
                else
                {
                    out.color.set(mat.ptr(cy,cx));
                }
            }
            if(!discard) {os.write(out);}
        }
    }
}

void points_t::output_format()
{
    std::cout<<comma::csv::format::value<points_t::output_t>() << std::endl;
}
void points_t::output_fields()
{
    std::cout<<comma::join( comma::csv::names<points_t::output_t>(false), ',' ) << std::endl;
}
std::string list::field_names()
{
    return comma::join( comma::csv::names<list::output_t>(false), ',' );
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
struct options_t
{
    static void get(std::vector<device_t> devices,const std::string& list)
    {
        std::vector<std::string> v=comma::split(list,",");
        for(unsigned i=0;i<devices.size();i++)
        {
            for(auto s : v)
            {
                option o(s);
                o.read(*devices[i].device);
                std::cout<<i<<","<<o.name()<<","<<o.value<<std::endl;
            }
        }
    }
    static void list(std::vector<device_t> devices)
    {
        for(unsigned i=0;i<devices.size();i++)
        {
            for(const auto a : option::get_names())
            {
                option_range o(a.second);
                try
                {
                    o.read(*devices[i].device);
                    std::cout<<i<<","<<o.name()<<","<<o.value<<","<<o.min<<","<<o.max<<","<<o.step<<std::endl;
                }
                catch(std::exception& ex)
                {
                    comma::verbose<<o.name()<<"; error: "<<ex.what()<<std::endl;
                }
            }
        }
    }
    static void set(device_t& device,const std::string& list)
    {
        std::vector<device_t> v={device};
        set(v,list);
    }
    static void set(std::vector<device_t>& devices,const std::string& list)
    {
        std::vector<std::string> v=comma::split(list,",");
        for(auto s : v)
        {
            std::vector<std::string> name_value=comma::split(s,"=");
            if(name_value.size()!=2) { COMMA_THROW( comma::exception, "expected name=value, got: "<<s); }
            option o(name_value[0]);
            o.value=boost::lexical_cast<double>(name_value[1]);
            for(auto d : devices)
            {
                o.write(*d.device);
            }
        }
    }
};
void set_margin_color(boost::optional<std::string> s)
{
    if(s)
    {
        std::vector<std::string> v=comma::split(*s,",");
        if(v.size()!=margin_color.size()) {COMMA_THROW(comma::exception,"--margin-color expected "<<margin_color.size()<<" numbers for color, got: "<<v.size());} 
        for(std::size_t i=0;i<v.size();i++)
        {
            unsigned c=boost::lexical_cast<unsigned>(v[i]);
            if(c>255) {COMMA_THROW(comma::exception,"--margin-color item, expected value between 0 and 255, got "<<c);}
            margin_color[i]=uchar(c);
        }
    }
}

void device_t::help(std::ostream& out, const std::string& indent)
{
//     std::cerr << "    --usb-port,--port=<port>: use device on given usb port" << std::endl;
//     std::cerr << "    --serial-number,--serial=<number>: use device with given serial number" << std::endl;
    out<<indent<<"semicolon separated list of options for device"<<std::endl;
    out<<indent<<"    <identifier>=<value>: <identifier> is either 'port' or 'serial' and <value> is usb port id or serial number"<<std::endl;
    out<<indent<<"    output=<stream>: stream name to publish data to, default '-' for stdout"<<std::endl;
    out<<indent<<"    options:<name_value_csv>: options to set for this device only, comma separated list of <option_name>=<value>"<<std::endl;
    out<<indent<<"example: "<<std::endl;
    out<<indent<<"    --device \"port=4-1;output=tcp:1234;options:color_brightness=50,color_contrast=75\" --device \"port=4-2\" --device \"serial=987654;output=tcp:5678\""<<std::endl;
}
device_t::device_t(rs::context& context) : context(context), device(NULL), output("-")
{
    auto count=context.get_device_count();
    if(count!=1) { COMMA_THROW( comma::exception, "multiple camera present, please specify --device"); }
    device=context.get_device(0);
    comma::verbose<<"using: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<", serial: "<<device->get_serial()<<std::endl;
}
device_t::device_t(rs::context& context, const std::string& opt_str) : context(context), device(NULL), output("-")
{
    std::vector<std::string> v=comma::split(opt_str,";");
    for(const auto& s : v)
    {
        parse_option(s);
    }
    if(device==NULL)
    {
        auto count=context.get_device_count();
        if(count!=1) {COMMA_THROW(comma::exception,"multiple camera present, please specify port=<s> or serial=<s> in --device");}
        device=context.get_device(0);
        comma::verbose<<"using: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<", serial: "<<device->get_serial()<<std::endl;
    }
    if(!device_options.empty()) {options_t::set(*this,device_options);}
}
void device_t::parse_option(const std::string& str)
{
    std::vector<std::string> v=comma::split(str,"=");
    if(v.size()==2)
    {
        if(v[0]=="port")
        {
            device=find_device(v[0], v[1], [](rs::device* device, const std::string& s) { return device->get_usb_port_id()==s;} );
            return;
        }
        else if(v[0]=="serial")
        {
            device=find_device(v[0], v[1], [](rs::device* device, const std::string& s) { return device->get_serial()==s;} );
            return;
        }
        else if(v[0]=="output")
        {
            output=v[1];
            return;
        }
        //else fall through
    }
    v=comma::split(str,":");
    if(v.size()!=2 || v[0]!="options") {COMMA_THROW(comma::exception,"invalid device option "<<str);}
    device_options=v[1];
}
rs::device* device_t::find_device(const std::string& name, const std::string& value, std::function<bool (rs::device*,const std::string& s)> fn)
{
    auto count=context.get_device_count();
    for(int i=0;i<count;i++)
    {
        rs::device* device=context.get_device(i);
        if(fn(device,value))
        {
            comma::verbose<<"matched: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<", serial: "<<device->get_serial()<<std::endl;
            return device;
        }
    }
    COMMA_THROW(comma::exception,"no device matched "<<name<<"="<<value);
}

// rs::device* match_device(rs::context& context,std::string s)
// {
//     std::vector<std::string> v=comma::split(s,"=");
//     if(v.size()!=2) {COMMA_THROW(comma::exception,"exepcted <name>=<value>, got "<<s);}
//     int count=context.get_device_count();
//     if(v[0]=="port")
//     {
//         for(int i=0;i<count;i++)
//         {
//             rs::device* device=context.get_device(i);
//             if(v[1]==device->get_usb_port_id())
//             {
//                 comma::verbose<<"matched: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<std::endl;
//                 return device;
//             }
//         }
//         COMMA_THROW(comma::exception,"no device matched port="<<v[1]);
//     }
//     else if(v[0]=="serial")
//     {
//         for(int i=0;i<count;i++)
//         {
//             rs::device* device=context.get_device(i);
//             if(v[1]==device->get_serial())
//             {
//                 comma::verbose<<"matched: "<<device->get_name()<<", serial: "<<device->get_serial()<<std::endl;
//                 return device;
//             }
//         }
//         COMMA_THROW(comma::exception,"no device matched serial="<<v[1]);
//     }
//     else {COMMA_THROW(comma::exception,"expected <name>: port or serial, got "<<v[0]);}
// }

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        comma::csv::options csv(options);
        bool do_list=options.exists("--list-devices,--list");
        //csv.full_xpath=true;
        if(options.exists("--output-format"))
        { 
             points_t::output_format();
            return 0; 
        }
        if(options.exists("--output-fields"))
        {
            points_t::output_fields();
            return 0;       
        }
        //
        rs::context context;
        if(do_list) { list::process(context,csv); return 0; }
        //
        std::string stream=options.value<std::string>("--camera-stream,--stream","");
        format format(options.value<std::string>("--image-format",""));
        set_margin_color(options.optional<std::string>("--margin-color"));
        discard_margin=options.exists("--discard-margin");
        //select devices
        auto count=context.get_device_count();
        comma::verbose<<"device count: "<<count<<std::endl;
        if(count==0) { COMMA_THROW( comma::exception, "no realsense camera found!"); }
        std::vector<std::string> devices=options.values<std::string>("--device");
        std::vector<device_t> selected;
        if(devices.size()==0)
        {
            selected.push_back(device_t(context));
        }
        else
        {
            for(const std::string& s : devices)
            {
                selected.push_back(device_t(context,s));
            }
        }
        if(selected.size()==0) { COMMA_THROW( comma::exception, "no device match found"); }
        //
        std::string camera_options=options.value<std::string>("--options,--camera-options","");
        if(!camera_options.empty())
        {
            options_t::set(selected,camera_options);
        }
        if(options.exists("--list-options"))
        {
            options_t::list(selected);
            return 0;
        }
        std::string get_options=options.value<std::string>("--get-options","");
        if(!get_options.empty())
        {
            options_t::get(selected,get_options);
            return 0;
        }
        if(!stream.empty())
        {
            if(selected.size()!=1) { COMMA_THROW( comma::exception, "multiple camera not implemented yet!"); }
            camera_stream::process(*selected[0].device,stream,format);
        }
        else
        {
            points_t::process(selected,csv);
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
