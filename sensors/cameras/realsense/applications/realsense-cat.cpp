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
#include "../../../../imaging/cv_mat/serialization.h"
#include "../realsense.h"
#include "../../../../visiting/eigen.h"

using namespace snark::realsense;
comma::signal_flag signaled;
std::vector<uchar> margin_color={255,255,255};
bool discard_margin=false;

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
    std::cerr << "        <format>: "<<format_t::name_list()<<std::endl;
    std::cerr << "    --usb-port,--port=<port>: use device on given usb port" << std::endl;
    std::cerr << "    --serial-number,--serial=<number>: use device with given serial number" << std::endl;
    std::cerr << "    --list-devices,--list: output available devices and exit" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --options,--camera-options=<options>: set camera options, <options> is a comma serparated list of <option_name>=<value>"<<std::endl;
    std::cerr << "    --list-options: list device options as csv and exit, fields are name,value,min,max,step"<<std::endl;
    std::cerr << "    --get-options=<options>: get device options and exit; <options>: comma separated option names to be retrieved"<<std::endl;
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
    std::cerr << "        " << comma::verbose.app_name() << "  --stream=color --image-format=bgr8 -v | cv-cat \"resize=2;view;null\" " << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << "  --stream=depth -v | cv-cat \"brightness=10;resize=2;view;null\" " << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view points cloud mapped with color:" << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << " --binary t,ui,3d,3ub | csv-select --binary t,ui,3d,3ub --fields t,s,x,y,z \"z;less=4\" | view-points --binary t,ui,3d,3ub --fields t,scan,x,y,z,r,g,b" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view points cloud without color map:"<< std::endl;
    std::cerr << "        " << comma::verbose.app_name() << " --binary t,ui,3d --fields t,scan,x,y,z | csv-select --binary t,ui,3d --fields t,s,x,y,z \"z;less=4\" | view-points --binary t,ui,3d --fields t,scan,x,y,z"<< std::endl;
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

template<typename T>
struct app_t
{
    static void output_format();
    static void output_fields();
};

struct points_t : public app_t<points_t>
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

struct list : public app_t<list>
{
    struct output_t
    {
        std::string name;
        std::string usb_port;
        std::string serial_number;
    };
    static void process(rs::context& context, const comma::csv::options& csv);
};

// todo
// - realsense.h
//   - use tbb::parallel_for to project colours

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
        v.apply( "scan", p.scan );
        v.apply( "coordinates", p.coordinates );
        v.apply( "color", p.color );
    }
};

} } // namespace comma { namespace visiting {

void process(rs::device& device,const std::string& stream_name,format_t format)
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

void points_t::process(rs::device& device,const comma::csv::options& csv)
{
    comma::verbose<<"points_t::process"<<std::endl;
    comma::csv::output_stream<output_t> os(std::cout, csv);
    bool has_color=csv.fields.empty() || csv.has_some_of_fields("r,g,b");
    comma::verbose<<"csv.fields "<<csv.fields<<std::endl;
    if(!has_color)
        comma::verbose<<"not processing color"<<std::endl;
    //
    stream color(device,rs::stream::color);
    color.init(rs::preset::best_quality);
    if(color.format.value!=rs::format::rgb8) { COMMA_THROW( comma::exception, "expected image format "<<rs::format::rgb8<<", got "<<color.format); }
    points_cloud points_cloud(device);
    points_cloud.init(rs::stream::color);
    run_stream runner(device);
    color.start_time=runner.start_time;
    comma::verbose<<"points_t::process running "<<device.is_streaming()<< " start_time "<< boost::posix_time::to_iso_string(runner.start_time) <<std::endl;
    for(unsigned scan=0;!signaled;scan++)
    {
        if(!device.is_streaming()) { COMMA_THROW(comma::exception, "device not streaming" ); }
        device.wait_for_frames();
        //get points
        output_t out;
        auto pair=color.get_frame();
        points_cloud.scan();
        out.t=pair.first;
        out.scan=scan;
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
    comma::verbose<<"received a signal"<<std::endl;
}

template<typename T>
void app_t<T>::output_format()
{
    std::cout<<comma::csv::format::value<typename T::output_t>() << std::endl;
}
template<typename T>
void app_t<T>::output_fields()
{
    std::cout<<comma::join( comma::csv::names<typename T::output_t>(false), ',' ) << std::endl;
    //std::cout<<"t,scan,x,y,z,r,g,b"<< std::endl; 
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
    static void get(rs::device& device,const std::string& list)
    {
        std::vector<std::string> v=comma::split(list,",");
        for(auto s : v)
        {
            option o(s);
            o.read(device);
            std::cout<<o.name()<<","<<o.value<<std::endl;
        }
    }
    static void list(rs::device& device)
    {
        for(const auto a : option::get_names())
        {
            option_range o(a.second);
            try
            {
                o.read(device);
                std::cout<<o.name()<<","<<o.value<<","<<o.min<<","<<o.max<<","<<o.step<<std::endl;
            }
            catch(std::exception& ex)
            {
                comma::verbose<<o.name()<<"; error: "<<ex.what()<<std::endl;
            }
        }
    }
    static void set(rs::device& device,const std::string& list)
    {
        std::vector<std::string> v=comma::split(list,",");
        for(auto s : v)
        {
            std::vector<std::string> name_value=comma::split(s,"=");
            if(name_value.size()!=2) { COMMA_THROW( comma::exception, "expected name=value, got: "<<s); }
            option o(name_value[0]);
            o.value=boost::lexical_cast<double>(name_value[1]);
            o.write(device);
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
            if(do_list)
                { list::output_fields(); }
            else
                { points_t::output_fields(); }
            return 0;
        }
        //
        rs::context context;
        if(do_list) { list::process(context,csv); return 0; }
        //
        std::string stream=options.value<std::string>("--camera-stream,--stream","");
        format_t format(options.value<std::string>("--image-format",""));
        std::string port=options.value<std::string>("--usb-port,--port","");
        std::string serial=options.value<std::string>("--serial-number,--serial","");
        set_margin_color(options.optional<std::string>("--margin-color"));
        discard_margin=options.exists("--discard-margin");
        //select device
        int count=context.get_device_count();
        comma::verbose<<"device count: "<<count<<std::endl;
        rs::device* device=NULL;
        std::vector<int> selected;
        for(int i=0;i<count;i++)
        {
            device=context.get_device(i);
            if( (port.empty() || port==device->get_usb_port_id()) &&
                (serial.empty() || serial==device->get_serial() ))
            {
                selected.push_back(i);
                comma::verbose<<"selected index "<<i<<" name: "<<device->get_name()<<", usb port id: "<<device->get_usb_port_id()<<std::endl;
            }
        }
        if(count==0) { COMMA_THROW( comma::exception, "no realsense camera found!"); }
        if(selected.size()==0) { COMMA_THROW( comma::exception, "no camera matched (port "<<port <<" serial "<<serial<<")"); }
        if(selected.size()!=1) { COMMA_THROW( comma::exception, "multiple camera not implemented yet!"); }
        //
        std::string camera_options=options.value<std::string>("--options,--camera-options","");
        if(!camera_options.empty())
        {
            options_t::set(*context.get_device(selected[0]),camera_options);
        }
        if(options.exists("--list-options"))
        {
            options_t::list(*context.get_device(selected[0]));
            return 0;
        }
        /*
        //device doesn't save options
        std::string set_options=options.value<std::string>("--set-options","");
        if(!set_options.empty())
        {
            options_t::set(*context.get_device(selected[0]),set_options);
            return 0;
        }
        */
        std::string get_options=options.value<std::string>("--get-options","");
        if(!get_options.empty())
        {
            options_t::get(*context.get_device(selected[0]),get_options);
            return 0;
        }
        if(!stream.empty())
            process(*context.get_device(selected[0]),stream,format);
        else
        {
            points_t::process(*context.get_device(selected[0]),csv);
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
