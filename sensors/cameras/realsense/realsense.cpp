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

#include "realsense.h"
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <boost/lexical_cast.hpp>
#include <sstream>

namespace snark { namespace realsense {

struct options
{
    std::map<std::string,rs::option> names;
    options();
};

options options;
    
namespace impl {
rs::format parse_format(const std::string& s)
{
    if(s.empty()) return rs::format::any;
    if(s=="any") return rs::format::any;
    if(s=="z16") return rs::format::z16;
    if(s=="disparity16") return rs::format::disparity16;
    if(s=="xyz32f") return rs::format::xyz32f;
    if(s=="yuyv") return rs::format::yuyv;
    if(s=="rgb8") return rs::format::rgb8;
    if(s=="bgr8") return rs::format::bgr8;
    if(s=="rgba8") return rs::format::rgba8;
    if(s=="bgra8") return rs::format::bgra8;
    if(s=="y8") return rs::format::y8;
    if(s=="y16") return rs::format::y16;
    if(s=="raw10") return rs::format::raw10;
    if(!s.empty()&&isdigit(s[0]))
    {
        int n=boost::lexical_cast<int>(s);
        if(n>=0&&n<=11) return (rs::format)n;
    }
    COMMA_THROW(comma::exception, "can't parse format: "<<s);
}

rs::stream parse_stream(const std::string& s)
{
    if(s=="depth") return rs::stream::depth;
    if(s=="color") return rs::stream::color;
    if(s=="infrared") return rs::stream::infrared;
    if(s=="infrared2") return rs::stream::infrared2;
    if(s=="points") return rs::stream::points;
    if(!s.empty()&&isdigit(s[0]))
    {
        int n=boost::lexical_cast<int>(s);
        if(n>=0&&n<=10) return (rs::stream)n;
    }
    COMMA_THROW( comma::exception, "invalid stream name: "<<s);
}

}   //namespace impl {

format::format():value(rs::format::any) { }
format::format(const std::string& s):value(impl::parse_format(s)) { }
format::format(rs::format f):value(f) { }
format::operator rs::format() const { return value; }
unsigned format::cv_type() const
{
    switch(value)
    {
        case rs::format::z16:
        case rs::format::disparity16:
        case rs::format::y16:
            return CV_16UC1;
        case rs::format::xyz32f:
            return CV_32FC3;
        case rs::format::y8:
            return CV_8UC1;
        case rs::format::rgb8: 
        case rs::format::bgr8:
            return CV_8UC3;
        case rs::format::rgba8:
        case rs::format::bgra8:
        case rs::format::yuyv:
            return CV_8UC4;
        case rs::format::raw10: //5 bytes format, doesn't fit any cv types; but CV_MAKETYPE(CV_8U,5) would have same size
        default:
        { COMMA_THROW( comma::exception, "format not supported: "<<value); }
    }
}
size_t format::size() const
{
    switch(value)
    {
        case rs::format::z16: 
        case rs::format::disparity16:
            return sizeof(std::int16_t);
        case rs::format::xyz32f:
            return 3*sizeof(std::int32_t); //32-bit float
        case rs::format::rgb8: 
        case rs::format::bgr8:
            return 3*sizeof(std::int8_t);
        case rs::format::rgba8:
        case rs::format::bgra8:
            return 4*sizeof(std::int8_t);
        case rs::format::raw10:
            return 5;
        //not sure?
        case rs::format::y8:
            return sizeof(std::int8_t);
        case rs::format::y16:
            return sizeof(std::int16_t);
        //??
        case rs::format::yuyv:
            return 4*sizeof(std::int8_t);
        default:
        { COMMA_THROW(comma::exception, "unknown format size: "<<value); }
    }
}
std::string format::name_list()
{
    return "any,z16,disparity16,xyz32f,yuyv,rgb8,bgr8,rgba8,bgra8,y8,y16,raw10";
}
/*********************************************************/
stream::stream(rs::device& device, const std::string& s) : device(device), value(impl::parse_stream(s)) { }
stream::stream(rs::device& device, rs::stream id) : device(device), value(id) { }
void stream::init(const stream_args& args)
{
    if(args.preset)
    {
        device.enable_stream(value,*args.preset);
    }
    else
    {
        comma::verbose<<"stream::init "<<args.width<<", "<<args.height<<","<<args.format<<","<<args.framerate<<std::endl;
        device.enable_stream(value,args.width,args.height,args.format,args.framerate);
    }
    format=device.get_stream_format(value);
    width=device.get_stream_width(value);
    height=device.get_stream_height(value);
//     comma::verbose<<"stream_writer: stream "<<value<<" width "<<width<<" height "<<height<<" format "<<format<<std::endl;
}
stream::~stream()
{
    device.disable_stream(value);
}
std::pair<boost::posix_time::ptime,cv::Mat> stream::get_frame() const
{
    //boost::posix_time::ptime time=start_time+boost::posix_time::milliseconds(device.get_frame_timestamp(value));
    boost::posix_time::ptime time=boost::posix_time::microsec_clock::universal_time();
    cv::Mat mat(height,width,format.cv_type());
    memcpy(mat.ptr(), device.get_frame_data(value), width*height*format.size());
    return std::pair<boost::posix_time::ptime,cv::Mat>(time,mat);
}
std::string stream::name_list()
{
    return "depth,color,infrared,infrared2";
}
/*********************************************************/
points_cloud::points_cloud(rs::device& device) : device(device)
{
}
points_cloud::~points_cloud()
{
    device.disable_stream(rs::stream::depth);
}
void points_cloud::init(const stream_args& args, rs::stream tex_stream)
{
    if(args.preset)
    {
        device.enable_stream(rs::stream::depth,*args.preset);
    }
    else
    {
        device.enable_stream(rs::stream::depth,args.width,args.height,rs::format::z16,args.framerate);
    }
    rs::format f=device.get_stream_format(rs::stream::depth);
    if(f!=rs::format::z16) { COMMA_THROW( comma::exception,"expected depth format z16 got: "<<f);} 
    depth_intrin=device.get_stream_intrinsics(rs::stream::depth);
    depth_to_tex=device.get_extrinsics(rs::stream::depth, tex_stream);
    tex_intrin=device.get_stream_intrinsics(tex_stream);
    //identical=depth_intrin == tex_intrin && extrin.is_identity();
    depth_scale=device.get_depth_scale();
    
    depth.resize(depth_intrin.width*depth_intrin.height);
}
unsigned points_cloud::scan()
{
    unsigned counter=device.get_frame_timestamp(rs::stream::depth);
    const void* data=device.get_frame_data(rs::stream::depth);
    if(!data) { COMMA_THROW(comma::exception, "depth data is null" ); }
    memcpy(&depth[0],data,depth.size()*sizeof(depth[0]));
    return counter;
}
/*********************************************************/
run_stream::run_stream(rs::device& d):device(d) 
{ 
    device.start(); 
    start_time=boost::posix_time::microsec_clock::universal_time();
}
run_stream::~run_stream() { device.stop(); }
/*********************************************************/
options::options()
{
    if (RS_API_VERSION < 5) { COMMA_THROW(comma::exception,"expected RS_API_VERSION = 5 or greater, got "<<RS_API_VERSION<<" (please install the latest librealsense and/or upgrade snark_librealsense)");}
    names["color_backlight_compensation"]=rs::option::color_backlight_compensation;
    names["color_brightness"]=rs::option::color_brightness;
    names["color_contrast"]=rs::option::color_contrast;
    names["color_exposure"]=rs::option::color_exposure;
    names["color_gain"]=rs::option::color_gain;
    names["color_gamma"]=rs::option::color_gamma;
    names["color_hue"]=rs::option::color_hue;
    names["color_saturation"]=rs::option::color_saturation;
    names["color_sharpness"]=rs::option::color_sharpness;
    names["color_white_balance"]=rs::option::color_white_balance;
    names["color_enable_auto_exposure"]=rs::option::color_enable_auto_exposure;
    names["color_enable_auto_white_balance"]=rs::option::color_enable_auto_white_balance;
    names["f200_laser_power"]=rs::option::f200_laser_power;
    names["f200_accuracy"]=rs::option::f200_accuracy;
    names["f200_motion_range"]=rs::option::f200_motion_range;
    names["f200_filter_option"]=rs::option::f200_filter_option;
    names["f200_confidence_threshold"]=rs::option::f200_confidence_threshold;
    names["sr300_auto_range_enable_motion_versus_range"]=rs::option::sr300_auto_range_enable_motion_versus_range;
    names["sr300_auto_range_enable_laser"]=rs::option::sr300_auto_range_enable_laser;
    names["sr300_auto_range_min_motion_versus_range"]=rs::option::sr300_auto_range_min_motion_versus_range;
    names["sr300_auto_range_max_motion_versus_range"]=rs::option::sr300_auto_range_max_motion_versus_range;
    names["sr300_auto_range_start_motion_versus_range"]=rs::option::sr300_auto_range_start_motion_versus_range;
    names["sr300_auto_range_min_laser"]=rs::option::sr300_auto_range_min_laser;
    names["sr300_auto_range_max_laser"]=rs::option::sr300_auto_range_max_laser;
    names["sr300_auto_range_start_laser"]=rs::option::sr300_auto_range_start_laser;
    names["sr300_auto_range_upper_threshold"]=rs::option::sr300_auto_range_upper_threshold;
    names["sr300_auto_range_lower_threshold"]=rs::option::sr300_auto_range_lower_threshold;
    names["r200_lr_gain"]=rs::option::r200_lr_gain;
    names["r200_lr_exposure"]=rs::option::r200_lr_exposure;
    names["r200_depth_units"]=rs::option::r200_depth_units;
    names["r200_auto_exposure_mean_intensity_set_point"]=rs::option::r200_auto_exposure_mean_intensity_set_point;
    names["r200_auto_exposure_bright_ratio_set_point"]=rs::option::r200_auto_exposure_bright_ratio_set_point;
    names["r200_auto_exposure_kp_gain"]=rs::option::r200_auto_exposure_kp_gain;
    names["r200_auto_exposure_kp_exposure"]=rs::option::r200_auto_exposure_kp_exposure;
    names["r200_auto_exposure_kp_dark_threshold"]=rs::option::r200_auto_exposure_kp_dark_threshold;
    names["r200_auto_exposure_top_edge"]=rs::option::r200_auto_exposure_top_edge;
    names["r200_auto_exposure_bottom_edge"]=rs::option::r200_auto_exposure_bottom_edge;
    names["r200_auto_exposure_left_edge"]=rs::option::r200_auto_exposure_left_edge;
    names["r200_auto_exposure_right_edge"]=rs::option::r200_auto_exposure_right_edge;
    names["r200_depth_control_estimate_median_decrement"]=rs::option::r200_depth_control_estimate_median_decrement;
    names["r200_depth_control_estimate_median_increment"]=rs::option::r200_depth_control_estimate_median_increment;
    names["r200_depth_control_median_threshold"]=rs::option::r200_depth_control_median_threshold;
    names["r200_depth_control_score_minimum_threshold"]=rs::option::r200_depth_control_score_minimum_threshold;
    names["r200_depth_control_score_maximum_threshold"]=rs::option::r200_depth_control_score_maximum_threshold;
    names["r200_depth_control_texture_count_threshold"]=rs::option::r200_depth_control_texture_count_threshold;
    names["r200_depth_control_texture_difference_threshold"]=rs::option::r200_depth_control_texture_difference_threshold;
    names["r200_depth_control_second_peak_threshold"]=rs::option::r200_depth_control_second_peak_threshold;
    names["r200_depth_control_neighbor_threshold"]=rs::option::r200_depth_control_neighbor_threshold;
    names["r200_depth_control_lr_threshold"]=rs::option::r200_depth_control_lr_threshold;
}
const std::map<std::string,rs::option>& option::get_names()
{
    return options.names;
}
option::option() { }
option::option(const std::string& s) { name(s); }
option::option(const rs::option& o) : key(o) { }
void option::name(const std::string& name)
{
    auto it=options.names.find(name);
    if(it==options.names.end()) { COMMA_THROW( comma::exception, "option name not found: "<<name); }
    key=it->second;
}
std::string option::name() const
{
    std::ostringstream out;
    out<<key;
    return boost::algorithm::to_lower_copy(out.str());
}
void option::read(rs::device& device)
{
    value=device.get_option(key);
}
void option::write(rs::device& device)
{
    device.set_option(key,value);
}
option_range::option_range(const rs::option& o) : option(o) { }
void option_range::read(rs::device& device)
{
    option::read(device);
    device.get_option_range(key,min,max,step);
}

} } //namespace snark { namespace realsense {
