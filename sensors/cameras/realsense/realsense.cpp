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

namespace snark { namespace realsense {

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

format_t::format_t():value(rs::format::any) { }
format_t::format_t(const std::string& s):value(impl::parse_format(s)) { }
format_t::format_t(rs::format f):value(f) { }
format_t::operator rs::format() { return value; }
unsigned format_t::cv_type() const
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
size_t format_t::size() const
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

/*********************************************************/
camera_stream_t::camera_stream_t(rs::device& device, const std::string& s) : device(device), value(impl::parse_stream(s)) { }
camera_stream_t::camera_stream_t(rs::device& device, rs::stream id) : device(device), value(id) { }
void camera_stream_t::init(format_t f)
{
    device.enable_stream(value,0,0,f,0);
    init_();
}
void camera_stream_t::init(rs::preset preset)
{
    device.enable_stream(value,preset);
    init_();
}
camera_stream_t::~camera_stream_t()
{
    device.disable_stream(value);
}
void camera_stream_t::init_()
{
    format=device.get_stream_format(value);
    width=device.get_stream_width(value);
    height=device.get_stream_height(value);
    //size=width*height*format.size();
    comma::verbose<<"stream_writer: stream "<<value<<" width "<<width<<" height "<<height<<" format "<<format<<std::endl;
}
std::pair<boost::posix_time::ptime,cv::Mat> camera_stream_t::get_frame()
{
    boost::posix_time::ptime time=start_time+boost::posix_time::milliseconds(device.get_frame_timestamp(value));
    cv::Mat mat(height,width,format.cv_type());
    memcpy(mat.ptr(), device.get_frame_data(value), width*height*format.size());
    return std::pair<boost::posix_time::ptime,cv::Mat>(time,mat);
}
/*********************************************************/
points_cloud::points_cloud(rs::device& device) : device(device)
{
    //device.enable_stream(rs::stream::depth,0,0,rs::format::any,0);
    /*
    format=device.get_stream_format(rs::stream::depth);
    width_=device.get_stream_width(rs::stream::depth);
    height_=device.get_stream_height(rs::stream::depth);
    //depth.resize(width_*height_);
    comma::verbose<<"points_cloud: depth stream: width "<<width_<<" height "<<height_<<" format "<<format<<std::endl;
    //if(format!=rs::format::xyz32f) { COMMA_THROW( comma::exception, "expected points_cloud (depth) format rs::format::xyz32f, got: "<<format);}
    if(format!=rs::format::z16) { COMMA_THROW( comma::exception, "expected points_cloud (depth) format rs::format::z16, got: "<<format);}
    */
}

points_cloud::~points_cloud()
{
    device.disable_stream(rs::stream::depth);
}
void points_cloud::init(rs::stream tex_stream)
{
    device.enable_stream(rs::stream::depth,rs::preset::best_quality);
    extrin=device.get_extrinsics(rs::stream::depth, tex_stream);
    depth_intrin=device.get_stream_intrinsics(rs::stream::depth);
    tex_intrin=device.get_stream_intrinsics(tex_stream);
    identical=depth_intrin == tex_intrin && extrin.is_identity();
    
    points.resize(depth_intrin.width*depth_intrin.height);

    /*
    const float depth_scale=device.get_depth_scale();
    const rs::extrinsics extrin = device.get_extrinsics(rs::stream::depth, tex_stream);
    const rs::intrinsics depth_intrin = device.get_stream_intrinsics(rs::stream::depth);
    const rs::intrinsics tex_intrin = device.get_stream_intrinsics(tex_stream);
    bool identical = depth_intrin == tex_intrin && extrin.is_identity();

    auto points = reinterpret_cast<const rs::float3 *>(device.get_frame_data(rs::stream::points));
    auto depth = reinterpret_cast<const uint16_t *>(device.get_frame_data(rs::stream::depth));
    
    for(int y=0; y<depth_intrin.height; ++y)
    {
        for(int x=0; x<depth_intrin.width; ++x)
        {
            if(points->z) //if(uint16_t d = *depth++)
            {
                //const rs::float3 point = depth_intrin.deproject({static_cast<float>(x),static_cast<float>(y)}, d*depth_scale);
                rs::float2 tex_coord=identical ? tex_intrin.pixel_to_texcoord({static_cast<float>(x),static_cast<float>(y)}) : tex_intrin.project_to_texcoord(extrin.transform(*points));
                rs::float3 point_xyz=*points;
            }
            ++points;
        }
    }
    */
}
const std::vector<rs::float3>& points_cloud::scan()
{
    //if(sizeof(points[0])!=4) { COMMA_THROW( comma::exception, "expected float to be 32-bit, got"<<sizeof(points[0])); }
    //points=static_cast<const float*>(device.get_frame_data(rs::stream::points));
    
    //memcpy(&depth[0], device.get_frame_data(rs::stream::points), depth.size()*sizeof(depth[0]));
    comma::verbose<<"points_cloud::scan: width "<<width()<<" height "<<height()<<" points.size() "<<points.size()<<std::endl;
    const void* data=device.get_frame_data(rs::stream::points);
    if(!data) { COMMA_THROW(comma::exception, "points data is null" ); }
    memcpy(&points[0],data,points.size()*sizeof(rs::float3));
    return points;
}
/*
Eigen::Vector3d points_cloud::get(unsigned index)
{
    //return Eigen::Vector3d(index%width_,index/width_,depth[index]);
}

Eigen::Vector2i points_cloud::project(unsigned index)
{
    //return Eigen::Vector2i(index%width_,index/width_);
}
*/
/*********************************************************/
run_stream::run_stream(rs::device& d):device(d) 
{ 
    device.start(); 
    start_time=boost::posix_time::microsec_clock::universal_time();
}
run_stream::~run_stream() { device.stop(); }

} } //namespace snark { namespace realsense {
