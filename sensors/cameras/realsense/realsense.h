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

#pragma once
#include <librealsense/rs.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <Eigen/Core>
#include <map>

namespace snark { namespace realsense {

//pixel format helper
struct format
{
    rs::format value;
    format();
    format(const std::string& s);
    format(rs::format f);
    unsigned cv_type() const; //e.g. CV_8UC3
    size_t size() const;  //size in bytes
    operator rs::format() const;
    //csv name of formats
    static std::string name_list();
};

struct stream_args
{
    boost::optional<rs::preset> preset;
    int width;
    int height;
    realsense::format format;
    int framerate;
    stream_args(rs::preset p) : preset(p), width(0), height(0), framerate(0) { }
    stream_args(int w=0,int h=0,realsense::format f=realsense::format(),int fps=0) : width(w), height(h), format(f), framerate(fps) { }
};

//individual camera stream
struct stream
{
    rs::device& device;
    rs::stream value;
    realsense::format format;
    boost::posix_time::ptime start_time;
    stream(rs::device& device, rs::stream id);
    stream(rs::device& device, const std::string& s);
    ~stream();
    void init(const stream_args& args);
    std::pair<boost::posix_time::ptime,cv::Mat> get_frame() const;
    //csv name of streams
    static std::string name_list();
private:
    //size_t size;
    unsigned width;
    unsigned height;
};

struct points_cloud
{
    points_cloud(rs::device& device);
    ~points_cloud();
    void init(const stream_args& args, rs::stream tex_stream=rs::stream::color);
    //return millisecond counter of retrieved frame
    unsigned scan();
    bool get(unsigned index,rs::float3& point) const;
    //map point to texture coordinate
    rs::float2 project(const rs::float3& point) const;
    unsigned count() const;
    unsigned width() const;
    unsigned height() const;
private:
    rs::device& device;
    std::vector<std::uint16_t> depth;
    rs::extrinsics depth_to_tex;
    rs::intrinsics depth_intrin;
    rs::intrinsics tex_intrin;
    float depth_scale;
};

struct run_stream
{
    rs::device& device;
    boost::posix_time::ptime start_time;
    run_stream(rs::device& d);
    ~run_stream();
};

struct option
{
    rs::option key;
    double value;
    option();
    option(const std::string& name);
    option(const rs::option& option);
    void name(const std::string& name);
    std::string name() const;
    //read option from device
    virtual void read(rs::device& device);
    //write option to device
    void write(rs::device& device);
    static const std::map<std::string,rs::option>& get_names();
};

struct option_range : public option
{
    option_range() { }
    option_range(const rs::option& o);
    double min;
    double max;
    double step;
    virtual void read(rs::device& device);
};

inline std::ostream& operator<<(std::ostream& o,format format) { return o << format.value; }

/*********************************************************/
// defined inline for performance
inline bool points_cloud::get(unsigned index,rs::float3& point) const
{
    std::uint16_t d=depth[index];
    if(d==0)
        return false;
    int x=index%depth_intrin.width;
    int y=index/depth_intrin.width;
    rs::float2 depth_pixel={(float)x, (float)y};
    point=depth_intrin.deproject(depth_pixel, d*depth_scale);
    return true;
}
inline rs::float2 points_cloud::project(const rs::float3& point) const
{
    return tex_intrin.project(depth_to_tex.transform(point));
}
inline unsigned points_cloud::count() const { return depth.size(); }
inline unsigned points_cloud::width() const { return depth_intrin.width; }
inline unsigned points_cloud::height() const { return depth_intrin.height; }

} } //namespace snark { namespace realsense {
