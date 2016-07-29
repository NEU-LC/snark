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
#include <Eigen/Core>

namespace snark { namespace realsense {

struct camera
{
    rs::device& device;
    camera(rs::device& device);
};

//pixel format helper
struct format_t
{
    rs::format value;
    format_t();
    format_t(const std::string& s);
    format_t(rs::format f);
    unsigned cv_type() const; //e.g. CV_8UC3
    size_t size() const;  //size in bytes
    operator rs::format();
};

//individual camera stream
struct camera_stream_t
{
    rs::device& device;
    rs::stream value;
    format_t format;
    boost::posix_time::ptime start_time;
    camera_stream_t(rs::device& device, rs::stream id);
    camera_stream_t(rs::device& device, const std::string& s);
    ~camera_stream_t();
    void init(format_t format=format_t());
    void init(rs::preset preset);
    std::pair<boost::posix_time::ptime,cv::Mat> get_frame();
private:
    //size_t size;
    unsigned width;
    unsigned height;
    void init_();
};

struct points_cloud
{
    points_cloud(rs::device& device);
    ~points_cloud();
    void init(rs::stream tex_stream);
    Eigen::Vector3d get(unsigned index)
    {
        return Eigen::Vector3d(points[index].x,points[index].y,points[index].z);
    }
    //map point at index to texture coordinate
    rs::float2 deproject(unsigned index)
    {
        int x=index%depth_intrin.width;
        int y=index/depth_intrin.width;
        return identical ? tex_intrin.pixel_to_texcoord({static_cast<float>(x),static_cast<float>(y)}) : tex_intrin.project_to_texcoord(extrin.transform(points[index])); 
    }
    const std::vector<rs::float3>& scan();
    unsigned count() { return points.size(); }
    unsigned width() { return depth_intrin.width; }
    unsigned height() { return depth_intrin.height; }
private:
    rs::device& device;
    /*
    std::vector<std::int16_t> depth;
    rs::format format;
    unsigned height_;
    unsigned width_;
    */
    std::vector<rs::float3> points;
    rs::extrinsics extrin;
    rs::intrinsics depth_intrin;
    rs::intrinsics tex_intrin;
    bool identical;
};

struct run_stream
{
    rs::device& device;
    boost::posix_time::ptime start_time;
    run_stream(rs::device& d);
    ~run_stream();
};

inline std::ostream& operator<<(std::ostream& o,format_t format) { return o << format.value; }

} } //namespace snark { namespace realsense {
