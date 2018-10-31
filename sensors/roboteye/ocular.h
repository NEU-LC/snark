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

#pragma once

#include <RobotEye.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace snark { namespace ocular { namespace roboteye { 

struct position_t
{
    double pan;
    double tilt;
    position_t() : pan( 0 ), tilt( 0 ) {}
    position_t( double pan, double tilt ) : pan( pan ), tilt( tilt ) {}
};
    
/// lidar point for ocular
struct point_t
{
    boost::posix_time::ptime t;
    uint32_t block;
    ::ocular::ocular_rbe_obs_t point;
    point_t( ) : block(0) { }
    point_t(boost::posix_time::ptime t,uint32_t block,const ::ocular::ocular_rbe_obs_t& point) : t(t), block(block), point(point) { }
};

struct region_scan
{
    double azimuth_rate;    //Hz
    double azimuth_min;     //degrees
    double azimuth_max;     //degrees
    double elevation_min;   //degrees
    double elevation_max;   //degrees
    unsigned short scan_lines;  //number of lines
};

class device : protected ::ocular::RobotEye
{
public:
    std::string serial;
    
    device(std::string ip,bool home=true);
    virtual ~device();
    
protected:
    friend class scanner;
    friend class listener;
};

class scanner
{
public:
    scanner(device& dev,const region_scan& rs);
    virtual ~scanner();
protected:
    device& dev;
};

class listener : protected ::ocular::RobotEyeLaserDataCallbackClass
{
public:
    listener(device& dev,bool highspeed_mode=false);
    virtual ~listener();
protected:
    device& dev;
    uint32_t block;
    unsigned short target_port;
    void on_frame(const std::vector<point_t>& points) { }
    void LaserDataCallback(std::vector<::ocular::ocular_rbe_obs_t> observations, unsigned int timestamp);
};

} } } //namespace snark { namespace ocular { namespace roboteye { 
