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

#include "ocular.h"
#include <comma/base/exception.h>
#include <comma/application/verbose.h>

namespace snark { namespace ocular { namespace roboteye { 
    
void check_ocular_error(::ocular::ocular_error_t status, const std::string& msg="")
{
    if(status)
    {
        COMMA_THROW(comma::exception, "ocular error: "<<msg<<": "<<::ocular::RobotEye::GetErrorString(status));
    }
}

device::device( std::string ip, bool home ) : ::ocular::RobotEye( ip )
{
    if(GetSerial(serial)) { check_ocular_error(GetLastBlockingError(), "cannot connect to ocular (GetSerial)"); }
    comma::verbose<< "connected to ocular ( serial number: " << serial <<")"<< std::endl;
    if(home)
    {
        check_ocular_error(Home(),"Home()");
    }
}
device::~device()
{
    StopLaser();
    Stop();
}
//***************************************************
scanner::scanner(device& dev,const region_scan& rs) : dev(dev)
{
    check_ocular_error(dev.StartRegionScan(rs.azimuth_rate,rs.azimuth_min,rs.azimuth_max,rs.elevation_min,rs.elevation_max,rs.scan_lines),"StartRegionScan");
}
scanner::~scanner()
{
    dev.Stop();
}
//***************************************************
listener::listener( device& dev, bool highspeed_mode ) : dev(dev), block(0), target_port(4365)
{
    unsigned short freq;
    unsigned short averaging;
    bool intensity;

    if (highspeed_mode)
    {
        freq = 30000;       // High-speed shot rate == 30,000 Hz.
        averaging = 1;      // No averaging available in high-speed mode.
        intensity = false;  // No intensity available in high-speed mode.
    }
    else
    {
        freq = 10000;       // Standard shot rate <= 10,000 Hz.
        averaging = 1;      // Up to 10-shot averaging available in standard mode.
        intensity = true;   // Intensity data available in standard mode.
    }
    check_ocular_error( dev.StartLaser( freq, averaging, intensity, target_port ));
}
listener::~listener()
{
    dev.StopLaser();
}
void listener::LaserDataCallback(std::vector<::ocular::ocular_rbe_obs_t> observations, unsigned int timestamp)
{
    boost::posix_time::ptime t=boost::posix_time::microsec_clock::universal_time();
    std::vector<point_t> points(observations.size());
    for(auto& p : observations)
    {
        points.push_back(point_t(t,block,p));
    }
    on_frame(points);
    block++;
}
    
} } } //namespace snark { namespace ocular { namespace roboteye { 
    
