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

#define _GLIBCXX_USE_CXX11_ABI 0

#include <iostream>
#include <sstream>
#include <unistd.h>
#include <RobotEyeCallbacks.h>
#include <RobotEyeGrabber.h>


std::string app_name="ocular-roboteye-grab";

void usage()
{
    std::cerr<< "usage: "<<app_name<<" <ip> <mode> <azimuth_rate> <azimuth_min> <azimuth_max> <elevation_min> <elevation_max> <scan_lines> " <<std::endl;
    std::cerr<< "    grab ocular roboteye data in region scan mode and write to stdout as raw binary" <<std::endl;
    std::cerr<<std::endl;
    std::cerr<< "args:"<<std::endl;
    std::cerr<< "    <ip>: ip address of ocular roboteye device"<<std::endl;
    std::cerr<< "    <mode>: 1 for highspeed 30000 points per second without intensity, 0 for normal mode"<<std::endl;
    std::cerr<< "    <azimuth_rate>: floating point number (Hz)"<<std::endl;
    std::cerr<< "    <azimuth_min> <azimuth_max> <elevation_min> <elevation_max>: floating point number (degrees)"<<std::endl;
    std::cerr<< "    <scan_lines>: integer number of scan lines"<<std::endl;
    std::cerr<<std::endl;
    std::cerr<< "example"<<std::endl;
    std::cerr<< "    "<<app_name<<" 169.254.111.102 0 10 15 345 0 20  100"<<std::endl;
    std::cerr<<std::endl;
}

template<typename T> void parse(const char* s,T& t)
{
    std::string ss(s);
    std::istringstream iss(ss);
    iss >> t;
}

void check_ocular_error(ocular::ocular_error_t status, const std::string& msg="")
{
    if(status)
    {
        std::ostringstream oss;
        oss<<"ocular error: "<<msg<<": "<<ocular::RobotEye::GetErrorString(status);
        throw std::runtime_error(oss.str());
    }
}

struct region_scan
{
    double azimuth_rate;
    double azimuth_min;
    double azimuth_max;
    double elevation_min;
    double elevation_max;
    unsigned short scan_lines;
    region_scan() : 
        azimuth_rate(0), 
        azimuth_min(0), 
        azimuth_max(0), 
        elevation_min(0), 
        elevation_max(0), 
        scan_lines(0)
    {
        
    }
};

struct scanner
{
    ocular::RobotEyeGrabber& grabber;
    scanner(ocular::RobotEyeGrabber& grabber,const region_scan& rs) : grabber(grabber)
    {
        check_ocular_error(grabber.StartRegionScan(rs.azimuth_rate,rs.azimuth_min,rs.azimuth_max,rs.elevation_min,rs.elevation_max,rs.scan_lines),"StartRegionScan");
    }
    ~scanner()
    {
        grabber.Stop();
    }
};

struct writer : public ocular::RobotEyeLaserDataCallbackClass
{
    ocular::RobotEyeGrabber& grabber;
    writer(ocular::RobotEyeGrabber& grabber,bool highspeed_mode) : grabber(grabber)
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
        check_ocular_error(grabber.StartLaser(freq,averaging,intensity,this));
    }
    ~writer()
    {
        grabber.StopLaser();
    }
    void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations, unsigned int timestamp)
    {
        static int debug=0;
        if(debug==0)
            std::cerr<<"sizeof(ocular::ocular_rbe_obs_t): "<<sizeof(ocular::ocular_rbe_obs_t)<<std::endl;
        if(debug++<10)
            std::cerr<<"timestamp: "<<timestamp<<std::endl;
        std::cout.write(reinterpret_cast<const char*>(observations.data()),observations.size()*sizeof(ocular::ocular_rbe_obs_t));
    }
};

int main( int argc, char** argv )
{
    if(argc==1||(argc==2&&argv[1]=="-h")) { usage(); return 1; }
    try
    {
        if(argc!=9) { throw std::runtime_error("expected 8 arguements"); }
        bool highspeed;
        
        std::string ip=argv[1];
        
        parse(argv[2],highspeed);
        
        region_scan rs;
        parse(argv[3],rs.azimuth_rate);
        parse(argv[4],rs.azimuth_min);
        parse(argv[5],rs.azimuth_max);
        parse(argv[6],rs.elevation_min);
        parse(argv[7],rs.elevation_max);
        parse(argv[8],rs.scan_lines);

        ocular::RobotEyeGrabber grabber(ip);
        std::string serial;
        check_ocular_error(grabber.GetSerial(serial),"cannot connect to ocular (GetSerial)");
        std::cerr<< "connected to ocular, serial number: " << serial <<std::endl;
        check_ocular_error(grabber.Home(),"Home()");

        ::scanner scanner(grabber,rs);
        ::writer writer(grabber,highspeed);
        
        while(std::cout.good())
            usleep(10000);
        
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << app_name<<": exception: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << app_name<<": unknown exception" << std::endl;
    }
    return 1;
}
