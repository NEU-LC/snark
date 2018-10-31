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

#include <cmath>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <RobotEye.h>

std::string app_name="ocular-roboteye-grab";

void usage()
{
    std::cerr<< "usage: "<<app_name<<" <ip> <mode> <azimuth_rate> <azimuth_min> <azimuth_max> <elevation_min> <elevation_max> <scan_lines> " <<std::endl;
    std::cerr<< "    grab ocular roboteye data in region scan mode and write to stdout as raw binary" <<std::endl;
    std::cerr<<std::endl;
    std::cerr<< "output is binary size 32"<<std::endl;
    std::cerr<< "    fields: range,bearing,elevation,timestamp,intensity,frame"<<std::endl;
    std::cerr<< "    format: 3d,ui,2uw"<<std::endl;
    std::cerr<<std::endl;
    std::cerr<< "args:"<<std::endl;
    std::cerr<< "    <ip>: ip address of ocular roboteye device"<<std::endl;
    std::cerr<< "    <mode>: 1 for highspeed 30000 points per second without intensity, 0 for normal mode"<<std::endl;
    std::cerr<< "    <azimuth_rate>: floating point number (Hz), max 15Hz"<<std::endl;
    std::cerr<< "    <azimuth_min> <azimuth_max>: floating point number (degrees), range 0 to 360"<<std::endl;
    std::cerr<< "    <elevation_min> <elevation_max>: floating point number (degrees), range -35 to 35"<<std::endl;
    std::cerr<< "    <scan_lines>: integer number of scan lines"<<std::endl;
    std::cerr<<std::endl;
    std::cerr<< "example"<<std::endl;
    std::cerr<< "    "<<app_name<<" 169.254.111.102 0 10 15 345 0 20  100"<<std::endl;
    std::cerr<< "    "<<app_name<<" 172.21.64.155 0 10 15 345 0 20  100 | points-to-cartesian --binary 3d,ui,2uw --fields r,b,e | view-points --binary 3d,ui,2uw --fields x,y,z,,scalar --color 0:15,jet"<<std::endl;
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
    ocular::RobotEye& grabber;
    scanner(ocular::RobotEye& grabber,const region_scan& rs) : grabber(grabber)
    {
        check_ocular_error(grabber.StartRegionScan(rs.azimuth_rate,rs.azimuth_min,rs.azimuth_max,rs.elevation_min,rs.elevation_max,rs.scan_lines),"StartRegionScan");
        std::cerr<<"started region scan"<<std::endl;
    }
    ~scanner()
    {
        ocular::ocular_error_t err=grabber.Stop();
        std::cerr<<"stop ("<<err<<")"<<std::endl;
    }
};

inline double deg2rad(double d) { return d*(M_PI/double(180)); }

struct output
{
    double range;
    double bearing;
    double elevation;
    unsigned int timestamp;
    double amplitude;
    double reflectance;
    double pulse_shape_deviation;
    unsigned short frame;
    output() { }
    output( unsigned int timestamp, const ocular::ocular_rbe_obs_t& p, unsigned short frame=0 )
        : range( p.range )
        , bearing( deg2rad( p.azimuth ))
        , elevation( deg2rad( p.elevation ))
        , timestamp( timestamp )
        , amplitude( p.amplitude )
        , reflectance( p.reflectance )
        , pulse_shape_deviation( p.pulseShapeDeviation )
        , frame( frame )
    {
    }
};

struct writer : public ocular::RobotEyeLaserDataCallbackClass
{
    ocular::RobotEye& grabber;
    unsigned short frame;
    unsigned int target_port;

    writer(ocular::RobotEye& grabber,bool highspeed_mode) : grabber(grabber), frame(0), target_port(4365)
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
        check_ocular_error( grabber.StartLaser( freq, averaging, intensity, target_port ));
        std::cerr<<"started laser "<<freq<<" "<<averaging<<" "<<intensity<<std::endl;
    }
    ~writer()
    {
        ocular::ocular_error_t err=grabber.StopLaser();
        std::cerr<<"stopped laser  ("<<err<<")"<<std::endl;
    }
    void LaserDataCallback(std::vector<ocular::ocular_rbe_obs_t> observations, unsigned int timestamp)
    {
//         static int debug=0;
//         if(debug==0)
//         {
//             std::cerr<<"sizeof(ocular::ocular_rbe_obs_t): "<<sizeof(ocular::ocular_rbe_obs_t)<<std::endl;
//             std::cerr<<"sizeof(output): "<<sizeof(output)<<std::endl;
//         }
//         if(debug++<10)
//             std::cerr<<"timestamp: "<<timestamp<<" size: "<<observations.size()<<std::endl;
        std::vector<output> outputs(observations.size());
        for(unsigned i=0;i<observations.size();i++)
        {
            outputs[i]=output(timestamp,observations[i],frame);
        }
        std::cout.write(reinterpret_cast<const char*>(outputs.data()),outputs.size()*sizeof(output));
        frame++;
    }
};

/// soft signal handler for linux
struct signaled
{
    static bool is_set;
    signaled()
    {
        register_handle(SIGINT);
        register_handle(SIGTERM);
        register_handle(SIGPIPE);
        register_handle(SIGHUP);
    }
    void register_handle(int signal)
    {
        struct sigaction sa;
        sa.sa_handler = handle;
        sigemptyset( &sa.sa_mask );
        sa.sa_flags = 0;
        if( ::sigaction( signal, &sa, NULL ) != 0 ) { throw std::runtime_error("failed to set handler for signal"); }
    }
    static void handle( int sig)
    {
        is_set=true;
    }
    operator bool() const { return is_set; }
};
bool signaled::is_set=false;

int main( int argc, char** argv )
{
    if( argc == 1 || ( argc == 2 && strncmp( argv[1], "-h", 2 ) == 0)) { usage(); return 1; }
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
        
        signaled sig;

        ocular::RobotEye grabber(ip);
        std::string serial;
        check_ocular_error(grabber.GetSerial(serial),"cannot connect to ocular (GetSerial)");
        std::cerr<< "connected to ocular, serial number: " << serial <<std::endl;
        check_ocular_error(grabber.Home(),"Home()");


        ::scanner scanner(grabber,rs);
        ::writer writer(grabber,highspeed);
        
        while(!sig && std::cout.good())
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
