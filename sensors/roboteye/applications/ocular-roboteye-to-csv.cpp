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

/// @author Navid Pirmarzdashti

#include "../ocular.h"
#include "../traits.h"
#include <comma/csv/stream.h>

using namespace snark::ocular::roboteye;

void usage(bool detail)
{
    std::cerr<<"    ocular lidar csv stream" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " <address> [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "    <address>: ip address of ocular roboteye device" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "output fields: t,azimuth,y,z,intensity"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --output-format: print output format and exit"<<std::endl;
    std::cerr << "    --output-fields: print output fields and exit"<<std::endl;
    std::cerr << "    --home: "<<std::endl;
    std::cerr << "    --region-scan=<rs>: comma separated region scan parameters <rs>: azimuth_rate,azimuth_min,azimuth_max,elevation_min,elevation_max,scan_lines"<<std::endl;
    std::cerr << "    --highspeed-mode: sample at 30 kHz without intensity measurement, if not specified samples at 10 kHz"<<std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << std::endl;
    std::cerr << comma::verbose.app_name()<<" 169.254.111.102 | head"<< std::endl;
    std::cerr << std::endl;
    std::cerr << comma::verbose.app_name()<<" 169.254.111.102 --binary t,ui,3f,uw | view-points --fields ,block,x,y,z,scalar --binary t,ui,3f,uw"<< std::endl;
    std::cerr << std::endl;
}

template<typename T>
struct app_t
{
    static void output_fields()
    {
        std::cout<<comma::join( comma::csv::names<T>(true), ',' ) << std::endl; 
    }
    static void output_format()    //const std::string& fields
    {
        //std::cout<<comma::csv::format::value<output_t>(fields, true) << std::endl;
        std::cout<<comma::csv::format::value<T>() << std::endl;
    }
};

struct app : public app_t<point_t>
{
    comma::csv::output_stream<point_t> os;
    device dev;
    region_scan scan;
    bool highspeed_mode;
    app(const std::string& address, const comma::command_line_options& options) : 
        os(std::cout,comma::csv::options(options)),
        dev(address,options.exists("--home")),
        scan(comma::csv::ascii<region_scan>().get(options.value<std::string>("--region-scan"))),
        highspeed_mode(options.exists("--highspeed-mode"))
    {
        
    }
    void process()
    {
        scanner scanner(dev,scan);
        comma::verbose<<"region scan started"<<std::endl;
        writer go(*this);
        while(std::cout.good())
            usleep(100000);
    }
    struct writer : public listener
    {
        comma::csv::output_stream<point_t>& os;
        writer(app& app) :
            listener(app.dev,app.highspeed_mode),
            os(app.os)
        {
            
        } 
        void on_frame(const std::vector<point_t>& points)
        {
            for(auto i=points.begin();i!=points.end()&&std::cout.good();i++)
            {
                os.write(*i);
            }
        }
    };
};


int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        comma::csv::options csv(options);
        if(options.exists("--output-format")) { app::output_format(); return 0; }
        if(options.exists("--output-fields")) { app::output_fields(); return 0; }
        
        std::vector<std::string> unnamed=options.unnamed( comma::csv::options::valueless_options()+ ",--verbose,-v,--output-fields,--output-format,--home", "-.*" );
        if(unnamed.size()!=1) { COMMA_THROW( comma::exception, "expected one unnamed arguement, got: "<<unnamed.size()); }
        
        app(unnamed[0],options).process();

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
