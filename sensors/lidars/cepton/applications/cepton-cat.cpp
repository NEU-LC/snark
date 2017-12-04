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

#include "../cepton.h"
#include "../traits.h"
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/ascii.h>
#include <iostream>
#include <comma/csv/stream.h>

void usage(bool detail)
{
    std::cerr<<"    cepton lidar stream" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "output fields: t,x,y,z,intensity"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --output-format: print output format and exit"<<std::endl;
    std::cerr << "    --output-fields: print output fields and exit"<<std::endl;
    std::cerr << "    --list: get list of cepton devices and sensor information"<<std::endl;
    std::cerr << "    --port=[<device-port>]: override sdk default" << std::endl;
    std::cerr << "    --disable-image-clip: disable clipping image field of view"<<std::endl;
    std::cerr << "    --disable-distance-clip: disable clipping distance"<<std::endl;
    std::cerr << "    --frames-per-block=<n>: accumulate <n> frames per block, default 1"<<std::endl;
    std::cerr << "    --system-time: use system time instead of timestamp from lidar"<<std::endl;
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
    std::cerr << comma::verbose.app_name()<<" --binary t,ui,4f | view-points --fields ,block,x,y,z,scalar --binary t,ui,4f"<< std::endl;
    std::cerr << std::endl;
    std::cerr << comma::verbose.app_name()<<" --list | name-value-from-csv $("<< comma::verbose.app_name()<<" --list --output-fields) --line-number --prefix cepton" << std::endl;
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

struct list_app : public app_t<CeptonSensorInformation>
{
    comma::csv::output_stream<CeptonSensorInformation> os;
    snark::cepton::device device;
    list_app( const comma::command_line_options& options ) :
        os( std::cout, comma::csv::options( options )),
        device( options.optional< uint16_t >( "--port" ))
    {
    }
    void process()
    {
        for(int i=0;i<10&&!device.end().index;i++)
            usleep(200000);
        for(auto it=device.begin();it<device.end();it++)
        {
            os.write(*(*it));
        }
    }
};

struct app : public app_t<snark::cepton::point_t>
{
    bool all_good;
    comma::csv::output_stream<snark::cepton::point_t> os;
    snark::cepton::device device;
    unsigned frames_per_block;
    bool system_time;
    app(const comma::command_line_options& options) : 
        all_good(true), os(std::cout,comma::csv::options(options)),
        device( options.optional< uint16_t >( "--port" )
              , options.exists( "--disable-image-clip" )
              , options.exists( "--disable-distance-clip" )),
        frames_per_block(options.value<unsigned>("--frames-per-block",1)),
        system_time(options.exists("--system-time"))
    {
    }
    void process()
    {
        listener go(os,frames_per_block,system_time);
        while(std::cout.good())
            usleep(100000);
    }
    struct listener : public snark::cepton::device::listener
    {
        comma::csv::output_stream<snark::cepton::point_t>& os;
        listener(comma::csv::output_stream<snark::cepton::point_t>& os,unsigned fpb,bool system_time) :
            snark::cepton::device::listener(fpb,system_time), os(os) { } 
        void on_frame(const std::vector<snark::cepton::point_t>& points)
        {
            for(auto i=points.begin();i!=points.end()&&std::cout.good();i++)
            {
                os.write(*i);
            }
        }
    };
};

struct factory_i
{
    virtual ~factory_i() { }
    virtual void output_fields()=0;
    virtual void output_format()=0;
    virtual void run(const comma::command_line_options& options)=0;
};

template<typename T>
struct factory_t : public factory_i
{
    typedef T type;
    void output_fields() { T::output_fields(); }
    void output_format() { T::output_format(); }
    void run(const comma::command_line_options& options)
    {

        T app(options);
        app.process();
    }
};

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        std::unique_ptr<factory_i> factory;
        if(options.exists("--list"))
            factory.reset(new factory_t<list_app>());
        else
            factory.reset(new factory_t<app>());
        comma::csv::options csv(options);
        if(options.exists("--output-format")) { factory->output_format(); return 0; }
        if(options.exists("--output-fields")) { factory->output_fields(); return 0; }
        
        factory->run(options);

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
