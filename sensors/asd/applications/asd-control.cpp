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

#include <iostream>
#include <cstring>
#include <algorithm>

#include "../commands.h"
#include "../protocol.h"
#include "../traits.h"
#include <comma/name_value/parser.h>
#include <comma/csv/options.h>
#include <comma/application/signal_flag.h>
#include <comma/name_value/serialize.h>
#include <comma/application/cverbose.h>
#include <comma/csv/stream.h>

comma::signal_flag signaled;

bool raw=false;

template<typename T>
struct app
{
    typedef typename T::reply reply_t;
    static void usage()
    {
        std::cerr<< "    "<< T::name() << std::endl
                        << "        command: " << T::command() << std::endl
                        << "        reply fileds: "<<comma::join(comma::csv::names<reply_t>(),',') << std::endl
                        << "        reply format: "<<comma::csv::format::value<reply_t>() << std::endl;
    }
    static bool process(snark::asd::protocol& protocol, const std::string& cmd)
    {
        if(cmd.find(T::command()) ==0 )
        {
            reply_t reply = protocol.send<T>(cmd);
            if(raw)
                std::cout.write(reply.data(),reply.size);
            else
                comma::write_json(reply, std::cout);
            return true;
        }
        return false;
    }
};

void usage(bool detail)
{
    std::cerr<<"    read control command from stdin, send it to asd device and output reply in json or raw format" << std::endl
                    <<std::endl;
    if( !detail ) { std::cerr << "    see --help --verbose for more details" << std::endl<< std::endl; }
    std::cerr<< "usage: " << comma::cverbose.app_name() << " <options>" << std::endl
                    << std::endl
                    << "options" << std::endl
                    << "    --help,-h: show help" << std::endl
                    << "    --verbose,-v: show detailed messages" << std::endl
                    << "    --address=<tcp_address>: device address of form tcp:<ip>:<port> e.g. tcp:10.1.1.11:8080 " << std::endl
                    << "    --raw: send raw binary reply to stdout; default when not specified it outputs json format of reply" << std::endl
                    << "    --trace: show messages being sent and received" << std::endl
                    << std::endl
                    << std::endl;
    if(detail)
    {
        std::cerr<< "commands detail: " << std::endl;
        app<snark::asd::commands::version>::usage();
        app<snark::asd::commands::abort>::usage();
        app<snark::asd::commands::optimize>::usage();
        std::cerr<< std::endl;
        
    }
    std::cerr<< "example" << std::endl
                    << "      echo \"V,\" | " << comma::cverbose.app_name() << " --address=\"tcp:10.1.1.11:8080\" --verbose --trace" << std::endl
                    << std::endl;
    exit(0);
}
int main( int ac, char** av )
{
    comma::command_line_options options( ac, av, usage );
    raw=options.exists("--raw");
    char buf[2000];
    try
    {
        comma::cverbose<<"asd-control"<<std::endl;
        snark::asd::protocol protocol(options.value<std::string>("--address"),options.exists("--trace"));
        while(std::cin.good() && !signaled)
        {
            comma::cverbose<<"reading stdin..."<<std::endl;
            std::cin.getline(buf,sizeof(buf));
            buf[sizeof(buf)-1]='\0';
            std::string cmd(buf);
            cmd+="\n";
            app<snark::asd::commands::version>::process(protocol,cmd)
                || app<snark::asd::commands::abort>::process(protocol,cmd)
                || app<snark::asd::commands::optimize>::process(protocol,cmd);
        }
    }
    catch( std::exception& ex )
    {
        std::cerr << "asd-control" << ex.what() << std::endl; return 1;
    }
    catch( ... )
    {
        std::cerr << "asd-control" << "unknown exception" << std::endl; return 1;
    }
    return 0;
}
