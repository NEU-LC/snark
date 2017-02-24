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
#include <vector>

#include "../commands.h"
#include "../protocol.h"
#include "../traits.h"
#include <comma/name_value/parser.h>
#include <comma/csv/options.h>
#include <comma/name_value/serialize.h>
#include <comma/application/verbose.h>
#include <comma/csv/stream.h>
#include "../../../../timing/timestamped.h"
#include <sstream>

bool raw=false;
bool timestamp=false;
//throw exception on error if strict
bool strict=false;
//loop over acquire command
bool acquire=false;
unsigned int sleep_seconds=0;

template<typename T>
static void write_output(const T& timestamped)
{
    if(raw)
    {
        if(timestamp) { std::cout.write(reinterpret_cast<const char*>(&timestamped.t),sizeof(boost::posix_time::ptime)); }
        std::cout.write(timestamped.data.data(),timestamped.data.size);
    }
    else
    {
        comma::write_json(timestamped.data, std::cout);
    }
}

void handle_reply(const snark::asd::commands::reply_header& header)
{
    if(header.header() != 100) { comma::verbose<<"reply header: "<<header.header()<<" error: "<<header.error()<<std::endl; }
    if(header.error() != 0)
    {
        if(strict) { COMMA_THROW(comma::exception, "asd reply error: " << header.error() ); }
        else { std::cerr<< comma::verbose.app_name() << ": asd reply error: " << header.error()<<std::endl; }
    }
}

template<typename T>
struct app
{
    typedef typename T::reply reply_t;
    typedef snark::timestamped<typename T::reply> timestamped_reply_t;
    static void usage()
    {
        std::cerr<< "    "<< T::name() << std::endl
                        << "        command: " << T::command() << std::endl
                        << "        fileds: "<<comma::join(comma::csv::names<reply_t>(),',') << std::endl
                        << "        format: "<<comma::csv::format::value<reply_t>() << std::endl;
    }
    static bool process(snark::asd::protocol& protocol, const std::string& cmd)
    {
        if(cmd.find(T::command()) !=0 ) { return false; }
        timestamped_reply_t reply=protocol.send<T>(cmd);
        handle_reply(reply.data.header);
        if(!acquire) { write_output(reply); }   //don't write output in --acquire mode
        else if(comma::verbose)
        {
            std::stringstream ss;
            comma::write_json(reply.data, ss);
            comma::verbose<<ss.str()<<std::endl;
        }
        return true;
    }
};


//for acquaire_data command
static bool process_acquire_data(snark::asd::protocol& protocol, const std::string& cmd)
{
    if(cmd.find(snark::asd::commands::acquire_data::command()) !=0 && cmd[0]!='C') { return false; }
    comma::verbose<<"process_acquire_data"<<std::endl;
    //do it once if !acquire
    do
    {
        //send one command
        snark::asd::protocol::acquire_reply_t reply=protocol.send_acquire_data(cmd);
        handle_reply(reply.data.header.header);
        write_output(reply);
        if(sleep_seconds) { sleep(sleep_seconds);}
    }
    while(acquire);
    return true;
}

void usage(bool detail)
{
    std::cerr<<"    read control command from stdin, send it to asd device and output reply in json or raw format" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " <address> [ <options> ]" << std::endl;
    std::cerr << "    <address>: device address of form tcp:<ip>:<port> e.g. tcp:10.1.1.11:8080 " << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --raw: send raw binary reply to stdout; default when not specified it outputs json format of reply" << std::endl;
    std::cerr << "        --timestamp: prepend output with timestamp of receiving response (only works with --raw)" << std::endl;
    std::cerr << "    --strict: throw exception if asd returns error (reply.error!=0)" << std::endl;
    std::cerr << "    --acquire: loop over acquire command" << std::endl;
    std::cerr << "        command 'A,' will be repeated in an infinite loop; so it should be the last input line" << std::endl;
    std::cerr << "        any other command will be executed once, the output will be sent to std err in json form if in --verbose mode" << std::endl;
    std::cerr << "    --output-size: print size of acquire data to stdout and exit" << std::endl;
    std::cerr << "    --timeout=<seconds>: if it doesn't receive any data from device after timeout seconds it will exit with error" << std::endl;
    std::cerr << "    --sleep=<seconds>: sleep this many seconds between receiving response and sending the next request; used with --acquire only" << std::endl;
    std::cerr << "    --omit-new-line: don't add \\n to command (input)" << std::endl;
    
    std::cerr << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr<< "commands detail: " << std::endl;
        app<snark::asd::commands::version>::usage();
        app<snark::asd::commands::abort>::usage();
        app<snark::asd::commands::optimize>::usage();
        app<snark::asd::commands::restore>::usage();
        app<snark::asd::commands::init>::usage();
        app<snark::asd::commands::save>::usage();
        app<snark::asd::commands::erase>::usage();
        app<snark::asd::commands::instrument_gain_control>::usage();
        std::cerr << "    "<< snark::asd::commands::acquire_data::name() << std::endl;
        std::cerr << "        command: " << snark::asd::commands::acquire_data::command() << std::endl;
        std::cerr << "        fileds: "<<comma::join(comma::csv::names<snark::asd::commands::acquire_data::spectrum_data>(),',') << std::endl;
        std::cerr << "        format: "<<comma::csv::format::value<snark::asd::commands::acquire_data::spectrum_data>() << std::endl;
        std::cerr << std::endl;
    }
    else { std::cerr << "    see --help --verbose for more details" << std::endl<< std::endl; }
    std::cerr << "example" << std::endl;
    std::cerr << "      echo \"V,\" | " << comma::verbose.app_name() << " \"tcp:10.1.1.11:8080\" --verbose" << std::endl;
    std::cerr << std::endl;
    exit(0);
}
int main( int ac, char** av )
{
    comma::command_line_options options( ac, av, usage );
    try
    {
        raw=options.exists("--raw");
        timestamp=options.exists("--timestamp");
        if(options.exists("--output-size"))
        {
            std::size_t size=snark::asd::commands::acquire_data::spectrum_data::size;
            if (timestamp) { size += sizeof(boost::posix_time::ptime);}
            std::cout<< size << std::endl;
            return 0;
        }
        if(timestamp && !raw) { COMMA_THROW(comma::exception, "--timestamp option only works with --raw");}
        comma::verbose<<"asd-control"<<std::endl;
        std::vector<std::string> unnamed=options.unnamed("--verbose,-v,--raw,--timestamp,--strict,--acquire,--omit-new-line", "--timeout,--sleep");
        if(unnamed.size() != 1) { COMMA_THROW(comma::exception, "expected address (one unnamed arg); got " << unnamed.size() ); }
        strict=options.exists("--strict");
        acquire=options.exists("--acquire");
        bool omit_new_line=options.exists("--omit-new-line");
        snark::asd::protocol protocol(unnamed[0], options.value("--timeout",0));
        sleep_seconds=options.value("--sleep",0);
        while(std::cin.good())
        {
            //comma::verbose<<"reading stdin..."<<std::endl;
            std::string cmd;
            std::getline( std::cin, cmd );
            if( cmd.empty() ) { continue; }
            if(!omit_new_line) { cmd += '\n'; }
            comma::verbose<<"sending command: "<<cmd<<std::endl;
            bool processed =  process_acquire_data(protocol,cmd)
                || app<snark::asd::commands::version>::process(protocol,cmd)
                || app<snark::asd::commands::abort>::process(protocol,cmd)
                || app<snark::asd::commands::restore>::process(protocol,cmd)
                || app<snark::asd::commands::optimize>::process(protocol,cmd)
                || app<snark::asd::commands::init>::process(protocol,cmd)
                || app<snark::asd::commands::save>::process(protocol,cmd)
                || app<snark::asd::commands::erase>::process(protocol,cmd)
                || app<snark::asd::commands::instrument_gain_control>::process(protocol,cmd);
            if ( !processed ) { COMMA_THROW(comma::exception, "invalid command " << cmd );  }
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
