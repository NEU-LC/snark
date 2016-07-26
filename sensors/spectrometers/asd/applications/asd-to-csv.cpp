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

#include "../commands.h"
#include "../protocol.h"
#include "../traits.h"
#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/application/verbose.h>
#include "../../../../timing/timestamped.h"

typedef snark::asd::commands::acquire_data::spectrum_data output_t;
typedef snark::timestamped<output_t> timestamped_output_t;

namespace comma { namespace visiting {

//alternatively: use snark timing traits.h: {t, data/output_t}
template <> struct traits< timestamped_output_t >
{
    template< typename K, typename V > static void visit( const K& k, const timestamped_output_t& p, V& v )
    {
        v.apply( "t", p.t );
        traits<output_t>::visit(k, p.data, v);
    }
};

} } // namespace comma { namespace visiting {
    

void usage(bool detail)
{
    std::cerr << "    read raw asd data aqcusition from stdin and output formated records to stdout " << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage:  " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr << "    input: raw asd data reply for acquire command (use --timestamp if input is timestamped" << std::endl;
    std::cerr << "    output: csv output stream of same data; big endian fields are converted; use --binary to output binary csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --output-fields: print field names to stdout" << std::endl;
    std::cerr << "    --output-format: print format to stdout, use --fields=<names> to get format of specific fields" << std::endl;;
    std::cerr << "    --timestamp: data includes timestamp (both input and output)" << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr<< "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
        std::cerr<< std::endl;
    }
    else { std::cerr << "    see --help --verbose for more details" << std::endl<< std::endl; }
    std::cerr << "example" << std::endl;
    std::cerr << "    echo \"A,1,10\" | asd-control --raw --timestamp | " << comma::verbose.app_name() << " --timestamp >a.csv " << std::endl;
    std::cerr << std::endl;
    exit(0);
}

template<typename T>
struct app
{
    static void run(const comma::command_line_options& options)
    {
        comma::csv::options csv(options);
        if(options.exists("--output-fields")) { std::cout<<comma::join(comma::csv::names<T>(), ',')<<std::endl; return;}
        if(options.exists("--output-format")) { std::cout<< comma::csv::format::value<T>(csv.fields,true)<<std::endl; return;}
        process(csv);
    }
    static const T* read();
    static void process(const comma::csv::options& csv);
};

template<typename T>
T* read_packet(std::istream& is, T& t)
{
    is.read(t.data(),t.size);
    std::streamsize read_count=is.gcount();
    if(read_count==0&&!is.good()){return NULL;}
    if(read_count != t.size) { COMMA_THROW(comma::exception, "read count mismatch, expected: " << t.size << " bytes; got: " << read_count );}
    return &t;
}

template<>
const timestamped_output_t* app<timestamped_output_t>::read()
{
    static timestamped_output_t reply;
    if(!std::cin.good()) { return NULL; }
    std::cin.read(reinterpret_cast<char*>(&reply.t), sizeof(boost::posix_time::ptime));
    if(!std::cin.good()) { return NULL; }
    if(read_packet(std::cin, reply.data)==NULL){return NULL;}
    return &reply;
}

template<>
const output_t* app<output_t>::read()
{
    static output_t reply;
    if(!std::cin.good()) { return NULL; }
    return read_packet(std::cin, reply);
}

template<typename T>
void app<T>::process(const comma::csv::options& csv)
{
    comma::csv::output_stream<T> os(std::cout, csv);
    while(std::cin.good())
    {
        const T* reply=read();
        if(reply==NULL) { return; }
        os.write(*reply);
    }
}

int main( int ac, char** av )
{
    comma::command_line_options options( ac, av, usage );
    try
    {
        if(options.exists("--timestamp"))
            app<timestamped_output_t>::run(options);
        else
            app<output_t>::run(options);
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; return 1;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; return 1;
    }
    return 0;

}
