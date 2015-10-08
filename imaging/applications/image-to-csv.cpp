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
#include <vector>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <boost/test/utils/nullstream.hpp>
#include "../cv_mat/serialization.h"
#include <comma/name_value/parser.h>
#include <comma/csv/options.h>
#include <memory>

static const char* app_name="image-to-csv";
boost::onullstream nullstream;
std::ostream* cverbose=&nullstream;
snark::cv_mat::serialization::options input_options;
snark::cv_mat::serialization::header header;
comma::csv::options csv;
int channels=3;
int depth=0;
comma::signal_flag signaled;

struct app_t
{
    virtual void output_fields()=0;
    virtual void output_format()=0;
    virtual bool process_image()=0;
};

template<typename T>
struct output_t:public app_t
{
    boost::posix_time::ptime t;
    int x;
    int y;
    std::vector<T> channel;
    output_t() : x(0), y(0), channel(channels)
    {
        
    }
    virtual void output_fields();
    virtual void output_format();
    virtual bool process_image();
};

namespace comma { namespace visiting {

template <typename T> struct traits< output_t<T> >
{
    template < typename K, typename V >
    static void visit( const K&, output_t<T>& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        for(int i=0;i<r.channel.size();i++)
        {
            std::string s="channel[";
            s+=boost::lexical_cast<std::string>(i);
            s+= "]";
            v.apply( s.c_str(), r.channel[i] );
        }
    }

    template < typename K, typename V >
    static void visit( const K&, const output_t<T>& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        for(int i=0;i<r.channel.size();i++)
        {
            std::string s="channel[";
            s+=boost::lexical_cast<std::string>(i);
            s+= "]";
            v.apply( s.c_str(), r.channel[i] );
        }
    }
};

} } // namespace comma { namespace visiting {

static void usage(bool detail)
{
    std::cerr << std::endl;
    std::cerr << "read images with header from stdin and output pixels x,y and data to stdout" << std::endl;
    if( !detail ) { std::cerr << "see --help --verbose for more details" << std::endl; }
    std::cerr << std::endl;
    std::cerr<< "usage: " << app_name << " <options>" << std::endl
                    << std::endl
                    << "options" << std::endl
                    << "    --help,-h: show help" << std::endl
                    << "    --verbose,-v: show detailed messages" << std::endl
                    << "    --channels=<n>: specify single channel type in input options and number of channels here; e.g. for image without header --input=\"type=0\" --channels=256" << std::endl
                    << "    --output-fields: output fields and exit need to feed in image header through stdin or specify --input options" << std::endl
                    << "    --output-format: output format string and exit need to feed in image header through stdin or specify --input options" << std::endl
                    << "    --output-header-fields: output header fields and exit" << std::endl
                    << "    --output-header-format: output header format and exit" << std::endl
                    << "    --input: input options if image has no header (see details)" << std::endl
                    << std::endl;
    if(detail)
    {
        std::cerr<< snark::cv_mat::serialization::options::usage() << std::endl
                        << std::endl;
        std::cerr<< "output stream csv options:" << std::endl
                        << csv.usage() << std::endl
                        << std::endl;
    }
    std::cerr<< "example" << std::endl
                    << "      cv-cat --input=image.jpg | " << app_name << " >pixels.csv" << std::endl
                    << std::endl;
    exit(0);
}

void read_header()
{
    comma::csv::options csvh;
    csvh.fields=input_options.fields;
    csvh.format(comma::csv::format::value< snark::cv_mat::serialization::header >(input_options.fields, false));
    comma::csv::input_stream<snark::cv_mat::serialization::header> is(std::cin, csvh);
    const snark::cv_mat::serialization::header* p=is.read();
    if(p!=NULL)
        header = *p;
    else
        exit(0);
}
template<typename T>
void output_t<T>::output_fields()
{
    std::cout<<comma::join( comma::csv::names< output_t<T> >(), ','  ) << std::endl;
}
template<typename T>
void output_t<T>::output_format()
{
    std::cout<<comma::csv::format::value< output_t<T> >(csv.fields, false) << std::endl;
}
template<typename T>
bool output_t<T>::process_image()
{
    comma::csv::output_stream<output_t<T> > os(std::cout, csv);
    output_t<T> out;
    out.channel.resize(channels);
    out.t=header.timestamp;
    int size=channels*sizeof(T);
    //process one image
    for(int j=0;j<header.rows;j++)
    {
        for(int i=0;i<header.cols;i++)
        {
            if(!std::cin.good() || std::cin.eof()  || signaled) { return false; }
            out.x=i;
            out.y=j;
            std::cin.read((char*)(T*)&out.channel[0],size);
            os.write(out);
        }
    }
    return true;
}
std::auto_ptr<app_t> get_app()
{
        switch(depth)
        {
            case CV_8U:
                return std::auto_ptr<app_t>(new output_t<unsigned char>());
                break;
            case CV_8S:
                return std::auto_ptr<app_t>(new output_t<char>());
                break;
            case CV_16U:
                return std::auto_ptr<app_t>(new output_t<comma::uint16>());
                break;
            case CV_16S:
                return std::auto_ptr<app_t>(new output_t<comma::int16>());
                break;
            case CV_32S:
                return std::auto_ptr<app_t>(new output_t<comma::int32>());
                break;
            case CV_32F:
                return std::auto_ptr<app_t>(new output_t<float>());
                break;
            case CV_64F:
                return std::auto_ptr<app_t>(new output_t<double>());
                break;
            default:
                COMMA_THROW(comma::exception, "unsupported depth " <<  depth);
        }
}

void process()
{
    #ifdef WIN32
    _setmode( _fileno( stdin ), _O_BINARY );
    #endif
    std::ios_base::sync_with_stdio( false ); // std::cin, std::cout access are thread-unsafe now (safe by default)
    std::cin.tie( NULL ); // std::cin is tied to std::cout by default, which is thread-unsafe now
    while(std::cin.good() && !signaled && std::cin.peek()!=-1)
    {
        if(!input_options.no_header) 
        {
            read_header();
            channels=CV_MAT_CN(header.type);
            depth=CV_MAT_DEPTH(header.type);
        }
        *cverbose<<app_name<<": timestamp: "<<header.timestamp<<" cols: "<<header.cols<<" rows: "<<header.rows<<" type: "<<header.type<<std::endl;
        *cverbose<<app_name<<": channels: "<<channels<<" depth: "<<depth<<std::endl;
        //we need new app_t because channel/depth may change after reading header
        if(!get_app()->process_image()) { break; }
    }
}
int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        cverbose =  options.exists( "--verbose,-v" ) ? &std::cerr : &nullstream;
        
        input_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value<std::string>("--input", "") );
        if(input_options.fields.empty()) { input_options.fields = "t,rows,cols,type"; }
        csv=comma::csv::options(options);
        header=input_options.get_header();
        if(options.exists("--channels"))
            channels=options.value<int>("--channels");
        else
            channels=CV_MAT_CN(header.type);
        depth=CV_MAT_DEPTH(header.type);
        *cverbose<<app_name<<": type: "<<header.type<<" channels: "<<channels<<" depth: "<<int(CV_MAT_DEPTH(header.type))<<std::endl;
        std::vector< std::string > unnamed = options.unnamed( "--output-fields,--output-format,--verbose,-v,--output-header-fields,--output-header-format,--flush", 
                                                              "--input,--channels,--binary,-b,--fields,-f,--precision,--quote,--delimiter,-d");
        if(!unnamed.empty())
        {
            std::cerr<<"invalid option"<<((unnamed.size()>1)?"s":"")<<": "<< comma::join(unnamed, ',') <<std::endl;
            return 1;
        }
        
        if (options.exists("--output-fields")) { get_app()->output_fields();exit(0); }
        if (options.exists("--output-format")) { get_app()->output_format(); exit(0); }
        if (options.exists("--output-header-fields")) { std::cout<<comma::join( comma::csv::names< snark::cv_mat::serialization::header >(input_options.fields), ','  ) << std::endl; exit(0); }
        if (options.exists("--output-header-format")) { std::cout<<comma::csv::format::value< snark::cv_mat::serialization::header >(input_options.fields, false) << std::endl; exit(0); }
        process();
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "unknown exception" << std::endl;
    }
    return 1;
}
