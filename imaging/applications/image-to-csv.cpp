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
std::size_t channels=3;
int depth=0;
bool output_non_zeroes;

struct app_t
{
    virtual ~app_t(){}
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
    std::vector<T> channels;
    output_t() : x(0), y(0), channels(::channels)
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
        v.apply( "channels", r.channels );
    }

    template < typename K, typename V >
    static void visit( const K&, const output_t<T>& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "x", r.x );
        v.apply( "y", r.y );
        v.apply( "channels", r.channels );
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
                    << "    --output-non-zero: output only values with non-zero data" << std::endl
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
                    << "      cv-cat --file image.jpg | " << app_name << " > pixels.csv" << std::endl
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
    if( !p ) { exit( 0 ); }
    header = *p;
}

bool zeroes(char* p, std::size_t size)
{
    for ( char* i = p; i < p + size; ++i ) { if (*i != 0) { return false; } }
    return true;
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
    out.channels.resize(::channels);
    out.t=header.timestamp;
    std::size_t size=::channels*sizeof(T);
    //process one image
    for(comma::uint32 j=0;j<header.rows && std::cin.good() ;j++)
    {
        std::vector<char> buffer(header.cols*size);
        std::cin.read(&buffer[0], buffer.size());
        if(std::size_t(std::cin.gcount())!=buffer.size()) { return false; }
        char* ptr=&buffer[0];
        for(comma::uint32 i=0;i<header.cols;i++)
        {
            if (output_non_zeroes && zeroes(ptr, size))
            {
                ptr+=size;
            }
            else
            {
                out.x=i;
                out.y=j;
                std::memcpy(&out.channels[0], ptr, size);
                os.write(out);
                ptr+=size;
            }
        }
    }
    return true;
}
struct get_app
{
    app_t* app;
    get_app():app(NULL)
    {
        switch(depth)
        {
            case CV_8U:
                app=new output_t<unsigned char>();
                break;
            case CV_8S:
                app=new output_t<char>();
                break;
            case CV_16U:
                app=new output_t<comma::uint16>();
                break;
            case CV_16S:
                app=new output_t<comma::int16>();
                break;
            case CV_32S:
                app=new output_t<comma::uint32>();
                break;
            case CV_32F:
                app=new output_t<float>();
                break;
            case CV_64F:
                app=new output_t<double>();
                break;
            default:
                COMMA_THROW(comma::exception, "unsupported depth " <<  depth);
        }
    }
    ~get_app()
    {
        if(app!=NULL)
        {
            delete app;
            app=NULL;
        }
    }
};

void process()
{
    #ifdef WIN32
    _setmode( _fileno( stdin ), _O_BINARY );
    #endif
    while(std::cin.good() && std::cin.peek()!=-1)
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
        get_app g;
        if(!g.app->process_image()) { break; }
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
        channels = options.value< int >( "--channels", CV_MAT_CN( header.type ) );
        depth = CV_MAT_DEPTH( header.type );
        *cverbose<<app_name<<": type: "<<header.type<<" channels: "<<channels<<" depth: "<<int(CV_MAT_DEPTH(header.type))<<std::endl;
        std::vector< std::string > unnamed = options.unnamed( "--output-fields,--output-format,--verbose,-v,--output-header-fields,--output-header-format,--output-non-zero,--flush", 
                                                              "--input,--channels,--binary,-b,--fields,-f,--precision,--quote,--delimiter,-d");
        if(!unnamed.empty()) { std::cerr<<"invalid option"<<((unnamed.size()>1)?"s":"")<<": "<< comma::join(unnamed, ',') <<std::endl; return 1; }
        get_app g;
        if (options.exists("--output-fields")) { g.app->output_fields();exit(0); }
        if (options.exists("--output-format")) { g.app->output_format(); exit(0); }
        if (options.exists("--output-header-fields")) { std::cout<<comma::join( comma::csv::names< snark::cv_mat::serialization::header >(input_options.fields), ','  ) << std::endl; exit(0); }
        if (options.exists("--output-header-format")) { std::cout<<comma::csv::format::value< snark::cv_mat::serialization::header >(input_options.fields, false) << std::endl; exit(0); }
        
        output_non_zeroes = options.exists("--output-non-zero");
        
        process();
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "image-to-csv: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "image-to-csv: unknown exception" << std::endl;
    }
    return 1;
}
