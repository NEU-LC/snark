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

#include <vector>
#include <iostream>
#include <iterator>
#include <math.h>
#include <comma/visiting/traits.h>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <fftw3.h>

std::size_t input_size=0;
bool filter_input=true;
bool logarithmic_output=true;

void usage(bool detail)
{
    std::cerr<<"    perform fft on input data" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr<< "    output is binary array of double with half the size of input"  << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --size: size of input vector" << std::endl;
    std::cerr << "    --no-filter: when not specified, filters input using a cut window to get limited output" << std::endl;
    std::cerr << "    --linear: output as linear; when not specified, output will be scaled to lograithm of 10" << std::endl;
    std::cerr << "    --output-size: print size of output record in bytes and exit" << std::endl;
    std::cerr << "    --output-format: print binary format of output and exit" << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "example" << std::endl;
    std::cerr << "      " << comma::verbose.app_name() << " --binary=\"t,16000f\" --fields=,data --size=16000" << std::endl;
    std::cerr << std::endl;
    exit(0);
}

struct input_t
{
    std::vector<double> data;
    input_t() : data(input_size) {}

};

namespace comma { namespace visiting {

//alternatively: use snark timing traits.h: {t, data/output_t}
template <> struct traits< input_t >
{
    template< typename K, typename V > static void visit( const K& k, input_t& p, V& v )
    {
        v.apply( "data", p.data );
    }
    template< typename K, typename V > static void visit( const K& k, const input_t& p, V& v )
    {
        v.apply( "data", p.data );
    }
};

} } // namespace comma { namespace visiting {

template<typename T>
T* allocate_fftw_array(std::size_t size)
{
    return reinterpret_cast<T*>(fftw_malloc(sizeof(T)*size));
}

// a quick and dirty helper class
struct fft
{
    std::vector<double> h;
    double* input;
    fftw_complex* output;
    fftw_plan plan;

    fft(std::size_t size) : h( size ), input(allocate_fftw_array<double>(size)) ,
        output(allocate_fftw_array<fftw_complex>(size / 2 + 1 )) ,
        plan( fftw_plan_dft_r2c_1d( size, input, output, 0 ) )
    {
        for( std::size_t i = 0; i < size; ++i ) { h[i] = 0.54 - 0.46 * std::cos( M_PI * 2 * i / size ); }
    }
    ~fft()
    {
        fftw_destroy_plan(plan);
        fftw_free( input ); // seems that fftw_destroy_plan() releases it
        fftw_free( output ); // seems that fftw_destroy_plan() releases it
    }
    void calculate() { fftw_execute( plan); }
};

void calculate(const input_t* input, std::vector<double>& output)
{
    static fft fft(input_size);
    if(filter_input)
    {
        for(std::size_t i=0;i<input_size;i++) { fft.input[i] = fft.h[i] * input->data[i]; }
    }
    else { memcpy(fft.input, input->data.data(), input->data.size() * sizeof(double)); }
    fft.calculate();
    for(std::size_t j=0;j<output.size();j++)
    {
        double a= std::abs( std::complex<double>( fft.output[j][0], fft.output[j][1] ) );
        if(logarithmic_output) { a = (a == 0) ? 0 : (std::log10(a)); }
        output[j]=a;
    }
}

struct app
{
    std::vector<double> output;
    app() : output(input_size/2) { }
    void process(const comma::csv::options& csv)
    {
        input_t sample;
        comma::csv::input_stream<input_t> is(std::cin, csv, sample);
        while(std::cin.good())
        {
            //read a record
            const input_t* input=is.read();
            if(input != NULL)
            {
                calculate(input, output);
                //write output
                write_output();
            }
        }
    }
    void write_output()
    {
        std::cout.write(reinterpret_cast<const char*>(&output[0]),output.size()*sizeof(double));
    }
};

template<typename T>
std::ostream& operator<< (std::ostream& o, const std::vector<T>& v)
{
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(o, " "));
    return o;
}

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        comma::csv::options csv(options);
        filter_input=options.exists("--no-filter");
        logarithmic_output=options.exists("--linear");
        input_size=options.value<std::size_t>("--size");
        std::vector<std::string> unnamed=options.unnamed("--verbose,-v,--output-size,--output-format,--no-filter,--linear,--timestamp", 
                                                         "--binary,-b,--fields,-f,--delimiter,-d,--size");
        if(unnamed.size() != 0) { COMMA_THROW(comma::exception, "invalid option(s): " << unnamed ); }
        app app;
        if(options.exists("--output-size")) { std::cout<< app.output.size() << std::endl; return 0; }
        if(options.exists("--output-format")) { std::cout<< app.output.size() << "d" << std::endl; return 0; }
        app.process(csv);
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
