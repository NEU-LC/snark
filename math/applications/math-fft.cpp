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
#include <boost/optional.hpp>
#include <comma/visiting/traits.h>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <fftw3.h>

std::size_t input_size=0;
bool filter_input=true;
bool logarithmic_output=true;
bool magnitude=false;
bool real=false;
bool split=false;
bool timestamp=false;
std::size_t bin_size=0;
boost::optional<double> bin_overlap;

void usage(bool detail)
{
    std::cerr<<"    perform fft on input data" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr<< "    output is binary array of pair (real, complex) of double with half the size of input; plus optional timestamp"  << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --size=<size>: size of input vector" << std::endl;
    std::cerr << "    --bin-size=[<size>]: cut data into several bins of this size and perform fft on each bin, when not speificed calculates fft on the whole data" << std::endl;
    std::cerr << "    --bin-overlap=[<overlap>]: if specified, each bin will contain this portion of the last bin's data, range: 0 (no overlap) to 1"<<std::endl;
    std::cerr << "    --magnitude: output magnitude only" << std::endl;
    std::cerr<< "        output is binary array of double with half the size of input"  << std::endl;
    std::cerr << "    --real: output real part only" << std::endl;
    std::cerr<< "        output is binary array of double with half the size of input"  << std::endl;
    std::cerr << "    --no-filter: when not specified, filters input using a cut window to get limited output" << std::endl;
    std::cerr << "    --linear: output as linear; when not specified, output will be scaled to lograithm of 10 for magnitude or real part (phase is not affected)" << std::endl;
    std::cerr << "    --split: output array of real followed by array of complex part; when not specified real and complex parts are interleaved" << std::endl;
    std::cerr << "    --output-size: print size of output record in bytes and exit" << std::endl;
    std::cerr << "    --output-format: print binary format of output and exit" << std::endl;
    std::cerr << "    --timestamp: prepend timestamp to the output data" << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
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
    boost::posix_time::ptime time;
    std::vector<double> data;
    input_t() : data(input_size) {}

};

namespace comma { namespace visiting {

//alternatively: use snark timing traits.h: {t, data/output_t}
template <> struct traits< input_t >
{
    template< typename K, typename V > static void visit( const K& k, input_t& p, V& v )
    {
        v.apply( "t", p.time );
        v.apply( "data", p.data );
    }
    template< typename K, typename V > static void visit( const K& k, const input_t& p, V& v )
    {
        v.apply( "t", p.time );
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
    std::size_t output_size() const { return h.size() / 2; }
};

void calculate(const double* data, std::size_t size, std::vector<double>& output)
{
    fft fft(size);
    if(filter_input)
    {
        for(std::size_t i=0;i<size;i++) { fft.input[i] = fft.h[i] * data[i]; }
    }
    else { memcpy(fft.input, data, size * sizeof(double)); }
    
    fft.calculate();
    
    if(magnitude)
    {
        if(fft.output_size()>output.size()) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
        for(std::size_t j=0;j<fft.output_size();j++)
        {
            double a= std::abs( std::complex<double>( fft.output[j][0], fft.output[j][1] ) );
            if(logarithmic_output) { a = (a == 0) ? 0 : (std::log10(a)); }
            output[j]=a;
        }
    }
    else if(real)
    {
        if(fft.output_size()>output.size()) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
        for(std::size_t j=0;j<fft.output_size();j++)
        {
            double a= fft.output[j][0];
            if(logarithmic_output) { a = (a == 0) ? 0 : (std::log10(a)); }
            output[j]=a;
        }
    }
    else
    {
        std::size_t k=0;
        std::size_t step=2;
        std::size_t off=1;
        if(split)
        {
            step=1;
            off=output.size()/2;
        }
        if(2*fft.output_size()>output.size()) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
        for(std::size_t j=0; j<fft.output_size(); j++, k+=step)
        {
            double a= fft.output[j][0];
            if(logarithmic_output) { a = (a == 0) ? 0 : (std::log10(a)); }
            output[k]=a;
            output[k+off]=fft.output[j][1];
        }
    }
}
void write_timestamp_bin(std::ostream& os, const boost::posix_time::ptime& time)
{
    static char buf[comma::csv::format::traits< boost::posix_time::ptime, comma::csv::format::time >::size];
    comma::csv::format::traits< boost::posix_time::ptime, comma::csv::format::time >::to_bin( time, buf );
    os.write(buf, sizeof(buf));
}
struct app
{
    std::vector<double> output;
    app()
    {
        std::size_t len=bin_size;
        if(magnitude || real) { len/=2; }
        output.resize(len);
    }
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
                if(bin_overlap && int(bin_size*(1-*bin_overlap)) <= 0) { COMMA_THROW( comma::exception, "bin size and overlap don't work" ); }
                for(std::size_t bin_offset=0;bin_offset<input_size;bin_offset+=(bin_overlap ? bin_size*(1-*bin_overlap) : bin_size))
                {
                    memset(output.data(),0,output.size()*sizeof(output[0]));
                    calculate(&input->data.data()[bin_offset], std::min(bin_size,input_size-bin_offset), output);
                    //write output
                    write_output(*input);
                }
            }
        }
    }
    void write_output(const input_t& input)
    {
        if(timestamp) { write_timestamp_bin(std::cout, input.time); }
        std::cout.write(reinterpret_cast<const char*>(&output[0]),output.size()*sizeof(double));
    }
    std::size_t get_output_size()
    {
        return output.size() + (timestamp ? comma::csv::format::traits< boost::posix_time::ptime, comma::csv::format::time >::size : 0);
    }
    std::string get_output_format()
    {
        return (timestamp ? "t," : "") + boost::lexical_cast<std::string>(output.size()) + "d";
    }
};

template<typename T>
std::ostream& operator<< (std::ostream& o, const std::vector<T>& v)
{
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(o, " "));
    return o;
}
template<typename T>
void range_check(T value, T min, T max, const char* label)
{
    if(value<min || value>max) { COMMA_THROW(comma::exception, label<<" out of range "<<min<<" to "<<max ); }
}

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        comma::csv::options csv(options,"data");
        filter_input= ! options.exists("--no-filter");
        logarithmic_output= ! options.exists("--linear");
        input_size=options.value<std::size_t>("--size");
        magnitude=options.exists("--magnitude");
        real=options.exists("--real");
        split=options.exists("--split");
        timestamp=options.exists("--timestamp");
        bin_size=options.value<std::size_t>("--bin-size", input_size);
        range_check<std::size_t>(bin_size,0,input_size,"bin_size");
        bin_overlap=options.optional<double>("--bin-overlap");
        if(bin_overlap)
            range_check<double>(*bin_overlap,0,1,"bin_overlap");
        std::vector<std::string> unnamed=options.unnamed("--verbose,-v,--output-size,--output-format,--no-filter,--linear,--timestamp,--magnitude,--real,--split", 
                                                         "--binary,-b,--fields,-f,--delimiter,-d,--size,--bin-size,--bin-overlap");
        if(unnamed.size() != 0) { COMMA_THROW(comma::exception, "invalid option(s): " << unnamed ); }
        app app;
        if(options.exists("--output-size")) { std::cout<< app.get_output_size() << std::endl; return 0; }
        if(options.exists("--output-format")) { std::cout<< app.get_output_format() << std::endl; return 0; }
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
