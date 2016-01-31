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

/// @author vsevolod vlaskine

#include <cmath>
#include <deque>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "simple wrapper for eigen library operations" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat sample.csv | math-eigen [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    eigen (default): calculate eigen vector and eigen values on a sample" << std::endl;
    std::cerr << "        fields" << std::endl;
    std::cerr << "            block: block number; output eigen vectors and eigen values for each" << std::endl;
    std::cerr << "                   contiguous block of samples with the same block id" << std::endl;
    std::cerr << "            data: sample data" << std::endl;
    std::cerr << "            default: data" << std::endl;
    std::cerr << "        output" << std::endl;
    std::cerr << "            one eigen vector per line; if block field present: vector,value,block" << std::endl;
    std::cerr << "                                       if no block field present: vector,value" << std::endl;
    std::cerr << "            binary format: 32-bit unsigned integer for block, doubles for other output fields" << std::endl;
    std::cerr << "        options" << std::endl;
    std::cerr << "            --normalize,-n: output normalized eigen values" << std::endl;
    std::cerr << "            --size: a hint of number of elements in the data vector, ignored, if data indices" << std::endl;
    std::cerr << "                    specified, e.g. data[0],data[1],data[2]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show this help; --help --verbose: more help" << std::endl;
    if( verbose ) { std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    eigen" << std::endl;
    std::cerr << "        cat sample.csv | math-eigen" << std::endl;
    std::cerr << "        cat sample.csv | math-eigen --fields=block,data" << std::endl;
    std::cerr << "        cat sample.csv | math-eigen --fields=block,,,data --size=4" << std::endl;
    std::cerr << "        cat sample.csv | math-eigen --fields=block,,data[0],,data[1],,data[2],,data[3]" << std::endl;
    std::cerr << "        cat sample.bin | math-eigen --fields=block,data --binary=ui,6d" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static boost::optional< unsigned int > size;

namespace snark { namespace eigen {

struct input_t
{
    std::vector< double > data;
    comma::uint32 block;
    
    input_t() : data( *size ), block( 0 ) {}
};

struct output_t
{
    std::vector< double > vector;
    double value;
    comma::uint32 block;
    
    output_t() : vector( *size ), value( 0 ), block( 0 ) {}
};

} } // namespace snark { namespace eigen {

namespace comma { namespace visiting {

template <> struct traits< snark::eigen::input_t >
{
    template < typename K, typename V > static void visit( const K&, snark::eigen::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const snark::eigen::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }
};

template <> struct traits< snark::eigen::output_t >
{
    template < typename K, typename V > static void visit( const K&, snark::eigen::output_t& p, V& v )
    {
        v.apply( "vector", p.vector );
        v.apply( "value", p.value );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const snark::eigen::output_t& p, V& v )
    {
        v.apply( "vector", p.vector );
        v.apply( "value", p.value );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

// Eigen::VectorXf normalize(Eigen::VectorXf vec) {
//   for (int i = 0; i < vec.size(); i++) { // normalize each feature.
//       vec[i] = (vec[i] - minCoeffs[i]) / scalingFactors[i];
//   }
//   return vec;
// }
// 
// // Calculate normalization coefficients (globals of type Eigen::VectorXf). 
// maxCoeffs = traindata.colwise().maxCoeff();
// minCoeffs = traindata.colwise().minCoeff();
// scalingFactors = maxCoeffs - minCoeffs;
// 
// // For each datapoint.
// for (int i = 0; i < traindata.rows(); i++) { // Normalize each datapoint.
//   traindata.row(i) = normalize(traindata.row(i));
// }
// 
// // Mean centering data.
// Eigen::VectorXf featureMeans = traindata.colwise().mean();
// Eigen::MatrixXf centered = traindata.rowwise() - featureMeans;
// 
// // Compute the covariance matrix.
// Eigen::MatrixXf cov = centered.adjoint() * centered;
// cov = cov / (traindata.rows() - 1);
// 
// Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);
// // Normalize eigenvalues to make them represent percentages.
// Eigen::VectorXf normalizedEigenValues =  eig.eigenvalues() / eig.eigenvalues().sum();
// 
// 
// // Get the two major eigenvectors and omit the others.
// Eigen::MatrixXf evecs = eig.eigenvectors();
// Eigen::MatrixXfpcaTransform = evecs.rightCols(2);
// 
// 
// // Map the dataset in the new two dimensional space.
// traindata = traindata * pcaTransform;

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        const std::vector< std::string >& unnamed = options.unnamed( "--flush,--normalize,-n,--verbose,-v" );
        std::string operation = unnamed.empty() ? std::string( "eigen" ) : unnamed[0];
        if( operation == "eigen" )
        {
            size = options.optional< unsigned int >( "--size" );
            if( csv.fields.empty() ) { csv.fields = "data"; }
            std::string first;
            if( !size )
            {
                const std::vector< std::string >& fields = comma::split( csv.fields, ',' );
                if( csv.has_field( "data" ) )
                {
                    unsigned int count;
                    if( csv.binary() )
                    {
                        count = csv.format().count();
                    }
                    else
                    {
                        while( std::cin.good() && first.empty() ) { std::getline( std::cin, first ); }
                        count = comma::split( first, csv.delimiter ).size(); // quick and dirty, wasteful
                    }
                    size = count - fields.size() + 1;
                }
                else
                {
                    unsigned int max = 0;
                    for( unsigned int i = 0; i < fields.size(); ++i )
                    {
                        if( fields[i].substr( 0, 5 ) == "data[" && *fields[i].rbegin() == ']' ) { unsigned int k = boost::lexical_cast< unsigned int >( fields[i].substr( 5, fields[i].size() - 6 ) ) + 1; if( k > max ) { max = k; } }
                    }
                    if( max == 0 ) { std::cerr << "math-eigen: please specify valid data fields" << std::endl; return 1; }
                    size = max;
                }
            }
            bool normalize = options.exists( "--normalize,-n" );
            comma::csv::options output_csv;
            bool has_block = csv.has_field( "block" );
            output_csv.fields = has_block ? "vector,value,block" : "vector,value";
            std::string s = boost::lexical_cast< std::string >( *size + 1 );
            if( csv.binary() ) { output_csv.format( has_block ? s + ",ui" : s ); }
            std::deque< snark::eigen::input_t > buffer;
            if( !first.empty() ) { buffer.push_back( comma::csv::ascii< snark::eigen::input_t >( csv ).get( first ) ); }
            comma::csv::input_stream< snark::eigen::input_t > istream( std::cin, csv );
            comma::csv::output_stream< snark::eigen::output_t > ostream( std::cout, output_csv );
            typedef Eigen::Matrix< double, -1, -1, Eigen::RowMajor > matrix_t;
            while( true )
            {
                const snark::eigen::input_t* p = istream.read();
                if( !p || ( !buffer.empty() && buffer.front().block != p->block ) )
                {
                    if( buffer.size() == 1 ) { std::cerr << "math-eigen: on block " << buffer.front().block << ": expected block with at least two entries, got only one" << std::endl; return 1; }
                    matrix_t sample( buffer.size(), *size );
                    for( std::size_t i = 0; i < buffer.size(); ++i ) // todo: hm... dodgy? use Eigen::Map instead?
                    {
                        ::memcpy( &sample( i, 0 ), &buffer[i].data[0], buffer[i].data.size() * sizeof( double ) );
                    }
                    matrix_t covariance = sample.adjoint() * sample;
                    covariance = covariance / ( sample.rows() - 1 );                    
                    Eigen::SelfAdjointEigenSolver< matrix_t > solver( covariance );
                    Eigen::VectorXd values = solver.eigenvalues();
                    if( normalize ) { values = values / solver.eigenvalues().sum(); }
                    const matrix_t& vectors = solver.eigenvectors().transpose();
                    // todo: get the two major eigenvectors and omit the others.
                    // Eigen::MatrixXf evecs = eig.eigenvectors();
                    // Eigen::MatrixXfpcaTransform = evecs.rightCols(2);
                    for( std::size_t i = 0; i < *size; ++i )
                    {
                        snark::eigen::output_t output;
                        output.block = buffer.front().block;
                        ::memcpy( &output.vector[0], &vectors( i, 0 ), *size * sizeof( double ) );
                        output.value = values[i];
                        ostream.write( output );
                    }
                    buffer.clear();
                }
                if( !p ) { break; }
                buffer.push_back( *p );
            }
            return 0;
        }
        std::cerr << "math-eigen: expected operation, got \"" << operation << "\"" << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "math-eigen: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "math-eigen: unknown exception" << std::endl; }
    return 1;
}
