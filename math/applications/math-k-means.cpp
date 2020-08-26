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

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <deque>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/optional/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/format.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/string/split.h>
#include <tbb/blocked_range.h>
#define TBB_PREVIEW_GLOBAL_CONTROL 1 // required to #include <tbb/global_control.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include "../../visiting/traits.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "run k-means on input data" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat sample.csv | math-k-means <operation> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields" << std::endl;
    std::cerr << "    block: block number; output k-means centroid id for each" << std::endl;
    std::cerr << "           contiguous block of samples with the same block id" << std::endl;
    std::cerr << "    data: vector of size <size>" << std::endl;
    std::cerr << "    default: data" << std::endl;
    std::cerr << "output" << std::endl;
    std::cerr << "    default output fields" << std::endl;
    std::cerr << "        one point per line; if block field present: data,block,centroid/data,centroid/id" << std::endl;
    std::cerr << "                            if no block field present: data,centroid/data,centroid/id" << std::endl;
    std::cerr << "    binary format: 64-bit floating-point for centroid/data, 32-bit unsigned integer for centroid/id" << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show this help; --help --verbose: more help" << std::endl;
    std::cerr << "    --max-iterations,--iterations=<n>: number of iterations for Lloyd's algorithm; default: 300" << std::endl;
    std::cerr << "    --max-threads,--threads=<n>: maximum number of threads to run, if less than 0, set to number of cores in system (" << std::thread::hardware_concurrency() << ')' << "; default: -1" << std::endl;
    std::cerr << "    --number-of-clusters,--clusters=<n>: number of k-means cluster per block/all" << std::endl;
    std::cerr << "    --number-of-runs,--runs=<n>: number of times to run k-means, best run is one with lowest inertia (sum of distance of cluster points to cluster centroid); default: 10" << std::endl;
    std::cerr << "    --size=[<n>]: a hint of number of elements in the data vector; ignored, if data indices specified, e.g. data[0],data[1],data[2]" << std::endl;
    std::cerr << "    --tolerance=<distance>: difference between two consecutive iteration centroid l2 norms to declare convergence and stop iterating in LLoyd's algorithm; default: 1e-4" << std::endl;
    if( verbose ) { std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    cat sample.csv | math-k-means --number-of-clusters=3" << std::endl;
    std::cerr << "    cat sample.csv | math-k-means --number-of-clusters=3 --fields=block,data --size=3" << std::endl;
    std::cerr << "    cat sample.csv | math-k-means --number-of-clusters=3 --fields=data[0],data[1],data[2]" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static comma::csv::options csv;
static boost::optional< unsigned int > size = boost::none;
static bool verbose = false;

namespace snark { namespace k_means {

using data_t = std::vector< double >;

struct input_t
{
    data_t data;
    comma::uint32 block;
    input_t() : data( *size ), block{ 0 } {}
};

} } // namespace snark { namespace k_means {

namespace comma { namespace visiting {

template <> struct traits< snark::k_means::input_t >
{
    template < typename K, typename V > static void visit( const K&, snark::k_means::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const snark::k_means::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

static double square( double value ) { return value * value; }

static double squared_l2_distance( const snark::k_means::data_t& first, const snark::k_means::data_t& second )
{
    double ret = 0.0;
    for( size_t i = 0; i < first.size(); ++i ) { ret += ::square( first[i] - second[i] ); }
    return ret;
}

namespace snark { namespace k_means {
struct k_means {
    const double tolerance;
    const unsigned int max_iterations;
    const unsigned int number_of_runs;
    const comma::uint32 number_of_clusters;
    k_means( double tolerance, unsigned int max_iterations, unsigned int number_of_runs, comma::uint32 number_of_clusters ) noexcept :
        tolerance( tolerance ),
        max_iterations( max_iterations ),
        number_of_runs( number_of_runs ),
        number_of_clusters( number_of_clusters ) {}
    std::vector< data_t > initialize_centroids( const std::deque< std::pair< input_t, std::string > >& inputs ) const
    {
        static std::random_device seed;
        static std::mt19937 generator( seed() );
        std::vector< data_t > centroids( number_of_clusters );
        std::uniform_int_distribution< size_t > indices( 0, inputs.size() - 1 );
        std::generate( centroids.begin(), centroids.end(), [&]() { return inputs[indices( generator )].first.data; } );
        return centroids;
    }
    std::vector< comma::uint32 > assign_centroids( const std::vector< data_t >& centroids, const std::deque< std::pair< input_t, std::string > >& inputs ) const
    {
        std::vector< comma::uint32 > centroid_assignments( inputs.size() );
        tbb::parallel_for( tbb::blocked_range< size_t >( 0, centroid_assignments.size() ), [&]( const tbb::blocked_range< size_t >& chunk ) {
            for( size_t point = chunk.begin(); point < chunk.end(); ++point )
            {
                auto best_distance = std::numeric_limits< double >::max();
                comma::uint32 best_centroid = 0;
                for( comma::uint32 centroid_i = 0; centroid_i < number_of_clusters; ++centroid_i )
                {
                    const double distance = ::squared_l2_distance( inputs[point].first.data, centroids[centroid_i] );
                    if( distance < best_distance )
                    {
                        best_distance = distance;
                        best_centroid = centroid_i;
                    }
                }
                centroid_assignments[point] = best_centroid;
            }
        });
        return centroid_assignments;
    }
    std::vector< data_t > update_centroids( const std::vector< comma::uint32 >& centroid_assignments, const std::deque< std::pair< input_t, std::string> >& inputs ) const
    {
        // for each cluster, calculate sum of data vectors and number of data vectors
        std::vector< size_t > size_of_centroid( number_of_clusters );
        std::vector< data_t > sum_of_points_in_centroid( number_of_clusters, data_t( *size, 0.0 ) );
        for( size_t point = 0; point < inputs.size(); ++point )
        {
            const auto centroid_i = centroid_assignments[point];
            for( size_t point_i = 0; point_i < inputs[point].first.data.size(); ++point_i ) { sum_of_points_in_centroid[centroid_i][point_i] += inputs[point].first.data[point_i]; }
            ++size_of_centroid[centroid_i];
        }
        // calculate new centroid means
        std::vector< data_t > new_centroids( number_of_clusters );
        tbb::parallel_for( tbb::blocked_range< comma::uint32 >( 0, number_of_clusters ), [&]( const tbb::blocked_range< comma::uint32 >& chunk ) {
            for( comma::uint32 centroid_i = chunk.begin(); centroid_i < chunk.end(); ++centroid_i )
            {
                new_centroids[centroid_i].reserve( *size );
                const auto centroid_size = std::max< size_t >( 1, size_of_centroid[centroid_i] ); // minimum size is at least 1
                for( size_t point_i = 0; point_i < sum_of_points_in_centroid[centroid_i].size(); ++point_i ) { new_centroids[centroid_i].emplace_back( sum_of_points_in_centroid[centroid_i][point_i] / centroid_size ); }
            }
        });
        return new_centroids;
    }
    // refer to http://www.goldsborough.me/c++/python/cuda/2017/09/10/20-32-46-exploring_k-means_in_python,_c++_and_cuda/
    int run( const std::deque< std::pair< input_t, std::string> >& inputs ) const
    {
        std::vector< size_t > all_scores( number_of_runs );  // keep track of score (sum of distance of points in cluster to cluster centroid) for each k means run
        std::vector< std::vector< data_t > > all_centroids( number_of_runs );
        std::vector< std::vector< comma::uint32 > > all_centroid_assignments( number_of_runs );
        tbb::parallel_for( tbb::blocked_range< unsigned int >( 0, number_of_runs ), [&]( const tbb::blocked_range< unsigned int >& chunk ) {
            for( unsigned int run = chunk.begin(); run < chunk.end(); ++run )
            {
                std::vector< data_t > run_centroids = initialize_centroids( inputs );
                std::vector< comma::uint32 > centroid_assignments;
                for( unsigned int iteration = 0; iteration < max_iterations; ++iteration )
                {
                    centroid_assignments = assign_centroids( run_centroids, inputs );
                    std::vector< data_t > new_centroids = update_centroids( centroid_assignments, inputs );
                    double difference = tbb::parallel_reduce( tbb::blocked_range< comma::uint32 >( 0, number_of_clusters ), 0.0,
                        [&]( const tbb::blocked_range< comma::uint32 > chunk, double difference ) -> double {
                            for( comma::uint32 centroid_i = chunk.begin(); centroid_i < chunk.end(); ++centroid_i )
                            {
                                // calculate distance of previous and new centroid
                                difference += std::sqrt( ::squared_l2_distance( run_centroids[centroid_i], new_centroids[centroid_i] ) );
                                // update previous centroid with new centroid
                                run_centroids[centroid_i] = new_centroids[centroid_i];
                            }
                            return difference;
                        },
                        std::plus<>()
                    );
                    if( difference < tolerance )
                    {
                        if( verbose )
                        {
                            std::fprintf(stderr, "math-k-means: sum of difference of consecutive cluster centroids (%.17g) has not changed by more than %.17g\n", difference, tolerance);
                            std::fprintf(stderr, "math-k-means: stopping at iteration %d of %d\n", iteration, max_iterations);
                        }
                        break;
                    }
                }
                double run_score = tbb::parallel_reduce( tbb::blocked_range< size_t >( 0, inputs.size() ), 0.0,
                    [&]( const tbb::blocked_range< size_t > chunk, double score ) -> double {
                        for( size_t point = chunk.begin(); point < chunk.end(); ++point ) { score += ::squared_l2_distance( inputs[point].first.data, run_centroids[centroid_assignments[point]] ); }
                        return score;
                    },
                    std::plus<>()
                );
                all_scores[run] = run_score;
                all_centroids[run] = std::move( run_centroids );
                all_centroid_assignments[run] = std::move( centroid_assignments );
            }
        });
        const auto& best_index = std::distance( all_scores.begin(), std::min_element( all_scores.begin(), all_scores.end() ) );
        if( verbose )
        {
            std::cerr << "math-k-means: all scores" << std::endl;
            std::cerr << "math-k-means:";
            for( const auto score : all_scores ) { std::cerr << ' ' << score; }
            std::cerr << std::endl;
            std::cerr << "math-k-means: run " << best_index << " has best score of " << all_scores[best_index] << std::endl;
        }
        for( size_t point = 0; point < inputs.size(); ++point )
        {
            const data_t& centroid = all_centroids[best_index][all_centroid_assignments[best_index][point]];
            std::cout << inputs[point].second;
            if( csv.binary() )
            {
                for( const auto centroid_i : centroid ) { std::cout.write( reinterpret_cast< const char* >( &centroid_i ), sizeof( centroid_i ) ); }
                std::cout.write( reinterpret_cast< const char* >( &all_centroid_assignments[best_index][point] ), sizeof( all_centroid_assignments[best_index][point] ) );
            }
            else
            {
                for( const auto centroid_i : centroid ) { std::cout << csv.delimiter << centroid_i; }
                std::cout << csv.delimiter << all_centroid_assignments[best_index][point] << std::endl;
            }
            if ( csv.flush ) { std::cout.flush(); }
        }
        return 0;
    }
};

static int run( const comma::command_line_options& options )
{
    csv = comma::csv::options( options, "data" );
    std::cout.precision( csv.precision );
    const auto max_iterations = options.value< unsigned int >( "--max-iterations,--iterations", 300 );
    if( max_iterations == 0 ) { std::cerr <<"math-k-means: got --max-iterations=0, --max-iterations should be at least 1" << std::endl; return 1; }
    const auto number_of_clusters = options.value< comma::uint32 >( "--number-of-clusters,--clusters" );
    if( number_of_clusters == 0 ) { std::cerr << "math-k-means: got --number-of-clusters=0, --number-of-clusters should be at least 1" << std::endl; return 1; }
    const auto number_of_runs = options.value< unsigned int >( "--number-of-runs,--runs", 10 );
    if( number_of_runs == 0 ) { std::cerr << "math-k-means: got --number-of-runs=0, --number-of-runs should be at least 1" << std::endl; return 1; }
    size = options.optional< unsigned int >( "--size" );
    const auto tolerance = options.value< double >( "--tolerance", 1e-4 );
    if( tolerance <= 0 ) { std::cerr << "math-k-means: got --tolerance=" << tolerance << ", --tolerance should be greater than 0" << std::endl; return 1; }
    k_means operation{ tolerance, max_iterations, number_of_runs, number_of_clusters };
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
            for( const auto & field : fields )
            {
                if( field.substr( 0, 5 ) == "data[" && *field.rbegin() == ']' ) { unsigned int k = boost::lexical_cast< unsigned int >( field.substr( 5, field.size() - 6 ) ) + 1; if( k > max ) { max = k; } }
            }
            if( max == 0 ) { std::cerr << "math-k-means: please specify valid data fields" << std::endl; return 1; }
            size = max;
        }
    }
    comma::csv::input_stream< input_t > istream( std::cin, csv );
    std::deque< std::pair< input_t, std::string > > inputs;
    if( !first.empty() ) { inputs.emplace_back( comma::csv::ascii< snark::k_means::input_t >( csv ).get( first ), first ); }
    while( istream.ready() || std::cin.good() )
    {
        const snark::k_means::input_t* p = istream.read();
        if( !inputs.empty() && p && inputs.front().first.block != p->block )
        {
            operation.run( inputs );
            inputs.clear();
        }
        if( !p ) { break; }
        inputs.emplace_back( *p, istream.last() );
    }
    return inputs.empty() ? 0 : operation.run( inputs );
}

} } // namespace snark { namespace k_means {

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--input-fields" ) ) { std::cout << "data" << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cout << "data,centroid/data,centroid/id" << std::endl; return 0; }
        verbose = options.exists( "--verbose,-v" );
        auto max_threads = options.value< int >( "--max-threads,--threads", -1 );
        if( max_threads < 0 ) { max_threads = std::thread::hardware_concurrency(); }
        // limit maximum available threads to application, let tbb control number of threads to use
        tbb::global_control gc( tbb::global_control::max_allowed_parallelism, max_threads );
        return snark::k_means::run( options );
    }
    catch( std::exception& ex ) { std::cerr << "math-k-means: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "math-k-means: unknown exception" << std::endl; }
    return 1;
}
