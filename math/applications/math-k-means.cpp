// Copyright (c) 2020 Kent Hu

/// @author kent hu

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <deque>
#include <functional>
#include <iostream>
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
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/string/split.h>
#include <tbb/blocked_range.h>

#define TBB_PREVIEW_GLOBAL_CONTROL 1 // required to #include <tbb/global_control.h>

#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include "../../visiting/traits.h"

#ifdef SNARK_USE_CUDA

#include "math-k-means/device.h"
#include "math-k-means/math_k_means.h"

#endif

void usage( const bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "run k-means on input data" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat sample.csv | math-k-means [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields" << std::endl;
    std::cerr << "    block: block number; output k-means centroid id for each" << std::endl;
    std::cerr << "           contiguous block of samples with the same block id" << std::endl;
    std::cerr << "    data: vector of size <size>" << std::endl;
    std::cerr << "    default: data" << std::endl;
    std::cerr << "output" << std::endl;
    std::cerr << "    appended fields: centroid/id,centroid/data" << std::endl;
    std::cerr << "    centroids only: if block field present: block,centroid/id,centroid/data" << std::endl;
    std::cerr << "                    if no block field present: centroid/id,centroid/data" << std::endl;
    std::cerr << "    binary format: 64-bit floating-point for centroid/data, 32-bit unsigned integer for centroid/id" << std::endl;
    std::cerr << "options" << std::endl;
#ifdef SNARK_USE_CUDA
    std::cerr << "    --cuda: use gpu; ATTENTION: only 32-bit float precision currently implemented; if you need 64-bit float precision, run without --cuda" << std::endl;
    std::cerr << "    --cuda-use-pitched,--use-pitched,--pitched: use pitched memory for 2d array (padded 2d array for memory coalescing; faster but uses more memory)" << std::endl;
#endif
    std::cerr << "    --help,-h: show this help; --help --verbose: more help" << std::endl;
    std::cerr << "    --ignore-tolerance: ignore tolerance value used for early exit" << std::endl;
    std::cerr << "    --max-iterations,--iterations=<n>: number of iterations for Lloyd's algorithm; default: 300" << std::endl;
    std::cerr << "    --max-threads,--threads=<n>: maximum number of threads to run, if 0, set to number of cores in system (" << std::thread::hardware_concurrency() << ')' << "; default: 0" << std::endl;
    std::cerr << "    --number-of-clusters,--clusters=<n>: number of k-means cluster per block/all" << std::endl;
    std::cerr << "    --number-of-runs,--runs=<n>: number of times to run k-means, best run is one with lowest inertia (sum of distance of cluster points to cluster centroid); default: 10" << std::endl;
    std::cerr << "    --output-centroids,--centroids: output centroids only" << std::endl;
    std::cerr << "    --size=[<n>]: a hint of number of elements in the data vector; ignored, if data indices specified, e.g. data[0],data[1],data[2]" << std::endl;
    std::cerr << "    --tolerance=<distance>: difference between two consecutive iteration centroid l2 norms to declare convergence and stop iterating in LLoyd's algorithm; default: 1.0e-4" << std::endl;
    if( verbose ) { std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    cat sample.csv | math-k-means --number-of-clusters=3" << std::endl;
    std::cerr << "    cat sample.csv | math-k-means --number-of-clusters=3 --fields=data --size=3" << std::endl;
    std::cerr << "    cat sample.csv | math-k-means --number-of-clusters=3 --fields=,,,data[0],,,,data[1],,data[2]" << std::endl;
    std::cerr << "    cat sample.bin | math-k-means --number-of-clusters=3 --fields=block,data --size=3 --binary=ui,3d" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static comma::csv::options csv;
static boost::optional< unsigned int > size = boost::none;
static bool output_centroids = false;
static bool use_block = false;
#ifdef SNARK_USE_CUDA
static bool use_pitched = false;
#endif
static bool verbose = false;

namespace snark {

#ifdef SNARK_USE_CUDA
namespace cuda { namespace k_means {

struct input_t
{
    std::vector< float > data;
    comma::uint32 block = 0;

    input_t() : data( *size ), block( 0 ) {}
};

class k_means
{
public:
    k_means( float tolerance, unsigned int max_iterations, unsigned int number_of_runs,
             comma::uint32 number_of_clusters, unsigned int size ) noexcept : tolerance_( tolerance ),
                                                                              max_iterations_( max_iterations ),
                                                                              number_of_runs_( number_of_runs ),
                                                                              number_of_clusters_( number_of_clusters ),
                                                                              ncols_( size ) {}

    template < template < typename > class Matrix >
    std::tuple< std::vector< float >, std::vector< comma::uint32 > >
    run_on_block( const std::vector< float >& h_matrix ) const
    {
        const size_t nrows = h_matrix.size() / ncols_;

        // device arrays/matrices
        Matrix< float > d_matrix( nrows, ncols_, h_matrix );
        Matrix< float > d_means( number_of_clusters_, ncols_ );
        Matrix< float > d_sums( number_of_clusters_, ncols_ );
        CUDA_CHECK_ERRORS( d_sums.fill( 0.0f ) )

        device::array< comma::uint32 > d_assignments( nrows );
        device::array< unsigned int > d_counts( number_of_clusters_ );
        CUDA_CHECK_ERRORS( d_counts.fill( 0 ) )

        // host vectors
        std::vector< float > h_means( number_of_clusters_* ncols_ );
        std::vector< comma::uint32 > h_assignments( nrows );

        // save runs to choose best centroid
        std::vector< float > all_scores;
        all_scores.reserve( number_of_runs_ );
        std::vector< decltype( h_means ) > all_centroids;
        all_centroids.reserve( number_of_runs_ );
        std::vector< decltype( h_assignments ) > all_centroid_assignments;
        all_centroid_assignments.reserve( number_of_runs_ );

        cudaDeviceProp prop{};
        CUDA_CHECK_ERRORS( cudaGetDeviceProperties( &prop, 0 ) )
        for( unsigned int run = 0; run < number_of_runs_; ++run )
        {
            CUDA_CHECK_ERRORS( d_means.to_device( initialize_centroids( h_matrix ) ) )
            float difference = std::numeric_limits< float >::max();
            for( unsigned int iteration = 0; iteration < max_iterations_; ++iteration )
            {
                assign_centroids( d_matrix, d_means, d_assignments, d_sums, d_counts, number_of_clusters_, prop.maxThreadsPerBlock );
                update_centroids( d_means, d_sums, d_counts, number_of_clusters_ );
                if( tolerance_ < 0 ) { continue; }
                std::vector< float > new_means( number_of_clusters_* ncols_ ); // temp buffer for calculating tolerance difference
                CUDA_CHECK_ERRORS( d_means.to_host( new_means ) )
                difference = tbb::parallel_reduce( tbb::blocked_range< comma::uint32 >( 0, number_of_clusters_ ), 0.0f,
                                                   [&]( const tbb::blocked_range< comma::uint32 > chunk, float difference ) -> float
                                                   {
                                                       for( comma::uint32 i = chunk.begin(); i < chunk.end(); ++i )
                                                       {
                                                           difference += squared_euclidean_distance( ncols_, &h_means[i * ncols_], &new_means[i * ncols_] );
                                                       }
                                                       return difference;
                                                   },
                                                   std::plus< float >()
                );
                h_means = std::move( new_means );
                if( difference < tolerance_ * tolerance_ ) { break; }
            }
            CUDA_CHECK_ERRORS( d_assignments.to_host( h_assignments ) )
            if( tolerance_ < 0 || difference > tolerance_ * tolerance_ ) { CUDA_CHECK_ERRORS( d_means.to_host( h_means ) ) }
            const float score = tbb::parallel_reduce( tbb::blocked_range< size_t >( 0, nrows ), 0.0f,
                                                      [&]( const tbb::blocked_range< size_t > chunk, float score ) -> float
                                                      {
                                                          for( size_t point = chunk.begin(); point < chunk.end(); ++point )
                                                          {
                                                              const auto cluster_assignment = h_assignments[point];
                                                              score += std::sqrt( squared_euclidean_distance( ncols_, &h_matrix[point * ncols_], &h_means[cluster_assignment * ncols_] ) );
                                                          }
                                                          return score;
                                                      },
                                                      std::plus< float >()
            );
            all_scores.emplace_back( score );
            all_centroids.emplace_back( h_means );
            all_centroid_assignments.emplace_back( h_assignments );
        }
        const auto& best_index = std::distance( all_scores.begin(), std::min_element( all_scores.begin(), all_scores.end() ) );
        return std::make_tuple( all_centroids[best_index], all_centroid_assignments[best_index] );
    }

private:
    std::vector< float > initialize_centroids( const std::vector< float >& h_matrix ) const
    {
        static std::mt19937 generator( std::random_device{}() );
        static std::vector< float > init_centroids( number_of_clusters_* ncols_ );
        static std::uniform_int_distribution< size_t > indices( 0, h_matrix.size() / ncols_ - 1 );
        for( size_t cluster = 0; cluster < number_of_clusters_; ++cluster )
        {
            for( size_t col = 0; col < ncols_; ++col )
            {
                init_centroids[cluster * ncols_ + col] = h_matrix[indices( generator ) * ncols_ + col];
            }
        }
        return init_centroids;
    }

    const float tolerance_;
    const unsigned int max_iterations_;
    const unsigned int number_of_runs_;
    const unsigned int number_of_clusters_;
    const unsigned int ncols_;
};

} } // namespace cuda { namespace k_means {

#endif

namespace k_means {

static double square( const double value ) { return value * value; }

static double squared_euclidean_distance( const std::vector< double >& first, const std::vector< double >& second )
{
    double ret = 0.0;
    for( size_t i = 0; i < first.size(); ++i ) { ret += square( first[i] - second[i] ); }
    return ret;
}

struct input_t
{
    std::vector< double > data;
    comma::uint32 block;

    input_t() : data( *size ), block( 0 ) {}
};

struct k_means
{
    const double tolerance;
    const unsigned int max_iterations;
    const unsigned int number_of_runs;
    const comma::uint32 number_of_clusters;

    k_means( double tolerance, unsigned int max_iterations, unsigned int number_of_runs, comma::uint32 number_of_clusters ) noexcept :
            tolerance( tolerance ),
            max_iterations( max_iterations ),
            number_of_runs( number_of_runs ),
            number_of_clusters( number_of_clusters ) {}

    std::vector< std::vector< double > > initialize_centroids( const std::deque< std::vector< double > >& dataframe ) const
    {
        static std::mt19937 generator( std::random_device{}() );
        std::vector< std::vector< double > > centroids( number_of_clusters );
        std::uniform_int_distribution< size_t > indices( 0, dataframe.size() - 1 );
        std::generate( centroids.begin(), centroids.end(), [&]() { return dataframe[indices( generator )]; } );
        return centroids;
    }

    std::vector< comma::uint32 >
    assign_centroids( const std::vector< std::vector< double > >& centroids, const std::deque< std::vector< double > >& dataframe ) const
    {
        std::vector< comma::uint32 > centroid_assignments( dataframe.size() );
        tbb::parallel_for( tbb::blocked_range< size_t >( 0, centroid_assignments.size() ), [&]( const tbb::blocked_range< size_t >& chunk )
        {
            for( size_t point = chunk.begin(); point < chunk.end(); ++point )
            {
                auto best_distance = std::numeric_limits< double >::max();
                comma::uint32 best_centroid = 0;
                for( comma::uint32 centroid_i = 0; centroid_i < number_of_clusters; ++centroid_i )
                {
                    const double distance = squared_euclidean_distance( dataframe[point], centroids[centroid_i] );
                    if( distance < best_distance ) { best_distance = distance; best_centroid = centroid_i; }
                }
                centroid_assignments[point] = best_centroid;
            }
        } );
        return centroid_assignments;
    }

    std::vector< std::vector< double > >
    update_centroids( const std::vector< comma::uint32 >& centroid_assignments, const std::deque< std::vector< double > >& dataframe ) const
    {
        // for each cluster, calculate sum of data vectors and number of data vectors
        std::vector< size_t > size_of_centroid( number_of_clusters );
        std::vector< std::vector< double > > sum_of_points_in_centroid( number_of_clusters, std::vector< double >( *size, 0.0 ) );
        for( size_t point = 0; point < dataframe.size(); ++point )
        {
            const auto centroid_i = centroid_assignments[point];
            for( size_t point_i = 0;
                 point_i < dataframe[point].size(); ++point_i ) { sum_of_points_in_centroid[centroid_i][point_i] += dataframe[point][point_i]; }
            ++size_of_centroid[centroid_i];
        }
        // calculate new centroid means
        std::vector< std::vector< double > > new_centroids( number_of_clusters );
        tbb::parallel_for( tbb::blocked_range< comma::uint32 >( 0, number_of_clusters ), [&]( const tbb::blocked_range< comma::uint32 >& chunk )
        {
            for( comma::uint32 centroid_i = chunk.begin(); centroid_i < chunk.end(); ++centroid_i )
            {
                new_centroids[centroid_i].reserve( *size );
                const auto centroid_size = std::max< size_t >( 1, size_of_centroid[centroid_i] ); // minimum size is at least 1
                for( size_t point_i = 0; point_i < sum_of_points_in_centroid[centroid_i].size(); ++point_i )
                {
                    new_centroids[centroid_i].emplace_back( sum_of_points_in_centroid[centroid_i][point_i] / centroid_size );
                }
            }
        } );
        return new_centroids;
    }

    // refer to http://www.goldsborough.me/c++/python/cuda/2017/09/10/20-32-46-exploring_k-means_in_python,_c++_and_cuda/
    std::tuple< std::vector< std::vector< double > >, std::vector< comma::uint32 > > run_on_block( const std::deque< std::vector< double > >& dataframe ) const
    {
        std::vector< double > all_scores( number_of_runs );
        std::vector< std::vector< std::vector< double > > > all_centroids( number_of_runs );
        std::vector< std::vector< comma::uint32 > > all_centroid_assignments( number_of_runs );
        tbb::parallel_for( tbb::blocked_range< unsigned int >( 0, number_of_runs ), [&]( const tbb::blocked_range< unsigned int >& chunk )
        {
            for( unsigned int run = chunk.begin(); run < chunk.end(); ++run )
            {
                std::vector< comma::uint32 > centroid_assignments;
                std::vector< std::vector< double > > run_centroids = initialize_centroids( dataframe );
                for( unsigned int iteration = 0; iteration < max_iterations; ++iteration )
                {
                    centroid_assignments = assign_centroids( run_centroids, dataframe );
                    std::vector< std::vector< double > > new_centroids = update_centroids( centroid_assignments, dataframe );
                    if( tolerance < 0 ) { run_centroids = std::move( new_centroids ); continue; }
                    double summed_difference = tbb::parallel_reduce( tbb::blocked_range< comma::uint32 >( 0, number_of_clusters ), 0.0,
                            [&]( const tbb::blocked_range< comma::uint32 > chunk, double difference ) -> double
                            {
                                for( comma::uint32 centroid_i = chunk.begin(); centroid_i < chunk.end(); ++centroid_i )
                                {
                                    difference += squared_euclidean_distance( run_centroids[centroid_i], new_centroids[centroid_i] );
                                }
                                return difference;
                            },
                            std::plus< double >()
                    );
                    run_centroids = std::move( new_centroids );
                    if( summed_difference < tolerance * tolerance ) { break; }
                }
                double run_score = tbb::parallel_reduce( tbb::blocked_range< size_t >( 0, dataframe.size() ), 0.0,
                                                         [&]( const tbb::blocked_range< size_t > chunk, double score ) -> double
                                                         {
                                                             for( size_t point = chunk.begin(); point < chunk.end(); ++point )
                                                             {
                                                                 score += std::sqrt( squared_euclidean_distance( dataframe[point], run_centroids[centroid_assignments[point]] ) );
                                                             }
                                                             return score;
                                                         },
                                                         std::plus< double >()
                );
                all_scores[run] = run_score;
                all_centroids[run] = std::move( run_centroids );
                all_centroid_assignments[run] = std::move( centroid_assignments );
            }
        } );
        const std::vector< double >::iterator& min_element = std::min_element( all_scores.begin(), all_scores.end() );
        const auto best_index = std::distance( all_scores.begin(), min_element );
        return std::make_pair( all_centroids[best_index], all_centroid_assignments[best_index] );
    }
};

} // namespace k_means {

} // namespace snark {

static std::string get_size()
{
    std::string first;
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
        for( const auto& field : fields )
        {
            if( field.substr( 0, 5 ) == "data[" && *field.rbegin() == ']' )
            {
                const auto k = boost::lexical_cast< unsigned int >( field.substr( 5, field.size() - 6 ) ) + 1;
                if( k > max ) { max = k; }
            }
        }
        if( max == 0 ) { COMMA_THROW( comma::exception, "please specify valid data fields" ) }
        size = max;
    }
    return first;
}

#ifdef SNARK_USE_CUDA

static int run_cuda_( const float tolerance, const unsigned int max_iterations, const unsigned int number_of_runs, const comma::uint32 number_of_clusters )
{
    using namespace snark::cuda::k_means;
    comma::uint32 block = 0;
    std::vector< float > dataframe;
    std::vector< std::string > input_lines;
    auto set_input_values_ = [&]( const input_t& p, const std::string& line )
    {
        block = p.block;
        input_lines.emplace_back( line );
        dataframe.insert( end( dataframe ), begin( p.data ), end( p.data ) );
    };
    if( !size )
    {
        const auto& first_line = get_size();
        set_input_values_( comma::csv::ascii< input_t >( csv ).get( first_line ), first_line );
    }
    auto write_centroids_only_ = [number_of_clusters]( const std::vector< float >& centroids, const comma::uint32 block )
    {
        for( comma::uint32 i = 0; i < number_of_clusters; ++i )
        {
            if( csv.binary() )
            {
                if( use_block ) { std::cout.write( reinterpret_cast< const char* >( &block ), sizeof( block ) ); }
                std::cout.write( reinterpret_cast< const char* >( &i ), sizeof( i ) );
                for( size_t col = 0; col < *size; ++col )
                {
                    const auto point = static_cast< double >( centroids[i * *size + col] );
                    std::cout.write( reinterpret_cast< const char* >( &point ), sizeof( point ) );
                }
            }
            else
            {
                if( use_block ) { std::cout << block << csv.delimiter; }
                std::cout << i;
                for( size_t col = 0; col < *size; ++col ) { std::cout << csv.delimiter << static_cast< double >( centroids[i * *size + col] ); }
                std::cout << std::endl;
            }
        }
    };
    auto write_lines_ = []( const std::vector< std::string >& input_lines, const std::vector< float >& centroids,
                            const std::vector< comma::uint32 >& centroid_assignments )
    {
        for( size_t i = 0; i < centroid_assignments.size(); ++i )
        {
            const comma::uint32 centroid_assignment = centroid_assignments[i];
            std::cout << input_lines[i];
            if( csv.binary() )
            {
                std::cout.write( reinterpret_cast< const char* >( &centroid_assignment ), sizeof( centroid_assignment ) );
                for( size_t col = 0; col < *size; ++col )
                {
                    const auto point = static_cast< double >( centroids[centroid_assignment * *size + col] );
                    std::cout.write( reinterpret_cast< const char* >( &point ), sizeof( point ) );
                }
            }
            else
            {
                std::cout << csv.delimiter << centroid_assignment;
                for( size_t col = 0; col < *size; ++col ) { std::cout << csv.delimiter << static_cast< double >( centroids[centroid_assignment * *size + col] ); }
                std::cout << std::endl;
            }
        }
    };
    std::vector< float > centroids;
    std::vector< comma::uint32 > centroid_assignments;
    k_means operation{ tolerance, max_iterations, number_of_runs, number_of_clusters, *size };
    comma::csv::input_stream< input_t > istream( std::cin, csv );
    while( istream.ready() || std::cin.good() )
    {
        const input_t* p = istream.read();
        if( !dataframe.empty() && ( !p || block != p->block ) )
        {
            std::tie( centroids, centroid_assignments ) = use_pitched ? operation.run_on_block< snark::cuda::device::pitched_matrix >( dataframe ) : operation.run_on_block< snark::cuda::device::matrix >( dataframe );
            output_centroids ? write_centroids_only_( centroids, block ) : write_lines_( input_lines, centroids, centroid_assignments );
            if( csv.flush ) { std::cout.flush(); }
            dataframe.clear();
            input_lines.clear();
        }
        if( !p ) { break; }
        set_input_values_( *p, istream.last() );
    }
    return 0;
}

#endif

static int run_( const double tolerance, const unsigned int max_iterations, const unsigned int number_of_runs, const comma::uint32 number_of_clusters )
{
    using namespace snark::k_means;
    std::deque< std::vector< double > > dataframe;
    std::deque< std::string > input_lines;
    comma::uint32 block = 0;
    auto set_input_values_ = [&]( const input_t& p, const std::string& line )
    {
        block = p.block;
        input_lines.emplace_back( line );
        dataframe.emplace_back( p.data );
    };
    if( !size )
    {
        const auto& first_line = get_size();
        set_input_values_( comma::csv::ascii< input_t >( csv ).get( first_line ), first_line );
    }
    auto write_centroids_only_ = [number_of_clusters]( const std::vector< std::vector< double > >& centroids, const comma::uint32 block )
    {
        for( comma::uint32 i = 0; i < number_of_clusters; ++i )
        {
            if( csv.binary() )
            {
                if( use_block ) { std::cout.write( reinterpret_cast< const char* >( &block ), sizeof( block ) ); }
                std::cout.write( reinterpret_cast< const char* >( &i ), sizeof( i ) );
                for( const auto point : centroids[i] ) { std::cout.write( reinterpret_cast< const char* >( &point ), sizeof( point ) ); }
            }
            else
            {
                if( use_block ) { std::cout << block << csv.delimiter; }
                std::cout << i;
                for( const auto point : centroids[i] ) { std::cout << csv.delimiter << point; }
                std::cout << std::endl;
            }
        }
    };
    auto write_lines_ = []( const std::deque< std::string >& input_lines,
                            const std::vector< std::vector< double > >& centroids,
                            const std::vector< comma::uint32 >& centroid_assignments )
    {
        for( size_t i = 0; i < centroid_assignments.size(); ++i )
        {
            std::cout << input_lines[i];
            const auto centroid_assignment = centroid_assignments[i];
            const std::vector< double >& centroid = centroids[centroid_assignment];
            if( csv.binary() )
            {
                std::cout.write( reinterpret_cast< const char* >( &centroid_assignment ), sizeof( centroid_assignment ) );
                for( const auto point : centroid ) { std::cout.write( reinterpret_cast< const char* >( &point ), sizeof( point ) ); }
            }
            else
            {
                std::cout << csv.delimiter << centroid_assignment;
                for( const auto point : centroid ) { std::cout << csv.delimiter << point; }
                std::cout << std::endl;
            }
        }
    };
    std::vector< std::vector< double > > centroids;
    std::vector< comma::uint32 > centroid_assignments;
    k_means operation{ tolerance, max_iterations, number_of_runs, number_of_clusters };
    comma::csv::input_stream< input_t > istream( std::cin, csv );
    while( istream.ready() || std::cin.good() )
    {
        const input_t* p = istream.read();
        if( !dataframe.empty() && ( !p || block != p->block ) )
        {
            std::tie( centroids, centroid_assignments ) = operation.run_on_block( dataframe );
            if( output_centroids ) { write_centroids_only_( centroids, block ); }
            else { write_lines_( input_lines, centroids, centroid_assignments ); }
            if( csv.flush ) { std::cout.flush(); }
            dataframe.clear();
            input_lines.clear();
        }
        if( !p ) { break; }
        set_input_values_( *p, istream.last() );
    }
    return 0;
}

static int run( const comma::command_line_options& options )
{
    output_centroids = options.exists( "--output-centroids,--centroids" );
    csv = comma::csv::options( options, "data" );
    use_block = csv.has_field( "block" );
    std::cout.precision( csv.precision );
    const auto max_iterations = options.value< unsigned int >( "--max-iterations,--iterations", 300 );
    if( max_iterations == 0 )
    {
        std::cerr << "math-k-means: got --max-iterations=0, --max-iterations should be at least 1" << std::endl;
        return 1;
    }
    const auto number_of_clusters = options.value< comma::uint32 >( "--number-of-clusters,--clusters" );
    if( number_of_clusters == 0 )
    {
        std::cerr << "math-k-means: got --number-of-clusters=0, --number-of-clusters should be at least 1" << std::endl;
        return 1;
    }
    const auto number_of_runs = options.value< unsigned int >( "--number-of-runs,--runs", 10 );
    if( number_of_runs == 0 )
    {
        std::cerr << "math-k-means: got --number-of-runs=0, --number-of-runs should be at least 1" << std::endl;
        return 1;
    }
    size = options.optional< unsigned int >( "--size" );
    auto tolerance = options.value< double >( "--tolerance", 1.0e-4 );
    if( tolerance <= 0 )
    {
        std::cerr << "math-k-means: got --tolerance=" << tolerance << ", --tolerance should be greater than 0" << std::endl;
        return 1;
    }
    if( options.exists( "--ignore-tolerance" ) ) { tolerance = -1.0; }
#ifdef SNARK_USE_CUDA
    use_pitched = options.exists( "--cuda-use-pitched,--use-pitched,--pitched" );
    return options.exists( "--cuda" ) ? run_cuda_( static_cast< float >( tolerance ), max_iterations, number_of_runs, number_of_clusters ) : run_( tolerance, max_iterations, number_of_runs, number_of_clusters );
#else
    if( options.exists( "--cuda" ) ) { std::cerr << "math-k-means: given --cuda, but built without cuda support; run anyway, but may be slower than you expect" << std::endl; }
    return run_( tolerance, max_iterations, number_of_runs, number_of_clusters );
#endif
}

namespace comma { namespace visiting {

template <>
struct traits< snark::k_means::input_t >
{
    template < typename K, typename V >
    static void visit( const K&, snark::k_means::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }

    template < typename K, typename V >
    static void visit( const K&, const snark::k_means::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }
};

#ifdef SNARK_USE_CUDA
template <>
struct traits< snark::cuda::k_means::input_t >
{
    template < typename K, typename V >
    static void visit( const K&, snark::cuda::k_means::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }

    template < typename K, typename V >
    static void visit( const K&, const snark::cuda::k_means::input_t& p, V& v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }
};
#endif

} } // namespace comma { namespace visiting {

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--input-fields" ) )
        {
            std::cout << "data" << std::endl;
            return 0;
        }
        if( options.exists( "--output-fields" ) )
        {
            std::cout << "data,centroid/id,centroid/data" << std::endl;
            return 0;
        }
        verbose = options.exists( "--verbose,-v" );
        auto max_threads = options.value< unsigned int >( "--max-threads,--threads", 0 );
        if( max_threads == 0 ) { max_threads = std::thread::hardware_concurrency(); }
        tbb::global_control gc( tbb::global_control::max_allowed_parallelism, max_threads );
        return run( options );
    }
    catch( std::exception& ex ) { std::cerr << "math-k-means: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "math-k-means: unknown exception" << std::endl; }
    return 1;
}
