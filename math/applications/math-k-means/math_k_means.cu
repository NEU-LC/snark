#include <algorithm>
#include <cfloat>
#include <cmath>
#include <exception>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

__device__ float squared_euclidean_distance( size_t size, const float *vector_a, const float *vector_b )
{
    float squared_euclidean_distance = 0;
    for( size_t i = 0; i < size; ++i )
    {
        squared_euclidean_distance += ( ( vector_a[i] - vector_b[i] ) * ( vector_a[i] - vector_b[i] ) );
    }
    return squared_euclidean_distance;
}

__global__ void
assign_clusters( size_t nrows, size_t ncols, size_t dataframe_pitch, const float *__restrict__ dataframe,
                 unsigned int *__restrict__ assignments, const unsigned int k, size_t means_pitch,
                 const float *__restrict__ means, size_t new_sums_pitch, float *__restrict__ new_sums,
                 unsigned int *__restrict__ counts )
{
    const unsigned int y = blockIdx.x * blockDim.x + threadIdx.x;
    if( y >= nrows ) { return; }

    // Make global loads once.
    auto dataframe_row = reinterpret_cast< const float * >( reinterpret_cast< const char * >( dataframe ) +
                                                            y * dataframe_pitch );

    unsigned int best_cluster = 0;
    float best_distance = FLT_MAX;
    for( unsigned int cluster = 0; cluster < k; ++cluster )
    {
        auto means_row = reinterpret_cast< const float * >( reinterpret_cast< const char * >( means ) +
                                                            cluster * means_pitch );
        const float distance = squared_euclidean_distance( ncols, dataframe_row, means_row );
        if( distance < best_distance )
        {
            best_distance = distance;
            best_cluster = cluster;
        }
    }
    unsigned int *assignments_row = assignments + y;
    *assignments_row = best_cluster;
    auto new_sums_row = reinterpret_cast< float * >( reinterpret_cast< char * >( new_sums ) +
                                                     best_cluster * new_sums_pitch );
    for( unsigned int i = 0; i < ncols; ++i ) { atomicAdd( &new_sums_row[i], dataframe_row[i] ); }
    atomicAdd( &counts[best_cluster], 1 );
}

__global__ void
compute_new_means_and_reset( size_t ncols, size_t means_pitch, float *__restrict__ means, size_t new_sums_pitch,
                             float *__restrict__ new_sums, unsigned int *__restrict__ counts )
{
    const unsigned int cluster = threadIdx.x;
    const auto count = static_cast< float >( max( 1u, counts[cluster] ));
    auto means_row = reinterpret_cast< float * >( reinterpret_cast< char * >( means ) + cluster * means_pitch );
    auto new_sums_row = reinterpret_cast< float * >( reinterpret_cast< char * >( new_sums ) +
                                                     cluster * new_sums_pitch );
    for( size_t i = 0; i < ncols; ++i )
    {
        means_row[i] = new_sums_row[i] / count;
        new_sums_row[i] = 0;
    }
    counts[cluster] = 0;
}

namespace snark { namespace cuda { namespace k_means {

static boost::optional< size_t > ncols;
static comma::csv::options csv;
static bool verbose;

template < typename FloatingPoint >
struct input_t
{
    std::vector< FloatingPoint > data;
    unsigned int block = 0;

    input_t() : data( *ncols ) {};
};

namespace device {

void check_errors()
{
    static cudaError_t err;
    err = cudaGetLastError();
    if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: error: " << cudaGetErrorString( err ) ) }
}

void check_async_errors()
{
    static cudaError_t err;
    err = cudaDeviceSynchronize();
    if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: error: " << cudaGetErrorString( err ) ) }
}

template < typename Numeric >
struct array
{
    array() = delete;

    explicit array( size_t size ) : size( size )
    {
        cudaError_t err = cudaMalloc( &data, bytes() );
        if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: error: " << cudaGetErrorString( err ) ) }
    }

    array( size_t size, const std::vector< Numeric > &h_array ) : size( size )
    {
        cudaError_t err = cudaMalloc( &data, bytes() );
        if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: error: " << cudaGetErrorString( err ) ) }
        to_device( h_array );
    }

    array( const array &other ) = delete;

    array &operator=( const array &other ) = delete;

    array( array &&other ) = delete;

    array &operator=( array &&other ) = delete;

    ~array() { cudaFree( data ); }

    size_t bytes() const noexcept { return sizeof( Numeric ) * size; }

    void fill( int value ) noexcept { cudaMemset( data, value, bytes() ); }

    void to_device( const std::vector< Numeric > &h_array ) noexcept
    {
        cudaMemcpy( data, h_array.data(), bytes(), cudaMemcpyHostToDevice );
    }

    void to_host( std::vector< Numeric > &h_array ) const noexcept
    {
        cudaMemcpy( h_array.data(), data, bytes(), cudaMemcpyDeviceToHost );
    }

    Numeric *data = nullptr;
    size_t size;
};

template < typename Numeric >
struct matrix
{
    matrix() = delete;

    matrix( size_t nrows, size_t ncols ) : nrows( nrows ), ncols( ncols )
    {
        cudaError_t err = cudaMallocPitch( &data, &pitch, ncols * sizeof( Numeric ), nrows );
        if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: error: " << cudaGetErrorString( err ) ) }
    }

    matrix( size_t nrows, size_t ncols, const std::vector< Numeric > &h_matrix ) : nrows( nrows ), ncols( ncols )
    {
        cudaError_t err = cudaMallocPitch( &data, &pitch, ncols * sizeof( Numeric ), nrows );
        if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "cuda: error: " << cudaGetErrorString( err ) ) }
        to_device( h_matrix );
    }

    matrix( const matrix &other ) = delete;

    matrix &operator=( const matrix &other ) = delete;

    matrix( matrix &&other ) = delete;

    matrix &operator=( matrix &&other ) = delete;

    ~matrix() { cudaFree( data ); }

    size_t bytes() const noexcept { return sizeof( Numeric ) * ncols * nrows; }

    void fill( int value ) noexcept { cudaMemset2D( data, pitch, value, ncols * sizeof( Numeric ), nrows ); }

    void to_device( const std::vector< Numeric > &h_matrix ) noexcept
    {
        cudaMemcpy2D( data, pitch, h_matrix.data(), ncols * sizeof( Numeric ), ncols * sizeof( Numeric ), nrows,
                      cudaMemcpyHostToDevice );
    }

    void to_host( std::vector< Numeric > &h_matrix ) const noexcept
    {
        cudaMemcpy2D( h_matrix.data(), ncols * sizeof( Numeric ), data, pitch, ncols * sizeof( Numeric ), nrows,
                      cudaMemcpyDeviceToHost );
    }

    Numeric *data = nullptr;
    size_t nrows;
    size_t ncols;
    size_t pitch = 0;
};

} // namespace device {

struct k_means
{
    const double tolerance;
    const unsigned int max_iterations;
    const unsigned int number_of_runs;
    const unsigned int number_of_clusters;

    k_means( double tolerance,
             unsigned int max_iterations,
             unsigned int number_of_runs,
             unsigned int number_of_clusters ) noexcept
            : tolerance( tolerance ), max_iterations( max_iterations ), number_of_runs( number_of_runs ),
              number_of_clusters( number_of_clusters ) {}

    std::vector< float > initialize_centroids( const std::vector< float > &h_matrix ) const
    {
        static std::mt19937 generator( std::random_device{}() );
        static std::vector< float > init_centroids( number_of_clusters * *ncols );
        static std::uniform_int_distribution< size_t > indices( 0, h_matrix.size() / *ncols - 1 );
        for( size_t cluster = 0; cluster < number_of_clusters; ++cluster )
        {
            for( size_t col = 0; col < *ncols; ++col )
            {
                init_centroids[cluster * *ncols + col] = h_matrix[indices( generator ) * *ncols + col];
            }
        }
        return init_centroids;
    }

    void run( const std::vector< float > &h_matrix, const std::vector< std::string > &input_lines ) const
    {
        const size_t nrows = h_matrix.size() / *ncols;

        // device arrays/matrices
        device::matrix< float > d_dataframe( nrows, *ncols, h_matrix );
        device::matrix< float > d_means( number_of_clusters, *ncols );
        device::matrix< float > d_sums( number_of_clusters, *ncols );
        d_sums.fill( 0 );
        device::array< unsigned int > d_assignments( nrows );
        device::array< unsigned int > d_counts( number_of_clusters );
        d_counts.fill( 0 );

        // host vectors
        std::vector< float > h_means( number_of_clusters * *ncols );
        std::vector< unsigned int > h_assignments( 1 * nrows );

        // save runs to choose best centroid
        std::vector< double > all_scores;
        std::vector< std::vector< float > > all_centroids;
        std::vector< std::vector< unsigned int > > all_centroid_assignments;

        cudaDeviceProp prop{};
        cudaGetDeviceProperties( &prop, 0 );
        device::check_errors();
        if( verbose )
        {
            std::cerr << "math-k-means: cuda: using " << prop.maxThreadsPerBlock << " threads" << std::endl;
        }
        const unsigned int blocks_per_grid = ( nrows + prop.maxThreadsPerBlock - 1 ) / prop.maxThreadsPerBlock;
        for( unsigned int run = 0; run < number_of_runs; ++run )
        {

            d_means.to_device( initialize_centroids( h_matrix ) );
            for( unsigned int iteration = 0; iteration < max_iterations; ++iteration )
            {
                assign_clusters<<<blocks_per_grid, prop.maxThreadsPerBlock>>>(
                        d_dataframe.nrows,
                        d_dataframe.ncols,
                        d_dataframe.pitch,
                        d_dataframe.data,
                        d_assignments.data,
                        number_of_clusters,
                        d_means.pitch,
                        d_means.data,
                        d_sums.pitch,
                        d_sums.data,
                        d_counts.data
                );
                device::check_errors();
                device::check_async_errors();
                compute_new_means_and_reset<<<1, number_of_clusters>>>(
                        *ncols,
                        d_means.pitch,
                        d_means.data,
                        d_sums.pitch,
                        d_sums.data,
                        d_counts.data
                );
                device::check_errors();
                device::check_async_errors();
            }
            d_assignments.to_host( h_assignments );
            d_means.to_host( h_means );
            const double score = tbb::parallel_reduce( tbb::blocked_range< size_t >( 0, nrows ), 0.0,
                                                    [&]( const tbb::blocked_range< size_t > chunk,
                                                         double score ) -> double
                                                    {
                                                        for( size_t point = chunk.begin();
                                                             point < chunk.end(); ++point )
                                                        {
                                                            const auto cluster_assignment = h_assignments[point];
                                                            double temp = 0.0;
                                                            for( size_t x = 0; x < *ncols; ++x )
                                                            {
                                                                temp += ( h_matrix[point * *ncols + x] -
                                                                          h_means[cluster_assignment * *ncols + x] ) *
                                                                        ( h_matrix[point * *ncols + x] -
                                                                          h_means[cluster_assignment * *ncols + x] );
                                                            }
                                                            score += std::sqrt( temp );
                                                        }
                                                        return score;
                                                    },
                                                    std::plus< double >()
            );
            all_scores.push_back( score );
            all_centroids.push_back( h_means );
            all_centroid_assignments.push_back( h_assignments );
        }
        const auto &best_index = std::distance( all_scores.begin(),
                                                std::min_element( all_scores.begin(), all_scores.end() ) );
        if( verbose )
        {
            std::cerr << "math-k-means: all scores" << std::endl;
            std::cerr << "math-k-means:";
            for( const auto score : all_scores ) { std::cerr << ' ' << score; }
            std::cerr << std::endl;
            std::cerr << "math-k-means: run " << best_index << " has best score of " << all_scores[best_index]
                      << std::endl;
        }
        h_means = std::move( all_centroids[best_index] );
        h_assignments = std::move( all_centroid_assignments[best_index] );
        for( size_t row = 0; row < nrows; ++row )
        {
            const comma::uint32 cluster_assignment = h_assignments[row];
            std::cout << input_lines[row];
            if( csv.binary() )
            {
                for( size_t col = 0; col < *ncols; ++col )
                {
                    const auto point = static_cast< double >( h_means[cluster_assignment * *ncols + col] );
                    std::cout.write( reinterpret_cast< const char * >( &point ), sizeof( point ) );
                }
                std::cout.write( reinterpret_cast< const char * >( &cluster_assignment ),
                                 sizeof( cluster_assignment ) );
            }
            else
            {
                for( size_t col = 0; col < *ncols; ++col )
                {
                    const auto point = static_cast< double >( h_means[cluster_assignment * *ncols + col] );
                    std::cout << csv.delimiter << point;
                }
                std::cout << csv.delimiter << cluster_assignment << std::endl;
            }
            if( csv.flush ) { std::cout.flush(); }
        }
    }
};

int run( const comma::command_line_options &options )
{
    verbose = options.exists( "--verbose,-v" );
    if( verbose ) { std::cerr << "math-k-means: running cuda version" << std::endl; }
    csv = comma::csv::options( options, "data" );
    std::cout.precision( csv.precision );
    const auto max_iterations = options.value< unsigned int >( "--max-iterations,--iterations", 300 );
    if( max_iterations == 0 )
    {
        std::cerr << "math-k-means: got --max-iterations=0, --max-iterations should be at least 1" << std::endl;
        return 1;
    }
    const auto number_of_clusters = options.value< unsigned int >( "--number-of-clusters,--clusters" );
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
    ncols = options.optional< unsigned int >( "--size" );
    const auto tolerance = options.value< double >( "--tolerance", 1e-4 );
    if( tolerance <= 0 )
    {
        std::cerr << "math-k-means: got --tolerance=" << tolerance << ", --tolerance should be greater than 0"
                  << std::endl;
        return 1;
    }
    if( options.exists( "--tolerance" ) && verbose )
    {
        std::cerr << "math-k-means: --tolerance specified, ignoring as --tolerance degrades performance" << std::endl;
    }
    std::string first;
    if( !ncols )
    {
        const std::vector< std::string > &fields = comma::split( csv.fields, ',' );
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
            ncols = count - fields.size() + 1;
        }
        else
        {
            unsigned int max = 0;
            for( const auto &field : fields )
            {
                if( field.substr( 0, 5 ) == "data[" && *field.rbegin() == ']' )
                {
                    unsigned int k = boost::lexical_cast< unsigned int >( field.substr( 5, field.size() - 6 ) ) + 1;
                    if( k > max ) { max = k; }
                }
            }
            if( max == 0 )
            {
                std::cerr << "math-k-means: please specify valid data fields" << std::endl;
                return 1;
            }
            ncols = max;
        }
    }
    comma::csv::input_stream< input_t< float > > istream( std::cin, csv );
    std::vector< float > dataframe;
    std::vector< unsigned int > blocks;
    std::vector< std::string > input_lines;
    if( !first.empty() )
    {
        auto p = comma::csv::ascii< input_t< float > >( csv ).get( first );
        dataframe.insert( end( dataframe ), begin( p.data ), end( p.data ) );
        blocks.emplace_back( p.block );
        input_lines.emplace_back( first );
    }
    k_means operation{ tolerance, max_iterations, number_of_runs, number_of_clusters };
    while( istream.ready() || std::cin.good() )
    {
        const snark::cuda::k_means::input_t< float > *p = istream.read();
        if( !blocks.empty() && ( !p || blocks.front() != p->block ) && !dataframe.empty() )
        {
            operation.run( dataframe, input_lines );
            dataframe.clear();
            blocks.clear();
            input_lines.clear();
        }
        if( !p ) { break; }
        dataframe.insert( end( dataframe ), begin( p->data ), end( p->data ) );
        blocks.emplace_back( p->block );
        input_lines.emplace_back( istream.last() );
    }
    return 0;
}
} } } // namespace snark { namespace cuda { namespace k_means {

namespace comma { namespace visiting {
template <>
struct traits< snark::cuda::k_means::input_t< float > >
{
    template < typename K, typename V >
    static void visit( const K &, snark::cuda::k_means::input_t< float > &p, V &v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }

    template < typename K, typename V >
    static void visit( const K &, const snark::cuda::k_means::input_t< float > &p, V &v )
    {
        v.apply( "data", p.data );
        v.apply( "block", p.block );
    }
};
} } // namespace comma { namespace visiting {
