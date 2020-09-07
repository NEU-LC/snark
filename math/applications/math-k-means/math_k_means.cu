#include <algorithm>
#include <cfloat>
#include <exception>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/visiting/apply.h>
#include <comma/visiting/visit.h>

__device__ float squared_l2_distance( size_t size, const float *vector_a, const float *vector_b )
{
    float squared_l2_distance = 0;
    for( size_t i = 0; i < size; ++i ) { squared_l2_distance += ( ( vector_a[i] - vector_b[i] ) * ( vector_a[i] - vector_b[i] ) ); }
    return squared_l2_distance;
}

__global__ void assign_clusters( size_t nrows, size_t ncols, size_t dataframe_pitch, const float *__restrict__ dataframe,
                                 unsigned int *__restrict__ assignments, const unsigned int k, size_t means_pitch,
                                 const float *__restrict__ means, size_t new_sums_pitch, float *__restrict__ new_sums,
                                 unsigned int *__restrict__ counts )
{
    const unsigned int y = blockIdx.x * blockDim.x + threadIdx.x;
    if( y >= nrows ) { return; }

    // Make global loads once.
    auto dataframe_row = reinterpret_cast< const float * >( reinterpret_cast< const char * >( dataframe ) + y * dataframe_pitch );

    unsigned int best_cluster = 0;
    float best_distance = FLT_MAX;
    for( unsigned int cluster = 0; cluster < k; ++cluster )
    {
        auto means_row = reinterpret_cast< const float * >( reinterpret_cast< const char * >( means ) + cluster * means_pitch );
        const float distance = squared_l2_distance( ncols, dataframe_row, means_row );
        if( distance < best_distance )
        {
            best_distance = distance;
            best_cluster = cluster;
        }
    }
    unsigned int *assignments_row = assignments + y;
    *assignments_row = best_cluster;
    auto new_sums_row = reinterpret_cast< float * >( reinterpret_cast< char * >( new_sums ) + best_cluster * new_sums_pitch );
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
    auto new_sums_row = reinterpret_cast< float * >( reinterpret_cast< char * >( new_sums ) + cluster * new_sums_pitch );
    for( size_t i = 0; i < ncols; ++i )
    {
        means_row[i] = new_sums_row[i] / count;
        new_sums_row[i] = 0;
    }
    counts[cluster] = 0;
}

namespace snark { namespace k_means { namespace cuda {

static boost::optional< size_t > ncols;

template < typename FloatingPoint >
struct input_t
{
    std::vector< FloatingPoint > vector;
    unsigned int block = 0;

    input_t() : vector( *ncols ) {};
};

namespace device {
template < typename FloatingPoint >
struct dataframe
{
    dataframe() = delete;

    dataframe( size_t nrows, size_t ncols ) : nrows( nrows ), ncols( ncols )
    {
        cudaError_t err = cudaMallocPitch( &data, &pitch, ncols * sizeof( FloatingPoint ), nrows );
        if( err != cudaSuccess )
        {
            COMMA_THROW( comma::exception, "cuda: malloc failed -> " << cudaGetErrorString( err ) << ' ' << cudaGetErrorName( err ) );
        }
        err = cudaMemset2D( data, pitch, 0, ncols * sizeof( FloatingPoint ), nrows );
        if( err != cudaSuccess )
        {
            cudaFree( data );
            COMMA_THROW( comma::exception, "cuda: memset failed -> " << cudaGetErrorString( err ) << ' ' << cudaGetErrorName( err ) );
        }
    }

    dataframe( size_t nrows, size_t ncols, const std::vector< FloatingPoint > &h_dataframe ) : nrows( nrows ), ncols( ncols )
    {
        cudaError_t err = cudaMallocPitch( &data, &pitch, ncols * sizeof( FloatingPoint ), nrows );
        if( err != cudaSuccess )
        {
            COMMA_THROW( comma::exception, "cuda: malloc failed -> " << cudaGetErrorString( err ) << ' ' << cudaGetErrorName( err ) );
        }
        to_device( h_dataframe );
    }

    dataframe( const dataframe &other ) = delete;

    dataframe &operator=( const dataframe &other ) = delete;

    dataframe( dataframe &&other ) = delete;

    dataframe &operator=( dataframe &&other ) = delete;

    ~dataframe() { cudaFree( data ); }

    size_t bytes() const { return sizeof( FloatingPoint ) * ncols * nrows; }

    void to_device( const std::vector< FloatingPoint > &h_dataframe )
    {
        cudaError_t err = cudaMemcpy2D( data, pitch, h_dataframe.data(), ncols * sizeof( FloatingPoint ), ncols * sizeof( FloatingPoint ), nrows, cudaMemcpyHostToDevice );
        if( err != cudaSuccess )
        {
            cudaFree( data );
            COMMA_THROW( comma::exception, "cuda: memcpy failed -> " << cudaGetErrorString( err ) << ' ' << cudaGetErrorName( err ) );
        }
    }

    void to_host( std::vector< FloatingPoint > &h_dataframe ) const
    {
        cudaError_t err = cudaMemcpy2D( h_dataframe.data(), ncols * sizeof( FloatingPoint ), data, pitch, ncols * sizeof( FloatingPoint ), nrows, cudaMemcpyDeviceToHost );
        if( err != cudaSuccess )
        {
            cudaFree( data );
            COMMA_THROW( comma::exception, "cuda: memcpy failed -> " << cudaGetErrorString( err ) << ' ' << cudaGetErrorName( err ) );
        }
    }

    friend std::ostream &operator<<( std::ostream &os, const dataframe &df )
    {
        std::vector< FloatingPoint > h_data( df.nrows * df.ncols, 0 );
        df.to_host( h_data );
        for( size_t y = 0; y < df.nrows; ++y )
        {
            std::string s;
            for( size_t x = 0; x < df.ncols; ++x )
            {
                os << s << h_data[y * df.ncols + x];
                s = ' ';
            }
            os << '\n';
        }
        return os;
    }

    FloatingPoint *data = nullptr;
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

    void run( std::vector< float > &h_dataframe, const std::vector< std::string > &input_lines ) const
    {
        const size_t nrows = h_dataframe.size() / *ncols;
        device::dataframe< float > d_dataframe( nrows, *ncols, h_dataframe );
        device::dataframe< unsigned int > d_assignments( 1, nrows );
        device::dataframe< float > d_means( number_of_clusters, *ncols );
        device::dataframe< float > d_sums( number_of_clusters, *ncols );
        device::dataframe< unsigned int > d_counts( 1, number_of_clusters );

        std::vector< unsigned int > h_assignments( 1 * nrows, 0 );
        std::vector< float > h_means( number_of_clusters * *ncols, 0 );

        std::mt19937 generator( std::random_device{}() );
        std::vector< float > centroids( number_of_clusters * *ncols, 0 );
        std::uniform_int_distribution< size_t > indices( 0, nrows - 1 );

        constexpr unsigned int threads_per_block = 1024;
        const unsigned int blocks_per_grid = ( nrows + threads_per_block - 1 ) / threads_per_block;
        for( unsigned int runs = 0; runs < number_of_runs; ++runs )
        {
            for( size_t y = 0; y < number_of_clusters; ++y )
            {
                for( size_t x = 0; x < *ncols; ++x )
                {
                    centroids[y * *ncols + x] = h_dataframe[indices( generator ) * *ncols + x];
                }
            }
            d_means.to_device( centroids );
            for( unsigned int iteration = 0; iteration < max_iterations; ++iteration )
            {
                assign_clusters<<<blocks_per_grid, threads_per_block>>>(
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
                compute_new_means_and_reset<<<1, number_of_clusters>>>(
                        *ncols,
                        d_means.pitch,
                        d_means.data,
                        d_sums.pitch,
                        d_sums.data,
                        d_counts.data
                );
                cudaDeviceSynchronize();
            }
        }

        d_assignments.to_host( h_assignments );
        d_means.to_host( h_means );

        for( size_t y = 0; y < nrows; ++y )
        {
            const unsigned int cluster_assignment = h_assignments[y];
            std::cout << input_lines[y];
            for( size_t x = 0; x < *ncols; ++x )
            {
                std::cout << ',' << h_means[cluster_assignment * ( *ncols ) + x];
            }
            std::cout << ',' << cluster_assignment << std::endl;
        }
    }
};

int run( const comma::command_line_options &options )
{
    if( options.exists( "--verbose,-v" ) ) { std::cerr << "math-k-means: running cuda version" << std::endl; }
    comma::csv::options csv = comma::csv::options( options, "data" );
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
        dataframe.insert( end( dataframe ), begin( p.vector ), end( p.vector ) );
        blocks.emplace_back( p.block );
        input_lines.emplace_back( first );
    }
    k_means operation{ tolerance, max_iterations, number_of_runs, number_of_clusters };
    while( istream.ready() || std::cin.good() )
    {
        const snark::k_means::cuda::input_t< float > *p = istream.read();
        if( !blocks.empty() && ( !p || blocks.front() != p->block ) && !dataframe.empty() )
        {
            operation.run( dataframe, input_lines );
            dataframe.clear();
            blocks.clear();
            input_lines.clear();
        }
        if( !p ) { break; }
        dataframe.insert( end( dataframe ), begin( p->vector ), end( p->vector ) );
        blocks.emplace_back( p->block );
        input_lines.emplace_back( istream.last() );
    }
    return 0;
}
} } } // namespace snark { namespace k_means { namespace cuda {

namespace comma { namespace visiting {
template <>
struct traits< snark::k_means::cuda::input_t< float > >
{
    template < typename K, typename V >
    static void visit( const K &, snark::k_means::cuda::input_t< float > &p, V &v )
    {
        v.apply( "data", p.vector );
        v.apply( "block", p.block );
    }

    template < typename K, typename V >
    static void visit( const K &, const snark::k_means::cuda::input_t< float > &p, V &v )
    {
        v.apply( "data", p.vector );
        v.apply( "block", p.block );
    }
};
} } // namespace comma { namespace visiting {
