#include "math_k_means.cuh"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <exception>
#include <iostream>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include <comma/base/exception.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

__host__ __device__ static float squared_euclidean_distance( size_t size, const float *vector_a, const float *vector_b )
{
    float squared_euclidean_distance = 0.0f;
    for( size_t i = 0; i < size; ++i )
    {
        squared_euclidean_distance += ( ( vector_a[i] - vector_b[i] ) * ( vector_a[i] - vector_b[i] ) );
    }
    return squared_euclidean_distance;
}

__global__ static void
assign_clusters( size_t nrows, size_t ncols, size_t dataframe_pitch, const float *__restrict__ dataframe,
                 unsigned int *__restrict__ assignments, const unsigned int k, size_t means_pitch,
                 const float *__restrict__ means, size_t new_sums_pitch, float *__restrict__ new_sums,
                 unsigned int *__restrict__ counts )
{
    const unsigned int y = blockIdx.x * blockDim.x + threadIdx.x;
    if( y >= nrows ) { return; }
    unsigned int best_cluster = 0;
    float best_distance = FLT_MAX;
    auto dataframe_row = reinterpret_cast< const float * >( reinterpret_cast< const char * >( dataframe ) + y * dataframe_pitch );
    for( unsigned int cluster = 0; cluster < k; ++cluster )
    {
        auto means_row = reinterpret_cast< const float * >( reinterpret_cast< const char * >( means ) + cluster * means_pitch );
        const float distance = squared_euclidean_distance( ncols, dataframe_row, means_row );
        if( distance < best_distance )
        {
            best_distance = distance;
            best_cluster = cluster;
        }
    }
    assignments[y] = best_cluster;
    auto new_sums_row = reinterpret_cast< float * >( reinterpret_cast< char * >( new_sums ) + best_cluster * new_sums_pitch );
    for( unsigned int i = 0; i < ncols; ++i ) { atomicAdd( &new_sums_row[i], dataframe_row[i] ); }
    atomicAdd( &counts[best_cluster], 1 );
}

__global__ static void
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
        new_sums_row[i] = 0.0f;
    }
    counts[cluster] = 0;
}

namespace snark { namespace cuda {

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
        cudaMemcpy2D( data, pitch, h_matrix.data(), ncols * sizeof( Numeric ), ncols * sizeof( Numeric ), nrows, cudaMemcpyHostToDevice );
    }

    void to_host( std::vector< Numeric > &h_matrix ) const noexcept
    {
        cudaMemcpy2D( h_matrix.data(), ncols * sizeof( Numeric ), data, pitch, ncols * sizeof( Numeric ), nrows, cudaMemcpyDeviceToHost );
    }

    Numeric *data = nullptr;
    size_t nrows;
    size_t ncols;
    size_t pitch = 0;
};

} // namespace device {

namespace k_means {

std::tuple< std::vector< float >, std::vector< comma::uint32 > >
k_means::run_on_block( const std::vector< float > &h_matrix ) const
{
    const size_t nrows = h_matrix.size() / ncols_;

    // device arrays/matrices
    device::matrix< float > d_dataframe( nrows, ncols_, h_matrix );
    device::matrix< float > d_means( number_of_clusters_, ncols_ );
    device::matrix< float > d_sums( number_of_clusters_, ncols_ );
    d_sums.fill( 0.0f );
    device::array< comma::uint32 > d_assignments( nrows );
    device::array< unsigned int > d_counts( number_of_clusters_ );
    d_counts.fill( 0 );

    // host vectors
    std::vector< float > h_means( number_of_clusters_ * ncols_ );
    std::vector< comma::uint32 > h_assignments( nrows );

    // save runs to choose best centroid
    std::vector< float > all_scores;
    std::vector< decltype( h_means ) > all_centroids;
    std::vector< decltype( h_assignments ) > all_centroid_assignments;

    cudaDeviceProp prop{};
    cudaGetDeviceProperties( &prop, 0 );
    device::check_errors();
    const unsigned int blocks_per_grid = ( nrows + prop.maxThreadsPerBlock - 1 ) / prop.maxThreadsPerBlock;
    for( unsigned int run = 0; run < number_of_runs_; ++run )
    {
        d_means.to_device( initialize_centroids( h_matrix ) );
        float difference = std::numeric_limits< float >::max();
        for( unsigned int iteration = 0; iteration < max_iterations_; ++iteration )
        {
            assign_clusters<<< blocks_per_grid, prop.maxThreadsPerBlock >>>(
                    d_dataframe.nrows,
                    d_dataframe.ncols,
                    d_dataframe.pitch,
                    d_dataframe.data,
                    d_assignments.data,
                    number_of_clusters_,
                    d_means.pitch,
                    d_means.data,
                    d_sums.pitch,
                    d_sums.data,
                    d_counts.data
            );
            compute_new_means_and_reset<<< 1, number_of_clusters_ >>>(
                    ncols_,
                    d_means.pitch,
                    d_means.data,
                    d_sums.pitch,
                    d_sums.data,
                    d_counts.data
            );
            device::check_errors();
            device::check_async_errors();
            if( tolerance_ < 0 ) { continue; }
            std::vector< float > new_means( number_of_clusters_ * ncols_ ); // temp buffer for calculating tolerance difference
            d_means.to_host( new_means );
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
        d_assignments.to_host( h_assignments );
        if( tolerance_ < 0 || difference > tolerance_ * tolerance_ ) { d_means.to_host( h_means ); }
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
        all_scores.push_back( score );
        all_centroids.push_back( h_means );
        all_centroid_assignments.push_back( h_assignments );
    }
    const auto& best_index = std::distance( all_scores.begin(), std::min_element( all_scores.begin(), all_scores.end() ) );
    return std::make_tuple( all_centroids[best_index], all_centroid_assignments[best_index] );
}

std::vector< float > k_means::initialize_centroids( const std::vector< float > &h_matrix ) const
{
    static std::mt19937 generator( std::random_device{}() );
    static std::vector< float > init_centroids( number_of_clusters_ * ncols_ );
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

} // namespace k_means {

} } // namespace snark { namespace cuda {
