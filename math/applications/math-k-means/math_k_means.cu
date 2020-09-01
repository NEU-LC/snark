#include <cfloat>
#include <iostream>
#include <vector>

#include <exception>
#include <string>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/visiting/apply.h>
#include <comma/visiting/visit.h>

__device__ float squared_l2_distance( float x_1, float y_1, float z_1, float x_2, float y_2, float z_2 )
{
    return (x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2) + (z_1 - z_2) * (z_1 - z_2);
}

__global__ void assign_clusters(const float* __restrict__ data_x,
                                const float* __restrict__ data_y,
                                const float* __restrict__ data_z,
                                unsigned int data_size,
                                unsigned int* __restrict__ assignments,
                                const float* __restrict__ means_x,
                                const float* __restrict__ means_y,
                                const float* __restrict__ means_z,
                                float* __restrict__ new_sums_x,
                                float* __restrict__ new_sums_y,
                                float* __restrict__ new_sums_z,
                                unsigned int k,
                                unsigned int* __restrict__ counts)
{
    extern __shared__ float shared_means[];

    const unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= data_size) return;

    if (threadIdx.x < k) {
        shared_means[threadIdx.x] = means_x[threadIdx.x];
        shared_means[k + threadIdx.x] = means_y[threadIdx.x];
        shared_means[k + k + threadIdx.x] = means_z[threadIdx.x];
    }

    __syncthreads();

    // Make global loads once.
    const float x = data_x[index];
    const float y = data_y[index];
    const float z = data_z[index];

    float best_distance = FLT_MAX;
    unsigned int best_cluster = 0;
    for (unsigned int cluster = 0; cluster < k; ++cluster) {
        const float distance = squared_l2_distance(x, y, z, shared_means[cluster], shared_means[k + cluster], shared_means[k + k + cluster]);
        if ( distance < best_distance ) {
            best_distance = distance;
            best_cluster = cluster;
        }
    }
    assignments[index] = best_cluster;
    atomicAdd( &new_sums_x[best_cluster], x );
    atomicAdd( &new_sums_y[best_cluster], y );
    atomicAdd( &new_sums_z[best_cluster], z );
    atomicAdd( &counts[best_cluster], 1 );
}

__global__ void compute_new_means_and_reset( float* __restrict__ means_x,
                                             float* __restrict__ means_y,
                                             float* __restrict__ means_z,
                                             float* __restrict__ new_sum_x,
                                             float* __restrict__ new_sum_y,
                                             float* __restrict__ new_sum_z,
                                             unsigned int* __restrict__ counts )
{
    const unsigned int cluster = threadIdx.x;
    const unsigned int count = max( 1, counts[cluster] );
    means_x[cluster] = new_sum_x[cluster] / count;
    means_y[cluster] = new_sum_y[cluster] / count;
    means_z[cluster] = new_sum_z[cluster] / count;

    new_sum_x[cluster] = 0;
    new_sum_y[cluster] = 0;
    new_sum_z[cluster] = 0;
    counts[cluster] = 0;
}

namespace snark { namespace k_means { namespace cuda {

struct input_t
{
    float x;
    float y;
    float z;
    comma::uint32 block = 0;
};

struct data {
    explicit data( unsigned int size ) : size( size ), bytes( size * sizeof( float )) {
        if( cudaMalloc( &x, bytes ) != cudaSuccess ) { COMMA_THROW( comma::exception, "math-k-means: cuda: malloc failed") }
        if( cudaMalloc( &y, bytes ) != cudaSuccess ) { cudaFree( x ); COMMA_THROW( comma::exception, "math-k-means: cuda: malloc failed"); }
        if( cudaMalloc( &z, bytes ) != cudaSuccess ) { cudaFree( x ); cudaFree( y ); COMMA_THROW( comma::exception, "math-k-means: cuda: malloc failed"); }
        if( cudaMemset( x, 0, bytes ) != cudaSuccess ||
            cudaMemset( y, 0, bytes ) != cudaSuccess ||
            cudaMemset( z, 0, bytes ) != cudaSuccess )
        {
            cudaFree( x );
            cudaFree( y );
            cudaFree( z );
            COMMA_THROW( comma::exception, "math-k-means: cuda: memset failed");
        }
    }

    data( unsigned int size, std::vector< float > &h_x, std::vector< float > &h_y, std::vector< float > &h_z ) : size( size ), bytes( size * sizeof( float )) {
        if( cudaMalloc( &x, bytes ) != cudaSuccess ) { COMMA_THROW( comma::exception, "math-k-means: cuda: malloc failed") }
        if( cudaMalloc( &y, bytes ) != cudaSuccess ) { cudaFree( x ); COMMA_THROW( comma::exception, "math-k-means: cuda: malloc failed"); }
        if( cudaMalloc( &z, bytes ) != cudaSuccess ) { cudaFree( x ); cudaFree( y ); COMMA_THROW( comma::exception, "math-k-means: cuda: malloc failed"); }
        if( cudaMemcpy( x, h_x.data(), bytes, cudaMemcpyHostToDevice ) != cudaSuccess ||
            cudaMemcpy( y, h_y.data(), bytes, cudaMemcpyHostToDevice ) != cudaSuccess ||
            cudaMemcpy( z, h_z.data(), bytes, cudaMemcpyHostToDevice ) != cudaSuccess )
        {
            cudaFree( x );
            cudaFree( y );
            cudaFree( z );
            COMMA_THROW( comma::exception, "math-k-means: cuda: memcpy failed");
        }
    }

    ~data() {
        cudaFree( x );
        cudaFree( y );
        cudaFree( z );
    }

    float *x{ nullptr };
    float *y{ nullptr };
    float *z{ nullptr };
    unsigned int size{ 0 };
    unsigned int bytes{ 0 };
};

struct counts {
    explicit counts( unsigned int k ) : k( k )
    {
        if( cudaMalloc( &counts_, k * sizeof( unsigned int )) != cudaSuccess ) { COMMA_THROW( comma::exception, "math-k-means: cuda: malloc failed") }
        if( cudaMemset( counts_, 0, k * sizeof( unsigned int )) != cudaSuccess ) { cudaFree( counts_ ); COMMA_THROW( comma::exception, "math-k-means: cuda: memset failed") }
    }

    ~counts() { cudaFree( counts_ ); }

    unsigned int *counts_{ nullptr };
    unsigned int k{ 0 };
};

struct k_means {
    const double tolerance;
    const unsigned int max_iterations;
    const unsigned int number_of_runs;
    const comma::uint32 number_of_clusters;
    k_means( double tolerance,
             unsigned int max_iterations,
             unsigned int number_of_runs,
             comma::uint32 number_of_clusters ) noexcept
         : tolerance( tolerance )
         , max_iterations( max_iterations )
         , number_of_runs( number_of_runs )
         , number_of_clusters( number_of_clusters ){}

    void run( std::vector< float > &x,
              std::vector< float > &y,
              std::vector< float > &z,
              const std::vector< std::string >& input_lines )
    {
        const size_t number_of_elements = x.size();

        data d_data( number_of_elements, x, y, z );
        data d_means( number_of_clusters, x, y, z );
        data d_sums( number_of_clusters );
        counts d_assignments( number_of_elements );
        counts d_counts( number_of_clusters );

        const int threads_per_block = 1024;
        const int blocks_per_grid = ( number_of_elements + threads_per_block - 1 ) / threads_per_block;
        const int shared_memory = d_means.bytes * 3;

        for( unsigned int runs = 0; runs < number_of_runs; ++runs )
        {
            for( unsigned int iteration = 0; iteration < max_iterations; ++iteration )
            {
                assign_clusters<<<blocks_per_grid, threads_per_block, shared_memory>>>( d_data.x,
                                                                     d_data.y,
                                                                     d_data.z,
                                                                     d_data.size,
                                                                     d_assignments.counts_,
                                                                     d_means.x,
                                                                     d_means.y,
                                                                     d_means.z,
                                                                     d_sums.x,
                                                                     d_sums.y,
                                                                     d_sums.z,
                                                                     number_of_clusters,
                                                                     d_counts.counts_ );
                cudaDeviceSynchronize();
//                if( cudaGetLastError() != cudaSuccess ) { COMMA_THROW( comma::exception, "math-k-means: cuda: assigning clusters failed" ); }
                compute_new_means_and_reset<<<1, number_of_clusters>>>( d_means.x,
                                                                        d_means.y,
                                                                        d_means.z,
                                                                        d_sums.x,
                                                                        d_sums.y,
                                                                        d_sums.z,
                                                                        d_counts.counts_ );
                cudaDeviceSynchronize();
//                if( cudaGetLastError() != cudaSuccess ) { COMMA_THROW( comma::exception, "math-k-means: cuda: computing new means failed" ); }
            }
        }
        std::vector< float > h_means_x( number_of_clusters, 0 );
        std::vector< float > h_means_y( number_of_clusters, 0 );
        std::vector< float > h_means_z( number_of_clusters, 0 );
        cudaMemcpy( h_means_x.data(), d_means.x, d_means.bytes, cudaMemcpyDeviceToHost );
        cudaMemcpy( h_means_y.data(), d_means.y, d_means.bytes, cudaMemcpyDeviceToHost );
        cudaMemcpy( h_means_z.data(), d_means.z, d_means.bytes, cudaMemcpyDeviceToHost );

        std::vector< unsigned int > h_assignments( number_of_elements, 1 );
        if( cudaMemcpy( h_assignments.data(), d_assignments.counts_, number_of_elements * sizeof( unsigned int ), cudaMemcpyDeviceToHost ) != cudaSuccess )
        {
            std::cerr << "copy failed\n";
        };

        for( size_t i = 0; i < input_lines.size(); ++i )
        {
            const auto assignment = h_assignments[i];
            std::cout << input_lines[i] << ',' << h_means_x[assignment] << ',' << h_means_y[assignment] << ',' << h_means_z[assignment] << ',' << assignment << std::endl;
        }
    }
};

int run_( const comma::command_line_options &options ) {
    std::cerr << "math-k-means: running cuda version" << std::endl;
    comma::csv::options csv = comma::csv::options( options, "x,y,z" );
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
    const auto tolerance = options.value< double >( "--tolerance", 1e-4 );
    if( tolerance <= 0 )
    {
        std::cerr << "math-k-means: got --tolerance=" << tolerance << ", --tolerance should be greater than 0" << std::endl;
        return 1;
    }
    comma::csv::input_stream< input_t > istream( std::cin, csv );
    std::vector< float > x;
    std::vector< float > y;
    std::vector< float > z;
    std::vector< comma::uint32 > blocks;
    std::vector< std::string > input_lines;
    k_means operation{ tolerance, max_iterations, number_of_runs, number_of_clusters };
    while( istream.ready() || std::cin.good() )
    {
        const snark::k_means::cuda::input_t *p = istream.read();
        if( !blocks.empty() && ( !p || blocks.front() != p->block ) )
        {
            operation.run( x, y, z, input_lines );
            x.clear();
            y.clear();
            z.clear();
            input_lines.clear();
        }
        if ( !p ) { break; }
        x.emplace_back( p->x );
        y.emplace_back( p->y );
        z.emplace_back( p->z );
        blocks.emplace_back( p->block );
        input_lines.emplace_back( istream.last() );
    }
    return 0;
}
} } } // namespace snark { namespace k_means { namespace cuda {

namespace comma { namespace visiting {

template <> struct traits< snark::k_means::cuda::input_t >
{
    template < typename K, typename V > static void visit( const K&, snark::k_means::cuda::input_t& p, V& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const snark::k_means::cuda::input_t& p, V& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {
