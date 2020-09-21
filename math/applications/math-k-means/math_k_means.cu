// Copyright (c) 2020 Kent Hu

/// @author kent hu

#include "math_k_means.h"
#include "device.h"

#include <cuda_runtime_api.h>
#include <comma/base/types.h>

#include <cfloat>
#include <exception>

namespace snark { namespace cuda { namespace k_means {

__host__ __device__
float square( const float value ) { return value * value; }

__host__ __device__
float squared_euclidean_distance( const size_t size, const float* first, const float* second )
{
    float squared_euclidean_distance = 0.0f;
    for( size_t i = 0; i < size; ++i ) { squared_euclidean_distance += square( first[i] - second[i] ); }
    return squared_euclidean_distance;
}

__global__
static void assign_centroids( const size_t nrows, const size_t ncols, const size_t dataframe_pitch, const float* __restrict__ dataframe,
                              unsigned int* __restrict__ assignments, const unsigned int k, const size_t means_pitch,
                              const float* __restrict__ means, const size_t new_sums_pitch, float* __restrict__ new_sums,
                              unsigned int* __restrict__ counts )
{
    const unsigned int y = blockIdx.x * blockDim.x + threadIdx.x;
    if( y >= nrows ) { return; }
    unsigned int best_cluster = 0;
    float best_distance = FLT_MAX;
    auto dataframe_row = reinterpret_cast< const float* >( reinterpret_cast< const char* >( dataframe ) + y * dataframe_pitch );
    for( unsigned int cluster = 0; cluster < k; ++cluster )
    {
        auto means_row = reinterpret_cast< const float* >( reinterpret_cast< const char* >( means ) + cluster * means_pitch );
        const float distance = squared_euclidean_distance( ncols, dataframe_row, means_row );
        if( distance < best_distance )
        {
            best_distance = distance;
            best_cluster = cluster;
        }
    }
    assignments[y] = best_cluster;
    auto new_sums_row = reinterpret_cast< float* >( reinterpret_cast< char* >( new_sums ) + best_cluster * new_sums_pitch );
    for( unsigned int i = 0; i < ncols; ++i ) { atomicAdd( &new_sums_row[i], dataframe_row[i] ); }
    atomicAdd( &counts[best_cluster], 1 );
}

__global__ static void update_centroids( size_t ncols, size_t means_pitch, float* __restrict__ means, size_t new_sums_pitch, float* __restrict__ new_sums,
                                         unsigned int* __restrict__ counts )
{
    const unsigned int cluster = threadIdx.x;
    const auto count = static_cast< float >( max( 1u, counts[cluster] ));
    auto means_row = reinterpret_cast< float* >( reinterpret_cast< char* >( means ) + cluster * means_pitch );
    auto new_sums_row = reinterpret_cast< float* >( reinterpret_cast< char* >( new_sums ) + cluster * new_sums_pitch );
    for( size_t i = 0; i < ncols; ++i )
    {
        means_row[i] = new_sums_row[i] / count;
        new_sums_row[i] = 0.0f;
    }
    counts[cluster] = 0;
}

template < template < typename > class Matrix >
void assign_centroids( const Matrix< float >& dataframe, const Matrix< float >& centroids, device::array< comma::uint32 >& centroid_assignments,
                       Matrix< float >& centroid_sums, device::array< unsigned int >& centroid_counts, const comma::uint32 number_of_clusters,
                       const size_t threads_per_block )
{
    const unsigned int blocks_per_grid = ( dataframe.rows() + threads_per_block - 1 ) / threads_per_block;
    assign_centroids<<< blocks_per_grid, threads_per_block >>>(
            dataframe.rows(),
            dataframe.cols(),
            dataframe.pitch(),
            dataframe.data(),
            centroid_assignments.data(),
            number_of_clusters,
            centroids.pitch(),
            centroids.data(),
            centroid_sums.pitch(),
            centroid_sums.data(),
            centroid_counts.data()
    );
    CUDA_CHECK_ERRORS( cudaGetLastError() )
    CUDA_CHECK_ERRORS( cudaDeviceSynchronize() )
}

template < template < typename > class Matrix >
void update_centroids( Matrix< float >& centroids, Matrix< float >& centroid_sums, device::array< unsigned int >& centroid_counts,
                       const comma::uint32 number_of_clusters )
{
    update_centroids<<< 1, number_of_clusters >>>(
            centroids.cols(),
            centroids.pitch(),
            centroids.data(),
            centroid_sums.pitch(),
            centroid_sums.data(),
            centroid_counts.data()
    );
    CUDA_CHECK_ERRORS( cudaGetLastError() )
    CUDA_CHECK_ERRORS( cudaDeviceSynchronize() )
}

template void
assign_centroids( const device::matrix< float >& dataframe, const device::matrix< float >& centroids, device::array< comma::uint32 >& centroid_assignments,
                  device::matrix< float >& centroid_sums, device::array< unsigned int >& centroid_counts, const comma::uint32 number_of_clusters,
                  size_t threads_per_block );

template void
assign_centroids( const device::pitched_matrix< float >& dataframe, const device::pitched_matrix< float >& centroids,
                  device::array< comma::uint32 >& centroid_assignments, device::pitched_matrix< float >& centroid_sums,
                  device::array< unsigned int >& centroid_counts, const comma::uint32 number_of_clusters, size_t threads_per_block );

template void
update_centroids( device::matrix< float >& centroids, device::matrix< float >& centroid_sums, device::array< unsigned int >& centroid_counts,
                  const comma::uint32 number_of_clusters );

template void
update_centroids( device::pitched_matrix< float >& centroids, device::pitched_matrix< float >& centroid_sums, device::array< unsigned int >& centroid_counts,
                  const comma::uint32 number_of_clusters );

} } } // namespace snark { namespace cuda { namespace k_means {
