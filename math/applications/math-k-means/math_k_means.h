// Copyright (c) 2020 Kent Hu

/// @author kent hu

#pragma once

#include "device.h"

#include <comma/base/types.h>

#include <vector>
#include <tuple>

namespace snark { namespace cuda { namespace k_means {

__host__ __device__
float squared_euclidean_distance( size_t size, const float* vector_a, const float* vector_b );

template < template < typename > class Matrix >
void assign_centroids( const Matrix< float >& dataframe, const Matrix< float >& centroids, device::array< comma::uint32 >& centroid_assignments,
                       Matrix< float >& centroid_sums, device::array< unsigned int >& centroid_counts, comma::uint32 number_of_clusters, size_t threads_per_block );

template < template < typename > class Matrix >
void update_centroids( Matrix< float >& centroids, Matrix< float >& centroid_sums, device::array< unsigned int >& centroid_counts,
                       comma::uint32 number_of_clusters );

} } } // namespace snark { namespace cuda { namespace k_means {
