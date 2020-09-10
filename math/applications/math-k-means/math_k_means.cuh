//
// Created by kent on 27/8/20.
//

#pragma once

#include <vector>
#include <tuple>

#include <comma/base/types.h>

namespace snark { namespace cuda { namespace k_means {

class k_means
{
public:
    k_means( float tolerance, unsigned int max_iterations, unsigned int number_of_runs,
             comma::uint32 number_of_clusters, unsigned int size ) noexcept : tolerance_( tolerance ),
                                                                              max_iterations_( max_iterations ),
                                                                              number_of_runs_( number_of_runs ),
                                                                              number_of_clusters_( number_of_clusters ),
                                                                              ncols_( size ) {}

    std::tuple< std::vector< float >, std::vector< comma::uint32 > >
    run_on_block( const std::vector< float >& dataframe ) const;

private:
    std::vector< float > initialize_centroids( const std::vector< float >& h_matrix ) const;

    const float tolerance_;
    const unsigned int max_iterations_;
    const unsigned int number_of_runs_;
    const unsigned int number_of_clusters_;
    const unsigned int ncols_;
};

} } } // namespace snark { namespace cuda { namespace k_means {
