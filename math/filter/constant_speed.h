// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_FILTER_SIMPLE_CONSTANT_SPEED_H
#define SNARK_FILTER_SIMPLE_CONSTANT_SPEED_H

#include <Eigen/Core>

namespace snark{ 

/// point motion model with constant speed    
template< unsigned int N >
struct constant_speed
{

typedef Eigen::Matrix< double, ( N << 1 ), 1 > state_type;
typedef Eigen::Matrix< double, ( N << 1 ), ( N << 1 ) > covariance_type;
    
/// state is [ position velocity ]
struct state
{
    state();
    state( const state_type& s );
    void set_innovation( const state_type& innovation );
    
    static const unsigned int dimension = ( N << 1 );
    state_type state_vector;
    covariance_type covariance;
};

/// motion model
struct model
{
    model( double noiseSigma );

    const covariance_type& jacobian( const state & s, double dt );

    void update_state( state & s, double dt ) const;

    const covariance_type& noise_covariance( double dt );
    
    state_type sigma; /// noise sigma
    covariance_type A;
    covariance_type noise;
};

/// measurement model: absolute position
struct position
{
    typedef Eigen::Matrix< double, N, 1 > position_type;
    
    position( const position_type& p, double sigma );

    const Eigen::Matrix< double, N, ( N << 1 ) >& measurement_jacobian( const state & s ) const
    {
        return jacobian;
    }

    const Eigen::Matrix< double, N, N >& measurement_covariance( const state & s ) const
    {
        return covariance;
    }

    position_type innovation( const state& s ) const
    {
        return ( position_vector - jacobian * s.state_vector );
    }
    
    static const int dimension = N;
    position_type position_vector;
    Eigen::Matrix< double, N, N > covariance;
    Eigen::Matrix< double, N, ( N << 1 ) > jacobian;
};

};

/// constructor, sets state and covariance to zero
template< unsigned int N >
constant_speed< N >::state::state():
    state_vector( state_type::Zero() ),
    covariance( covariance_type::Zero() )
{

}

/// constructor, sets state and covariance to zero
template< unsigned int N >
constant_speed< N >::state::state( const state_type& s ):
    state_vector( s ),
    covariance( covariance_type::Zero() )
{

}

/// update the state from the innovation
template< unsigned int N >
void constant_speed< N >::state::set_innovation( const Eigen::Matrix< double, ( N << 1 ), 1 >& innovation )
{
    state_vector += innovation;
}


/// constructor
/// @param noiseSigma process noise for speed and position
/// TODO separate noise for position and speed ?
template< unsigned int N >
constant_speed< N >::model::model( double noiseSigma ):
    sigma( state_type::Ones() * noiseSigma ),
    A( covariance_type::Identity() ),
    noise( covariance_type::Zero() )
{

}

/// get the jacobian of the motion model
template< unsigned int N >
const typename constant_speed< N >::covariance_type& constant_speed< N >::model::jacobian( const typename constant_speed< N >::state& s, double dt )
{
    A.block( 0, N, N, N ) = Eigen::Matrix< double, N, N >::Identity() * dt;
    return A;
}

/// perform prediction update
template< unsigned int N >
void constant_speed< N >::model::update_state( state& s, double dt ) const
{
    s.state_vector = A * s.state_vector;
}

/// get process noise covariance
template< unsigned int N >
const typename constant_speed< N >::covariance_type& constant_speed< N >::model::noise_covariance( double dt )
{
    for( unsigned int i = 0; i < ( N << 1 ); i++ )
    {
        noise(i,i) = dt * sigma[i];
    }
    return noise;
}

/// constructor
/// @param p measurement position estimate
/// @param sigma measurement noise
template< unsigned int N >
constant_speed< N >::position::position( const position_type& p, double sigma ):
    position_vector( p ),
    covariance( Eigen::Matrix< double, N, N >::Identity()*sigma ),
    jacobian( Eigen::Matrix< double, N, ( N << 1 ) >::Identity() )
{

}

    
} 

#endif // SNARK_FILTER_SIMPLE_CONSTANT_SPEED_H
