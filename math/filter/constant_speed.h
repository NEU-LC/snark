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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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
