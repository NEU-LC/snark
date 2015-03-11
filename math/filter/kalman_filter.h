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
// 3. Neither the name of the University of Sydney nor the
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


#ifndef SNARK_FILTER_KALMAN_FILTER_H
#define SNARK_FILTER_KALMAN_FILTER_H

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace snark{ 
    
/// generic kalman filter
template< class State, class Model >
class kalman_filter
{
public:
    /// constructor
    /// @param state container for state and covariance
    /// @param model motion model
    kalman_filter( const State& initialstate, Model& model ) : m_state( initialstate ), m_model( model ) {}

    /// predict step
    /// @param deltaT time in seconds since last prediction
    void predict( double deltaT )
    {
        const Eigen::Matrix< double, State::dimension, State::dimension >& A = m_model.jacobian( m_state, deltaT );
        m_state.covariance = A * m_state.covariance * A.transpose() + m_model.noise_covariance( deltaT );
        m_model.update_state( m_state, deltaT );
    }

    /// update step
    /// @param m measurement model with jacobian and covariance
    template< class Measurement >
    void update( const Measurement& m )
    {
        const Eigen::Matrix<double,Measurement::dimension,State::dimension>& H = m.measurement_jacobian( m_state );
        const Eigen::Matrix<double,Measurement::dimension,Measurement::dimension>& R = m.measurement_covariance( m_state );
        const Eigen::Matrix<double,Measurement::dimension,1>& innovation = m.innovation( m_state );
        
        const Eigen::Matrix<double,State::dimension, Measurement::dimension> PHt = m_state.covariance * H.transpose();
        const Eigen::LDLT< Eigen::Matrix<double,Measurement::dimension,Measurement::dimension> > S = ( H * PHt + R ).ldlt();
        
        m_state.covariance -= PHt * S.solve( PHt.transpose() );
        m_state.set_innovation( PHt * S.solve( innovation ) );
    }

    /// get state
    const State& state() const { return m_state; }

private:
    State m_state; /// state
    Model& m_model; /// process model
};

} 

#endif // SNARK_FILTER_KALMAN_FILTER_H
