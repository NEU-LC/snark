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
