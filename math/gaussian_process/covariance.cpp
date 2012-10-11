#include <cmath>
#include <Eigen/Eigen>
#include "./covariance.h"

namespace snark{ 

squared_exponential_covariance::squared_exponential_covariance( double length_scale
                                                              , double signal_variance
                                                              , double data_variance )
    : signal_variance_( signal_variance )
    , data_variance_( data_variance )
    , factor_( -1.0 / ( 2.0 * std::sqrt( length_scale ) ) )
    , self_covariance_( signal_variance + data_variance )
{
}

double squared_exponential_covariance::covariance( const Eigen::VectorXd& v, const Eigen::VectorXd& w ) const
{
    const Eigen::VectorXd& diff = v - w;
    return signal_variance_ * std::exp( factor_ * diff.dot( diff ) );
}

double squared_exponential_covariance::self_covariance() const { return self_covariance_; }

} // namespace snark{ { namespace Robotics {
