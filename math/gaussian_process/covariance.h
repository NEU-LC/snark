#ifndef SNARK_GAUSSIAN_PROCESS_COVARIANCE_
#define SNARK_GAUSSIAN_PROCESS_COVARIANCE_

#include <Eigen/Core>

namespace snark{ 

class squared_exponential_covariance
{
    public:
        squared_exponential_covariance( double length_scale
                                      , double signal_variance
                                      , double data_variance );

        double covariance( const Eigen::VectorXd& v, const Eigen::VectorXd& w ) const;
        
        double self_covariance() const;

    private:
        double signal_variance_;
        double data_variance_;
        double factor_;
        double self_covariance_;
};

} // namespace snark{

#endif // #ifndef SNARK_GAUSSIAN_PROCESS_COVARIANCE_
