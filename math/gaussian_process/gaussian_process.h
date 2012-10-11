#ifndef SNARK_GAUSSIAN_PROCESS_
#define SNARK_GAUSSIAN_PROCESS_

#include <boost/function.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace snark{ 

/// gaussian process
class gaussian_process
{
    public:
        /// covariance functor type
        typedef boost::function< double ( const Eigen::VectorXd&, const Eigen::VectorXd& ) > covariance;

        /// constructor
        gaussian_process( const Eigen::MatrixXd& domains
                       , const Eigen::VectorXd& targets
                       , const gaussian_process::covariance& covariance
                       , double self_covariance = 0 );

        /// evaluate
        void evaluate( const Eigen::MatrixXd& domains
                     , Eigen::VectorXd& means
                     , Eigen::VectorXd& variances ) const;

        /// evaluate a single domain, return mean-variance pair
        std::pair< double, double > evaluate( const Eigen::MatrixXd& domain ) const;

    private:
        Eigen::MatrixXd domains_; //!< domain locations corresponding to targets
        Eigen::VectorXd targets_; //!< targets
        covariance covariance_;
        double self_covariance_;
        double offset_;
        Eigen::MatrixXd K_; //!< the inverse of (Kxx + noiseVariance*I)
        Eigen::LLT< Eigen::MatrixXd  > L_; //!< cholesky factorization of the covariance Kxx
        Eigen::VectorXd alpha_;
};

} // namespace snark{

#endif // #ifndef SNARK_GAUSSIAN_PROCESS_
